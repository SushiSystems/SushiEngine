// The regional nest's shared vocabulary: its parameter block, its grid, its base state, and
// the thermodynamic relations every stage of the step is written in terms of.
//
// docs/slop/atmosphere_system.md §6. Included by every atmosphere_*.comp. The C++ side of the
// same relations lives in include/SushiEngine/render/atmosphere_nest.hpp, which explains why
// the *formulas* are mirrored and the *numbers* are not: every constant below is read out of
// the uploaded block, so there is exactly one set of values and the mirror is only ever a
// restatement of the maths. The two files name each other; neither is edited alone.
//
// **Staggered Arakawa C-grid.** Scalars (theta, q_v, q_c, q_r, pi') sit at cell centres; the
// velocity components sit on the faces they cross — u on the x-faces, v on the z-faces, w on
// the y-faces (the engine's world is +Y up, so the model's vertical index runs along Y). Index
// i of u means the face at the *low* side of cell i. Staggering is what makes the divergence
// and the pressure gradient exact adjoints of each other, and it is why the pressure solve
// cannot develop the checkerboard mode a collocated grid produces.

// Parameters, uploaded once per step. std140 packs a flat run of floats and ints tightly (only
// vec3 and arrays carry surprises), so this is written in named fields rather than packed
// vec4s — it is read in a dozen places and the packing would cost more than it saves. Mirrors
// AtmosphereNestPass::NestParams field for field.
layout(set = 0, binding = 0) uniform NestParams
{
    // Thermodynamics of moist air.
    float gas_constant_dry;
    float gas_constant_vapour;
    float specific_heat_pressure;
    float latent_heat_vaporization;
    float gravity;
    float reference_pressure;
    float water_density;

    // Base state.
    float surface_temperature;
    float surface_pressure;
    float lapse_rate;
    float tropopause_altitude;
    float surface_humidity;
    float humidity_scale_height;

    // Dynamics.
    float eddy_viscosity;
    float sponge_depth;
    float sponge_rate;
    float boundary_relaxation;
    float thermal_seed_amplitude;
    float coriolis;
    float convective_velocity_scale;

    // Microphysics.
    float autoconversion_rate;
    float autoconversion_threshold;
    float accretion_rate;
    float accretion_exponent;
    float rain_evaporation_rate;
    float fall_speed_coefficient;
    float fall_speed_exponent;
    float droplet_effective_radius;

    // Surface forcing.
    float surface_sensible_flux;   // peak, at solar noon with the sun overhead
    float surface_latent_flux;     // peak, likewise
    float surface_night_flux;      // net radiative cooling while the sun is down, positive
    float solar_elevation_sine;    // sine of the sun's elevation; negative below the horizon

    // Grid and step.
    float spacing;      // horizontal cell size, metres
    float domain_top;   // domain top above the surface, metres
    float dt;           // this step's duration, seconds of game time
    float elapsed;      // total game seconds simulated, for the thermal seed's phase
    int cells_x;
    int cells_z;
    int levels;
    int boundary_zone;  // Davies relaxation zone width, cells
    int step_index;     // monotonically increasing step counter, decorrelates the seed
} nest;

const float ATMOSPHERE_VERTICAL_STRETCH = 1.5;

// ---- Grid -----------------------------------------------------------------------------

int nest_level_count() { return max(nest.levels, 1); }

// Altitude of level k's centre, metres above the surface. Stretched so the boundary layer and
// cloud base get the resolution and the anvil does not: at 48 levels over 18 km the spacing
// runs ~54 m at the ground to ~560 m aloft.
float nest_level_altitude(float level)
{
    float fraction = (level + 0.5) / float(nest_level_count());
    return nest.domain_top * pow(max(fraction, 0.0), ATMOSPHERE_VERTICAL_STRETCH);
}

// Altitude of the *face* below level k — where w lives.
float nest_face_altitude(float level)
{
    float fraction = level / float(nest_level_count());
    return nest.domain_top * pow(max(fraction, 0.0), ATMOSPHERE_VERTICAL_STRETCH);
}

// Thickness of level k, metres: the spacing every vertical derivative divides by.
float nest_level_thickness(int level)
{
    return max(nest_face_altitude(float(level) + 1.0) - nest_face_altitude(float(level)), 1.0);
}

// Clamps a cell index into the domain. The lateral boundary is a relaxation zone rather than a
// wall, so clamping only ever catches the halo a stencil reaches past the edge.
ivec3 nest_clamp(ivec3 c)
{
    return clamp(c, ivec3(0), ivec3(nest.cells_x - 1, nest_level_count() - 1, nest.cells_z - 1));
}

// ---- Base state -----------------------------------------------------------------------

float nest_base_temperature(float altitude)
{
    float capped = min(altitude, nest.tropopause_altitude);
    return nest.surface_temperature - nest.lapse_rate * capped;
}

float nest_base_pressure(float altitude)
{
    float exponent = nest.gravity / (nest.gas_constant_dry * nest.lapse_rate);
    float capped = min(altitude, nest.tropopause_altitude);
    float ratio = max(1.0 - nest.lapse_rate * capped / nest.surface_temperature, 1e-4);
    float troposphere = nest.surface_pressure * pow(ratio, exponent);
    if (altitude <= nest.tropopause_altitude)
        return troposphere;
    // Isothermal above the tropopause; the scale height is that constant temperature's.
    float scale_height = nest.gas_constant_dry * nest_base_temperature(altitude) / nest.gravity;
    return troposphere * exp(-(altitude - nest.tropopause_altitude) / max(scale_height, 1.0));
}

// The Exner function, Pi = (p/p0)^(R/cp). Temperature is theta * Pi, which is how the
// prognostic potential temperature becomes something saturation can be evaluated against.
float nest_exner(float altitude)
{
    return pow(max(nest_base_pressure(altitude) / nest.reference_pressure, 1e-6),
               nest.gas_constant_dry / nest.specific_heat_pressure);
}

float nest_base_theta(float altitude)
{
    return nest_base_temperature(altitude) / max(nest_exner(altitude), 1e-6);
}

float nest_base_density(float altitude)
{
    return nest_base_pressure(altitude) /
           (nest.gas_constant_dry * max(nest_base_temperature(altitude), 1.0));
}

// ---- Moist thermodynamics -------------------------------------------------------------

// The moisture volume's storage scale. Mixing ratios are a few grams per kilogram, and an
// rgba16f texel has a ten-bit mantissa, so storing kg/kg directly would spend the format's
// precision on leading zeros; the field is held in grams per kilogram and divided back here.
//
// It lives in this shared header rather than beside each user because the volume it describes
// is one volume: a copy that drifted, or a reader that never learned about the scale at all,
// is off by a factor of a thousand with no symptom the shader itself can see. That is not
// hypothetical — it is exactly how this diagnostic first reported 69500% relative humidity.
//
// Note also what the channels are: **total** vapour, cloud and rain mixing ratios, not
// perturbations about the base state, unlike `theta` and the wind components next to them.
const float MOISTURE_UNIT = 1000.0;

// Saturation vapour pressure over liquid water, Pa (Magnus/Teten). The relation the shipped
// system replaced with `if (humidity > 0.85)`: saturation depends on temperature
// *exponentially*, so a relative-humidity threshold cannot place a cloud base where a rising
// parcel actually reaches it.
float nest_saturation_pressure(float temperature)
{
    return 611.2 * exp(17.67 * (temperature - 273.15) / max(temperature - 29.65, 1.0));
}

// Saturation mixing ratio, kg/kg.
float nest_saturation_mixing_ratio(float temperature, float pressure)
{
    float e_s = nest_saturation_pressure(temperature);
    float denominator = pressure - 0.378 * e_s;
    return denominator > 1.0 ? 0.622 * e_s / denominator : 1.0;
}

float nest_base_vapour(float altitude)
{
    float temperature = nest_base_temperature(altitude);
    float pressure = nest_base_pressure(altitude);
    float saturation = nest_saturation_mixing_ratio(temperature, pressure);
    float humidity = nest.surface_humidity * exp(-altitude / max(nest.humidity_scale_height, 1.0));
    return min(humidity * saturation, saturation);
}

// ---- Zone weights ----------------------------------------------------------------------

// Davies (1976) lateral relaxation weight: 1 at the outermost cell, falling to 0 at the inner
// edge of the zone. A ramp rather than a hard injection is what stops the boundary reflecting
// everything that reaches it — which is the difference between a front crossing the domain and
// a front bouncing off its edge.
float nest_boundary_weight(ivec3 cell)
{
    if (nest.boundary_zone <= 0)
        return 0.0;
    int distance = min(min(cell.x, nest.cells_x - 1 - cell.x),
                       min(cell.z, nest.cells_z - 1 - cell.z));
    float inside = float(distance) / float(nest.boundary_zone);
    return 1.0 - clamp(inside, 0.0, 1.0);
}

// Rayleigh sponge weight below the rigid lid. Without it the gravity waves convection radiates
// upward reflect off the top and interfere with the storms that launched them.
float nest_sponge_weight(float altitude)
{
    float start = nest.domain_top - max(nest.sponge_depth, 1.0);
    float t = clamp((altitude - start) / max(nest.sponge_depth, 1.0), 0.0, 1.0);
    // sin^2 ramp: zero value *and* zero slope where the sponge begins, so the damping does not
    // itself become a discontinuity waves can reflect from.
    float s = sin(1.5707963 * t);
    return s * s;
}

// A cheap, deterministic hash-based surface perturbation. Convection needs something to break
// the horizontal symmetry of a uniformly heated surface or the whole boundary layer rises as
// one slab and no cell ever forms; real air has turbulence doing this, and every cloud model
// without a resolved surface layer seeds it explicitly.
float nest_thermal_seed(ivec3 cell)
{
    uvec3 h = uvec3(cell) * uvec3(0x8da6b343u, 0xd8163841u, 0xcb1ab31fu) +
              uint(nest.step_index) * 0x9e3779b9u;
    uint n = h.x ^ h.y ^ h.z;
    n ^= n >> 15; n *= 0x2c1b3c6du; n ^= n >> 12; n *= 0x297a2d39u; n ^= n >> 15;
    return (float(n & 0xffffffu) / float(0xffffff) - 0.5) * 2.0;
}
