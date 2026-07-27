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
    float boundary_layer_depth;
    float boundary_layer_velocity_scale;
    float sponge_depth;
    float sponge_rate;
    float boundary_relaxation;
    float thermal_seed_amplitude;
    float coriolis;
    float convective_velocity_scale;

    // Microphysics.
    float cloud_critical_humidity;
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

// The moisture volume's storage scale: the field is held in grams per kilogram and divided
// back to kg/kg here, so every reader sees SI and only the texels are scaled.
//
// It lives in this shared header rather than beside each user because the volume it describes
// is one volume: a copy that drifted, or a reader that never learned about the scale at all,
// is off by a factor of a thousand with no symptom the shader itself can see. That is not
// hypothetical — it is exactly how this diagnostic first reported 69500% relative humidity.
//
// Note also what the channels are: **total** vapour, cloud and rain mixing ratios, not
// perturbations about the base state, unlike `theta` and the wind components next to them.
// The two conventions sitting side by side in one nest is the single easiest thing to get
// wrong here, and it has been got wrong twice.
//
// The fourth channel is the odd one out and is deliberately *not* scaled by this: it is the
// diagnosed cloud fraction, already dimensionless and already in [0, 1]. It is stored beside
// the water rather than in its own volume because it is a property of the same cell and costs a
// channel that was zero, and it is a **diagnosis, not a prognostic** -- `atmosphere_microphysics`
// overwrites it in full every step from that step's own state. Advection and the boundary-layer
// mixing carry it along with the rest of the vec4, which is harmless precisely because nothing
// reads it before microphysics has replaced it.
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

// ---- Subgrid cloud fraction -------------------------------------------------------------

// The fraction of a nominal excess that survives its own latent heating.
//
// Condensing water warms the parcel, which raises q_s, which leaves less excess to condense --
// so a saturation adjustment is a fixed point rather than a subtraction. One Newton step on
// f(d) = (q_v - d) - q_s(T + L d / c_p) is exact enough at these time steps, and this is that
// step's denominator. Roughly 1/2 at 290 K, which is why a warm cloud takes about twice the
// water a cold one does to reach the same condensate.
float nest_condensation_efficiency(float saturation, float temperature)
{
    float safe = max(temperature, 1.0);
    float slope = saturation * nest.latent_heat_vaporization /
                  (nest.gas_constant_vapour * safe * safe);
    return 1.0 / (1.0 + nest.latent_heat_vaporization / nest.specific_heat_pressure * slope);
}

// Split a cell's total water into a cloud fraction and the cell-mean condensate it implies.
//
// **The closure that lets a grid-mean model draw a cumulus.** A cell here is 2 km across, so
// its humidity is a cell mean, and a fair-weather cumulus is a 200 m - 1 km thermal that is
// saturated inside while the cell around it is not. Condensing only when the mean saturates
// therefore cannot produce one -- it produces nothing until the whole 4 km^2 column saturates,
// and then it produces fog. That is measured: before this existed the mixed-layer top ran
// 78-95 % relative humidity and never crossed, and every run that made condensate made it at
// 19 m.
//
// So the humidity inside a cell is a *distribution* about its mean, taken as a top-hat of
// half-width (1 - critical) * q_s (Sommeria & Deardorff 1977; Mellor 1977 -- the simplest
// member of the family Smith 1990 generalises with a triangular one). Writing Q for how far the
// mean sits across that half-width, the saturated tail gives fraction (1 + Q)/2 and a condensate
// quadratic in (1 + Q): the first cloud in a cell is thin and thickens faster than linearly.
//
// At critical = 1 the width is zero and this collapses *exactly* onto the all-or-nothing
// adjustment it generalises, which is what makes it one condensation path rather than two.
//
// Mirrors Render::atmosphere_cloud_partition; neither is edited alone.
//
// @param total_water Vapour plus cloud, kg/kg. Rain precipitates and is not in the distribution.
// @param saturation  Saturation mixing ratio at the *liquid-water* temperature, kg/kg.
// @param efficiency  nest_condensation_efficiency at that temperature.
// @param fraction    Out: the fraction of the cell that is cloud.
// @return            The cell-mean condensate, kg/kg.
float nest_cloud_partition(float total_water, float saturation, float efficiency,
                           out float fraction)
{
    float mean = efficiency * (total_water - saturation);
    float width = efficiency * (1.0 - clamp(nest.cloud_critical_humidity, 0.0, 1.0)) * saturation;
    if (width <= 0.0)
    {
        fraction = mean > 0.0 ? 1.0 : 0.0;
        return max(mean, 0.0);
    }
    float across = mean / width;
    if (across <= -1.0)
    {
        fraction = 0.0;
        return 0.0;
    }
    if (across >= 1.0)
    {
        fraction = 1.0;
        return mean;
    }
    fraction = 0.5 * (1.0 + across);
    return width * (1.0 + across) * (1.0 + across) * 0.25;
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

// Vertical eddy diffusivity at a level *face*, m^2/s — the mixed layer, parameterized.
//
// The turbulence that carries surface heat and moisture upward has eddies of tens to hundreds
// of metres; this grid is 2 km across, so that transport is entirely subgrid and has to be
// represented rather than resolved. Every operational model does the same thing (§2.1's YSU,
// MYNN) and for the same reason.
//
// The profile is Troen & Mahrt's (1986), the one YSU and its lineage use:
//
//     K(z) = kappa * w_s * z * (1 - z/h)^2
//
// Linear in height at the ground, where surface-layer similarity requires it; peaking at h/3;
// and returning to zero at the layer top so the stratified free troposphere above is not eroded
// from below. What it produces is a *well-mixed* layer — uniform potential temperature and
// moisture through its depth — which is what turns a surface flux into a boundary layer instead
// of a hotplate.
//
// **Why not the simpler parabola 4*K_peak*f*(1-f).** That was the first form here and it fails in
// exactly the place that decides whether this model makes cloud. Near the ground it goes as
// K_peak * (z/h), so its near-surface diffusivity *weakens as the layer deepens* — and the lowest
// face is the one that must carry the entire surface flux out of a 54 m level. The feedback is
// perverse and it was measured: with a 2 500 m layer the lowest face saw 12 m^2/s, the surface
// level sat 9 K above the one 80 m over it, the layer never homogenised, and its top reached only
// 57 % relative humidity in eight hours of heating. Troen-Mahrt's slope does not know h at all, so
// the same authored number gives 45 m^2/s there instead. A model normalised to a peak cannot
// express that, which is why the parameter is a *velocity scale* and not a diffusivity.
//
// @param face_altitude Altitude of the level face, metres above the surface.
// @param depth         This column's diagnosed mixed-layer depth, metres. Per column and not
//                      the authored constant, because the depth is a *state*: it grows through
//                      the morning as the surface parcel outgrows more of the stratification
//                      above it, and it is that growth that concentrates the surface moisture
//                      in a shallow layer early and lifts it to its condensation level later.
float nest_face_diffusivity(float face_altitude, float depth)
{
    const float VON_KARMAN = 0.4;
    if (depth <= 0.0)
        return 0.0;
    float fraction = face_altitude / depth;
    if (fraction <= 0.0 || fraction >= 1.0)
        return 0.0;
    float taper = 1.0 - fraction;
    return VON_KARMAN * nest.boundary_layer_velocity_scale * face_altitude * taper * taper;
}

// A cheap, deterministic hash-based surface perturbation, in [-1, 1]. Convection needs something
// to break the horizontal symmetry of a uniformly heated surface or the whole boundary layer
// rises as one slab and no cell ever forms; real air has turbulence doing this, and every cloud
// model without a resolved surface layer seeds it explicitly.
//
// Its *caller* decides what it perturbs, and `atmosphere_forces.comp` scales the surface flux by
// it rather than adding it to theta — see there for why an additive kick redrawn every step is a
// random walk with no bound, and what that random walk did.
//
// Named limit: white in space as well as in time, so it carries no structure at the several-cell,
// several-minute scale a 2 km-resolved plume would organise around. A time-correlated,
// spatially-smooth field is the refinement.
float nest_thermal_seed(ivec3 cell)
{
    uvec3 h = uvec3(cell) * uvec3(0x8da6b343u, 0xd8163841u, 0xcb1ab31fu) +
              uint(nest.step_index) * 0x9e3779b9u;
    uint n = h.x ^ h.y ^ h.z;
    n ^= n >> 15; n *= 0x2c1b3c6du; n ^= n >> 12; n *= 0x297a2d39u; n ^= n >> 15;
    return (float(n & 0xffffffu) / float(0xffffff) - 0.5) * 2.0;
}
