// The regional nest's shared vocabulary: its parameter block, its grid, its base state, and
// the thermodynamic relations every stage of the step is written in terms of.
//
// docs/slop/atmosphere_system.md §6. Included by every atmosphere_*.comp. The C++ side of the
// same relations lives in engine/domain/environment's atmosphere_nest.hpp, which explains why
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
// AtmosphereNestPass::NestParameters field for field.
layout(set = 0, binding = 0) uniform NestParameters
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
    float free_troposphere_drying;
    float free_troposphere_exponent;

    // Dynamics.
    float eddy_viscosity;
    float boundary_layer_depth;
    float boundary_layer_velocity_scale;
    float sponge_depth;
    float sponge_rate;
    float boundary_relaxation;
    float thermal_seed_amplitude;
    float thermal_seed_length;   // metres the surface heterogeneity is correlated over
    float thermal_seed_period;   // seconds it is correlated over
    float coriolis;
    float convective_velocity_scale;

    // Microphysics.
    float cloud_top_longwave_flux;
    float cloud_top_equilibrium_depression; // K below ambient the loss above shuts off at
    float cloud_top_entrainment_efficiency; // dimensionless; 0 disables the closure
    float cloud_water_absorption;
    float cloud_critical_humidity;
    float autoconversion_rate;
    float autoconversion_threshold;
    float accretion_rate;
    float accretion_exponent;
    float rain_evaporation_rate;
    float fall_speed_coefficient;
    float fall_speed_exponent;
    float droplet_effective_radius;
    // Ice. A diagnostic phase partition rather than a second condensate species; see
    // AtmosphereParameters::latent_heat_fusion for the trade and what it gives up.
    float latent_heat_fusion;
    float freezing_temperature;    // warm edge of the mixed-phase band, K
    float glaciation_temperature;  // cold edge, K
    float snow_fall_speed_coefficient;
    float snow_fall_speed_exponent;
    float glaciated_autoconversion_factor;
    float ice_effective_radius;

    // Surface energy balance. The fluxes are solved from these, not authored; see
    // atmosphere_surface.comp.
    float solar_constant;              // top-of-atmosphere irradiance, W/m^2
    float clear_sky_transmittance;     // at zenith; applied along the slant path
    float surface_albedo;
    float surface_emissivity;
    float surface_heat_capacity;       // J/m^2/K of the slab
    float surface_moisture_availability; // beta: the fraction of the deficit the ground supplies
    float surface_exchange_coefficient;  // C_H, bulk transfer
    float surface_minimum_wind;          // m/s the bulk formulae never fall below
    float solar_elevation_sine;    // sine of the sun's elevation; negative below the horizon

    // Grid and step.
    float spacing;      // horizontal cell size, metres
    float domain_top;   // domain top above the surface, metres
    float dt;           // this step's duration, seconds of game time
    int cells_x;
    int cells_z;
    int levels;
    int boundary_zone;  // Davies relaxation zone width, cells
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
// hypothetical: a missed scale here surfaces as a diagnostic reporting 69500 % relative
// humidity.
//
// Note also what the channels are: **total** vapour, cloud and rain mixing ratios, not
// perturbations about the base state, unlike `theta` and the wind components next to them.
// The two conventions sitting side by side in one nest is the single easiest thing to get
// wrong here.
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

// ---- Ice ---------------------------------------------------------------------------------

// Saturation vapour pressure over *ice*, Pa (Magnus, WMO coefficients). A different curve from
// the liquid one above, not a correction to it, and the gap between them is the whole of the
// Bergeron process: at -12 C the ice curve sits about 10 % below the supercooled-water one, so
// air in equilibrium with a droplet is supersaturated with respect to a crystal beside it. The
// crystal grows, the droplet evaporates to feed it, and a mixed-phase cloud turns itself into
// snow. The two curves meet exactly at 0 C, which is what lets the band be blended continuously.
float nest_saturation_pressure_ice(float temperature)
{
    return 611.2 * exp(22.46 * (temperature - 273.15) / max(temperature - 0.53, 1.0));
}

// How much of a cell's condensate is ice, [0, 1] -- **the whole of the phase partition**.
//
// A ramp across the mixed-phase band rather than a switch at 0 C: a cloud at -5 C is mostly
// supercooled water and one at -20 C is mostly crystals, and the band between is where they
// coexist. Everything else about ice here is a function of this one number -- the saturation the
// cell condenses at, the latent heat it releases, how readily it precipitates, how fast that
// precipitation falls.
//
// Mirrors Render::atmosphere_ice_fraction; neither is edited alone.
float nest_ice_fraction(float temperature)
{
    float span = nest.freezing_temperature - nest.glaciation_temperature;
    if (span <= 0.0)
        return temperature < nest.freezing_temperature ? 1.0 : 0.0;
    return clamp((nest.freezing_temperature - temperature) / span, 0.0, 1.0);
}

// The saturation mixing ratio a cell actually condenses at: the two curves blended by the ice
// fraction. Below the glaciation point this *is* the ice curve, well under the liquid one, so a
// column that would be clear at +2 C can be cloudy at -25 C on the same water.
float nest_saturation_mixing_ratio_phase(float temperature, float pressure)
{
    float ice = nest_ice_fraction(temperature);
    if (ice <= 0.0)
        return nest_saturation_mixing_ratio(temperature, pressure);
    float e_s = mix(nest_saturation_pressure(temperature),
                    nest_saturation_pressure_ice(temperature), ice);
    float denominator = pressure - 0.378 * e_s;
    return denominator > 1.0 ? 0.622 * e_s / denominator : 1.0;
}

// Latent heat released per kilogram condensed, J/kg: vaporization plus the ice fraction's share
// of fusion. Depositing straight to a crystal releases both -- 13 % more heat than condensing to
// a droplet, which is a second push for an updraft just as it is running out of the first.
float nest_latent_heat(float temperature)
{
    return nest.latent_heat_vaporization + nest_ice_fraction(temperature) * nest.latent_heat_fusion;
}

// The relative humidity the base state is allowed to reach: the Weisman-Klemp (1982) idealized
// sounding's shape, falling from the surface value to (1 - drying) of it at the tropopause and
// holding above. Mirrors Render::atmosphere_base_humidity_ceiling.
float nest_base_humidity_ceiling(float altitude)
{
    float height = max(altitude, 0.0) / max(nest.tropopause_altitude, 1.0);
    float shape = height < 1.0 ? pow(height, nest.free_troposphere_exponent) : 1.0;
    return max(nest.surface_humidity * (1.0 - nest.free_troposphere_drying * shape), 0.0);
}

// The base-state vapour profile: the *mixing ratio* decays exponentially from its surface value,
// which is what humidity_scale_height names and what a real sounding does. Relative humidity is
// then whatever q_v / q_s(z) comes to — 70 % at the ground and 62 % at 1.3 km, against the 41 %
// applying the exponential to RH itself gave. Above ~7 km q_s folds faster than the vapour and
// the bare exponential would saturate, so the ceiling caps it; without that every run began with
// a global cirrus deck at 9.5 km.
//
// Mirrors Render::atmosphere_base_vapour; neither is edited alone.
float nest_base_vapour(float altitude)
{
    float surface_saturation =
        nest_saturation_mixing_ratio(nest_base_temperature(0.0), nest_base_pressure(0.0));
    float vapour = nest.surface_humidity * surface_saturation *
                   exp(-altitude / max(nest.humidity_scale_height, 1.0));
    float saturation =
        nest_saturation_mixing_ratio(nest_base_temperature(altitude), nest_base_pressure(altitude));
    return min(min(vapour, nest_base_humidity_ceiling(altitude) * saturation), saturation);
}

// ---- Subgrid cloud fraction -------------------------------------------------------------

// The fraction of a nominal excess that survives its own latent heating.
//
// Condensing water warms the parcel, which raises q_s, which leaves less excess to condense --
// so a saturation adjustment is a fixed point rather than a subtraction. One Newton step on
// f(d) = (q_v - d) - q_s(T + L d / c_p) is exact enough at these time steps, and this is that
// step's denominator. Roughly 1/2 at 290 K, which is why a warm cloud takes about twice the
// water a cold one does to reach the same condensate.
// The latent heat is the *phase-blended* one, so a glaciating cell is correctly harder to
// condense in: it releases 13 % more heat per kilogram, which raises q_s further, which leaves
// less of the excess surviving. Using the liquid value below freezing would over-condense
// exactly where the extra heating matters most to the updraft.
float nest_condensation_efficiency(float saturation, float temperature)
{
    float safe = max(temperature, 1.0);
    float latent = nest_latent_heat(temperature);
    float slope = saturation * latent / (nest.gas_constant_vapour * safe * safe);
    return 1.0 / (1.0 + latent / nest.specific_heat_pressure * slope);
}

// Split a cell's total water into a cloud fraction and the cell-mean condensate it implies.
//
// **The closure that lets a grid-mean model draw a cumulus.** A cell here is 2 km across, so
// its humidity is a cell mean, and a fair-weather cumulus is a 200 m - 1 km thermal that is
// saturated inside while the cell around it is not. Condensing only when the mean saturates
// therefore cannot produce one -- it produces nothing until the whole 4 km^2 column saturates,
// and then it produces fog. That is measured: without this closure the mixed-layer top runs
// 78-95 % relative humidity and never crosses, and the only condensate any run produces is at
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
// **Why not the simpler parabola 4*K_peak*f*(1-f).** It fails in exactly the place that decides
// whether this model makes cloud. Near the ground it goes as K_peak * (z/h), so its near-surface
// diffusivity *weakens as the layer deepens* — and the lowest face is the one that must carry the
// entire surface flux out of a 54 m level. The feedback is perverse, and measured: with a
// 2 500 m layer the lowest face sees 12 m^2/s, the surface level sits 9 K above the one 80 m
// over it, the layer never homogenises, and its top reaches only 57 % relative humidity in eight
// hours of heating. Troen-Mahrt's slope does not know h at all, so the same authored number gives
// 45 m^2/s there instead. A model normalised to a peak cannot express that, which is why the
// parameter is a *velocity scale* and not a diffusivity.
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

// ---- Surface heterogeneity ---------------------------------------------------------------

// Lattice cells before the seed field repeats. A power of two, so the wrap the caller applies to
// the world coordinate — planet-scale coordinates do not survive a float32 otherwise — is exactly
// a mask here and the seam is invisible. 256 cells of 6 km is 1536 km, four times the widest
// domain the nest is ever given.
const int NEST_SEED_WRAP = 256;

// One lattice corner's value, in [0, 1).
float nest_seed_corner(ivec3 lattice)
{
    // x and z wrap; the middle axis is time and simply runs, since unsigned arithmetic wraps it
    // 10^14 seconds from now.
    uvec3 c = uvec3(lattice & ivec3(NEST_SEED_WRAP - 1, 0x7fffffff, NEST_SEED_WRAP - 1));
    uint n = c.x * 0x8da6b343u + c.y * 0xd8163841u + c.z * 0xcb1ab31fu;
    n ^= n >> 15; n *= 0x2c1b3c6du; n ^= n >> 12; n *= 0x297a2d39u; n ^= n >> 15;
    return float(n & 0xffffffu) / float(0xffffff);
}

// A deterministic model of *patchy ground*, in [-1, 1]. Convection needs something to break the
// horizontal symmetry of a uniformly heated surface or the whole boundary layer rises as one slab
// and no cell ever forms; real air has turbulence and real ground has patchy albedo, soil moisture
// and cover doing this, and every cloud model without a resolved surface layer seeds it explicitly.
//
// Its *caller* decides what it perturbs, and `atmosphere_forces.comp` scales the surface flux by
// it rather than adding it to theta — see there for why an additive kick redrawn every step is a
// random walk with no bound, and what such a walk does to a level that decouples.
//
// **Correlated, and in physical units.** White noise on (cell index, step index) has three
// properties, none of them true of the ground: no length scale but one cell, no time
// scale but one step, and — because both of those are *grid* quantities — a dependence on the
// render quality tier and on how the frame rate happens to break the step.
//
// Measured, and the measurement is what fixes both scales. `atmosphere_probe` carries two domain
// diagnostics for it (`sky_coverage_sd`, `sky_coverage_roughness`) because the mean coverage
// cannot tell a broken cumulus field from a sheet of the same total cloud, and that distinction is
// the *entire* question a symmetry-breaking seed exists to decide. Roughness over simulated hours
// 3-6, relative to running with the seed switched off altogether:
//
//     no seed at all                        1.00
//     white in space and time               1.22
//     6 km in space, white in time          1.25
//     6 km / 60 s                           1.88
//     3.7 m / 900 s                         3.15
//     24 km / 900 s                         2.27
//     6 km / 900 s (this)                   3.72
//
// A white seed is within 22 % of having no seed at all. Both axes are needed and the
// *time* axis carries most of it: a patch must last long enough for a thermal to organise around
// it, and 900 s is that time. A period well under the turnover (60 s) recovers half the effect and
// a length well over the plume (24 km) two thirds. Four realisations of the field agree to 0.5 %.
//
// So: value noise on a lattice whose two horizontal axes are a *length* and whose third is a
// *time*, interpolated with a smoothstep — C1, because a linear interpolation creases on every
// lattice plane and convection would organise along the creases. The [-1, 1] bound matches a
// white field's, so `thermal_seed_amplitude` means what its documentation says.
//
// Its standard deviation does not: 0.37 against uniform noise's 0.577, because interpolating
// between corners averages them (analytically, 2·sqrt((1/12)·0.743³)). The measurement above says
// that costs nothing — what lifts a plume is the *mean* forcing over the plume's footprint, and
// white noise cancels over an N-cell footprint as 1/sqrt(N) while a correlated field does not.
//
// Anchored to the ground and not to the grid: the caller passes a *world* position, so the pattern
// stays put when the nest re-centres on a moving observer, and all four tiers sample one field.
// (Contrast `cloud_critical_humidity`, §6's named limit, which does not scale with the spacing and
// therefore does not have this property.)
//
// Named limit: one octave, so the ground this models is patchy at exactly one scale, and real land
// cover is not. Phase B3 is where the pattern should stop being a hash at all — once a surface
// energy balance carries a land/sea mask and an albedo field, *those* are the heterogeneity, and
// this reduces to the unresolved-turbulence residual on top of them.
//
// @param world_xz Cell centre in world metres, wrapped by the caller onto NEST_SEED_WRAP lattice
//                 cells so a planet-scale coordinate does not arrive quantised.
// @param seconds  Game seconds the *step* begins at. Per step and not per frame: a frame records
//                 several steps against one upload of the parameter block, so a field of that
//                 block would hand every step of the frame the identical pattern.
float nest_thermal_seed(vec2 world_xz, float seconds)
{
    float length = max(nest.thermal_seed_length, 1.0);
    vec3 p = vec3(world_xz.x / length,
                  seconds / max(nest.thermal_seed_period, 1.0),
                  world_xz.y / length);
    ivec3 base = ivec3(floor(p));
    vec3 f = p - vec3(base);
    vec3 w = f * f * (3.0 - 2.0 * f);

    float x00 = mix(nest_seed_corner(base + ivec3(0, 0, 0)),
                    nest_seed_corner(base + ivec3(1, 0, 0)), w.x);
    float x10 = mix(nest_seed_corner(base + ivec3(0, 1, 0)),
                    nest_seed_corner(base + ivec3(1, 1, 0)), w.x);
    float x01 = mix(nest_seed_corner(base + ivec3(0, 0, 1)),
                    nest_seed_corner(base + ivec3(1, 0, 1)), w.x);
    float x11 = mix(nest_seed_corner(base + ivec3(0, 1, 1)),
                    nest_seed_corner(base + ivec3(1, 1, 1)), w.x);
    return (mix(mix(x00, x10, w.y), mix(x01, x11, w.y), w.z) - 0.5) * 2.0;
}
