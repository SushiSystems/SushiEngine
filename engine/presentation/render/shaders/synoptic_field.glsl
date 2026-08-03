// The planetary weather placement, evaluated in the shader.
//
// **This is the second of two evaluators of one function**, and that is the point of it rather
// than an accident. `Atmosphere::SynopticField` (include/SushiEngine/atmosphere/synoptic_field.hpp)
// is the definition; the simulation samples it per column to publish the weather field the
// cloudscape bake reads, and this samples the *same* closed form per march step out past every
// baked window, where no lattice reaches. Both read the same centre list out of the scene block,
// so the two cannot disagree about where the weather is — and disagreeing is exactly what would
// show, as a seam at the far window's rim where the baked answer hands over to this one.
//
// Kept in step with the C++ by construction rather than by discipline: every constant below is
// quoted from the header's own, and any change to one is a change that must be made twice on
// purpose. The header carries the reasoning for the numbers; this file carries only the
// arithmetic.
//
// Requires the scene block (`scene_weather_tail.glsl`) to be in scope.

const float SYNOPTIC_DEGREE = 0.01745329251994329577;

// The zonal-mean cloud fraction at a latitude: three Gaussians on a base. ~0.64 at the ITCZ,
// **0.06 in the subtropics**, 0.66 in the storm track, 0.31 at the pole, 0.30 across ordinary
// midlatitudes.
//
// The subtropical minimum is the term that earns its place: it is why an orbital photograph of
// Earth has large, genuinely clear ocean in it, and a field without it reads as overcast
// everywhere no matter how the rest is tuned.
//
// This is the *optically thick* fraction, not the total cloud fraction — see the long note on
// `Atmosphere::synoptic_zonal_coverage`, whose numbers these mirror exactly. The mirroring is
// the correctness property of the whole two-evaluator design: this function and that one are
// sampled on opposite sides of the render seam, and any disagreement between them shows up as
// a step at the far window's rim. Change one, change both.
float synoptic_zonal_coverage(float latitude, float itcz)
{
    float absolute = abs(latitude);
    float band_itcz = (latitude - itcz) / (8.0 * SYNOPTIC_DEGREE);
    float band_subtropics = (absolute - 25.0 * SYNOPTIC_DEGREE) / (12.0 * SYNOPTIC_DEGREE);
    float band_storm = (absolute - 58.0 * SYNOPTIC_DEGREE) / (16.0 * SYNOPTIC_DEGREE);
    return clamp(0.30 + 0.34 * exp(-band_itcz * band_itcz) -
                     0.24 * exp(-band_subtropics * band_subtropics) +
                     0.36 * exp(-band_storm * band_storm),
                 0.0, 1.0);
}

// The zonal-mean convective fraction. The tropics convect and the midlatitudes do not — a
// tropical sky is towering cumulus, a midlatitude one is layered frontal cloud — and that is a
// function of latitude before it is a function of anything else.
float synoptic_zonal_convective(float latitude, float itcz)
{
    float tropics = (latitude - itcz) / (15.0 * SYNOPTIC_DEGREE);
    return 0.15 + 0.75 * exp(-tropics * tropics);
}

/**
 * Total cloud fraction and convective character at a point on the body.
 *
 * @param radial       The sample's unit radial, scene space — the same vector the march already
 *                     forms to get its altitude, so this costs no extra normalize.
 * @param pole         The body's rotation axis, scene space (`scene.planet_frame.xyz`).
 * @param convective   Receives the convective fraction there, [0, 1].
 * @return             Cloud fraction there, [0, 1].
 */
float synoptic_coverage(vec3 radial, vec3 pole, out float convective)
{
    if (scene.synoptic_params.z < 0.5)
    {
        // No atmosphere published a structure. Uniform is the honest answer, and it is what
        // the deck stack alone gives everywhere.
        convective = 0.5;
        return 1.0;
    }

    // Latitude straight from the radial: the pole is a unit vector, so its dot with the radial
    // is the sine of the latitude and one `asin` finishes it. Cheaper than reconstructing a
    // full geographic coordinate, and longitude is never needed — the centres are compared
    // against the radial directly, which is the whole reason they are published as directions.
    float latitude = asin(clamp(dot(radial, normalize(pole)), -1.0, 1.0));
    float itcz = scene.synoptic_params.y;

    float coverage = synoptic_zonal_coverage(latitude, itcz);
    float zonal_convective = synoptic_zonal_convective(latitude, itcz);

    // The placed systems. A low raises coverage and brings its own convective character; a high
    // suppresses coverage toward zero and brings none — an anticyclone does not make a sky
    // stratiform, it makes it empty, and the coverage term already says so.
    float weighted = 0.0;
    float total = 0.0;
    int count = int(scene.synoptic_params.x);
    for (int i = 0; i < count && i < 12; ++i)
    {
        vec4 a = scene.synoptic_centre_a[i];
        vec4 b = scene.synoptic_centre_b[i];
        float weight = exp(-a.w * max(1.0 - dot(radial, a.xyz), 0.0));
        coverage += b.x * weight;
        if (b.x > 0.0)
        {
            float presence = b.x * weight;
            weighted += presence * b.y;
            total += presence;
        }
    }

    convective = total > 1e-6
                     ? clamp(zonal_convective +
                                 clamp(total, 0.0, 1.0) * (weighted / total - zonal_convective),
                             0.0, 1.0)
                     : clamp(zonal_convective, 0.0, 1.0);
    return clamp(coverage, 0.0, 1.0);
}
