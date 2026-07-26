// The simulated weather field's coverage authority, shared by every consumer of the baked
// cloudscape field so the march, the light volume, the shadow map, and the far-field
// panorama can never disagree about where cloud actually is.
//
// docs/slop/atmosphere_system.md §1.1/§7.3: the baked field carries *shape* — the deck
// stack's genus profile, its noise, its erosion — over a wrapping tile, and that tile is the
// same everywhere. The simulation carries *where the weather is*, uniquely, over hundreds of
// kilometres. Multiplying one by the other is what turns a globally uniform sky into a sky
// with a front in it, and it is why tiled noise stops reading as tiling: the thing that
// repeats is no longer the thing deciding whether there is a cloud here.
//
// The coverage term is applied as a re-threshold, not a fade. Scaling density directly would
// dim a whole cloud toward transparency as coverage fell; re-thresholding erodes its
// low-density fringe first, so cloud edges retreat and holes open — clouds shrink and grow
// the way coverage actually makes them.

// Continuous band coordinate for a sample altitude, as the field's 3-slice W texture
// coordinate. Placing the bands at their centres and interpolating between them is what
// keeps a march climbing past a band boundary from stepping discontinuously.
float weather_field_w(vec3 band_altitudes, float altitude)
{
    float index;
    if (altitude <= band_altitudes.x)
        index = 0.0;
    else if (altitude <= band_altitudes.y)
        index = (altitude - band_altitudes.x) / max(band_altitudes.y - band_altitudes.x, 1.0);
    else if (altitude <= band_altitudes.z)
        index = 1.0 + (altitude - band_altitudes.y) / max(band_altitudes.z - band_altitudes.y, 1.0);
    else
        index = 2.0;
    return (index + 0.5) / 3.0;
}

// The raw field texel at a camera-relative position and altitude:
// r = coverage, g = density scale (half-encoded), b = convective fraction, a = precipitation.
vec4 weather_field_fetch(sampler3D weather_field, vec4 map, vec4 levels, vec3 p, float altitude)
{
    vec2 uv = p.xz * map.xy + map.zw;
    return texture(weather_field, vec3(uv, weather_field_w(levels.xyz, altitude)));
}

// The reference column's coverage at a sample altitude, interpolated across the same band
// centres weather_field_w uses — so numerator and denominator are always read from the same
// point in the vertical, and a sample between two bands cannot compare one band's local
// coverage against another band's reference.
float weather_reference_coverage(vec3 band_altitudes, vec3 reference, float altitude)
{
    if (altitude <= band_altitudes.x)
        return reference.x;
    if (altitude <= band_altitudes.y)
        return mix(reference.x, reference.y,
                   (altitude - band_altitudes.x) / max(band_altitudes.y - band_altitudes.x, 1.0));
    if (altitude <= band_altitudes.z)
        return mix(reference.y, reference.z,
                   (altitude - band_altitudes.y) / max(band_altitudes.z - band_altitudes.y, 1.0));
    return reference.z;
}

// How much more (or less) cloud the simulation puts here than where the deck stack was
// compiled. 1.0 at the reference column by construction, so a scene whose weather happens to
// be uniform renders exactly as it did before the field existed.
float weather_coverage_scale(sampler3D weather_field, vec4 map, vec4 levels, vec4 reference,
                             vec3 p, float altitude)
{
    if (levels.w < 0.5)
        return 1.0;

    float coverage = weather_field_fetch(weather_field, map, levels, p, altitude).r;
    float base = weather_reference_coverage(levels.xyz, reference.xyz, altitude);

    // A reference column with essentially no cloud carries no information about how much
    // more cloud belongs here — the ratio would be arbitrarily large. Fall back to leaving
    // the bake alone rather than inventing a multiplier out of a division by nothing.
    const float MIN_REFERENCE_COVERAGE = 0.02;
    if (base < MIN_REFERENCE_COVERAGE)
        return 1.0;

    // Capped rather than unbounded: past roughly this much the re-threshold below is filling
    // in sky the bake never shaped, which reads as a flat sheet instead of as more cloud.
    const float MAX_COVERAGE_SCALE = 1.6;
    return clamp(coverage / base, 0.0, MAX_COVERAGE_SCALE);
}

// Applies a coverage scale to a baked density (see the file header for why this is a
// re-threshold). `scale == 1` is exactly the identity, so the disabled path costs nothing
// beyond the branch.
float weather_apply_coverage(float density, float scale)
{
    if (scale > 0.999 && scale < 1.001)
        return density;
    if (scale <= 0.0)
        return 0.0;
    return clamp((density - (1.0 - scale)) / scale, 0.0, 1.0) * scale;
}
