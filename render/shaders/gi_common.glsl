// The probe-volume GI parameterization, shared by the shading pass that gathers probes
// and any pass that fills them, so the lattice math and the SH evaluation are written
// once. No bindings and no version here: this is included into shaders that declare
// their own probe buffers, exactly as atmosphere_common.glsl is included into the sky.

// The std140 mirror of Render::Gi::ProbeVolumeConfig. Field-for-field identical packing.
struct GiProbeVolume
{
    vec4 origin_enabled; // xyz camera-relative origin of probe (0,0,0), w = enabled
    vec4 spacing_bias;   // xyz spacing in metres, w = normal bias in metres
    vec4 intensity;      // x = indirect intensity, yzw spare
    ivec4 counts;        // xyz probe counts per axis, w = total probe count
};

// Second-order SH irradiance from nine coefficients and a normal. The cosine-lobe band
// factors and 1/pi are already baked into the coefficients at projection, so this is the
// bare polynomial — the same one evaluate_sh() runs on the global set.
vec3 gi_sh_irradiance(vec4 coeff[9], vec3 n)
{
    float y[9];
    y[0] = 0.282095;
    y[1] = 0.488603 * n.y;
    y[2] = 0.488603 * n.z;
    y[3] = 0.488603 * n.x;
    y[4] = 1.092548 * n.x * n.y;
    y[5] = 1.092548 * n.y * n.z;
    y[6] = 0.315392 * (3.0 * n.z * n.z - 1.0);
    y[7] = 1.092548 * n.x * n.z;
    y[8] = 0.546274 * (n.x * n.x - n.y * n.y);
    vec3 result = vec3(0.0);
    for (int i = 0; i < 9; ++i)
        result += coeff[i].rgb * y[i];
    return max(result, vec3(0.0)); // clamp the small negative lobe SH can ring into
}

// Linear index of a probe cell, in the (z, y, x) row-major order the relight writes.
int gi_probe_index(GiProbeVolume vol, ivec3 cell)
{
    ivec3 c = clamp(cell, ivec3(0), vol.counts.xyz - 1);
    return (c.z * vol.counts.y + c.y) * vol.counts.x + c.x;
}

// Locates the lower corner of the lattice cell a camera-relative point falls in and the
// fractional position inside it. Returns false when the point is outside the cascade, so
// the caller can fall back to the distant environment rather than extrapolate.
bool gi_locate(GiProbeVolume vol, vec3 camera_relative_pos, out ivec3 base, out vec3 frac)
{
    vec3 grid = (camera_relative_pos - vol.origin_enabled.xyz) / vol.spacing_bias.xyz;
    base = ivec3(floor(grid));
    frac = grid - vec3(base);
    if (any(lessThan(base, ivec3(0))) || any(greaterThanEqual(base, vol.counts.xyz - 1)))
        return false;
    return true;
}
