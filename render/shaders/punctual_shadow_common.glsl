// The shared punctual shadow atlas: the record struct, its bindings, and the sampling
// function every consumer of it shares. Split out of clustered_lighting.glsl so a shader
// that only needs shadow visibility (the ground march in sky.frag, which has no cluster
// grid of its own reason to include the decal/bindless-heap machinery that file also
// carries) can pull in just this. Include AFTER temporal_common.glsl — sample_punctual_shadow
// uses vogel_disc() and temporal_dither() from there — and declare its two bindings (18, 19
// on the shared scene set; a pass-local set may renumber them the way particle.frag does)
// before including this file if the including shader uses a non-default binding scheme.
//
// A directional light's secondary shadow caster (LightSystem::assign_directional_shadows)
// rides the exact same records and atlas as a spot/point caster: the struct doesn't care
// whether the projection behind it was perspective or orthographic.

#ifndef PUNCTUAL_SHADOW_COMMON_GLSL
#define PUNCTUAL_SHADOW_COMMON_GLSL

struct LightShadow
{
    mat4 view_proj;
    vec4 tile; // xy = atlas uv offset, z = uv scale, w spare
};

layout(std430, set = 0, binding = 19) readonly buffer LightShadowData
{
    LightShadow records[];
} light_shadow_data;

layout(set = 0, binding = 18) uniform sampler2DShadow light_shadow_atlas;

// Which cube face a light-to-fragment direction falls on, in the +X,-X,+Y,-Y,+Z,-Z
// order LightSystem::assign_shadows lays the six point-light face records down in. The
// dominant axis picks the face; its sign picks which of the pair. A point light's shadow
// record index (cone.z) is the base of its six faces, and this offset selects the one.
int cube_shadow_face(vec3 d)
{
    vec3 a = abs(d);
    if (a.x >= a.y && a.x >= a.z)
        return d.x > 0.0 ? 0 : 1;
    if (a.y >= a.z)
        return d.y > 0.0 ? 2 : 3;
    return d.z > 0.0 ? 4 : 5;
}

// One caster's visibility at a fragment: project into its atlas tile and filter. A
// single 2×2 hardware PCF tap leaves a hard, stair-stepped penumbra edge; a small
// Vogel-disc spread of comparison taps (each itself a free 2×2 average) softens it into
// a real penumbra. The disc rotates per pixel with the frame so the temporal resolve
// averages the residual. Taps are clamped into the caster's own tile so none reads a
// neighbouring tile's depth; out-of-tile projection reads the white border, i.e. lit.
float sample_punctual_shadow(int record, vec3 world_pos)
{
    LightShadow s = light_shadow_data.records[record];
    vec4 clip = s.view_proj * vec4(world_pos, 1.0);
    if (clip.w <= 0.0)
        return 1.0;
    vec3 ndc = clip.xyz / clip.w;
    if (ndc.x < -1.0 || ndc.x > 1.0 || ndc.y < -1.0 || ndc.y > 1.0 || ndc.z < 0.0 || ndc.z > 1.0)
        return 1.0;
    vec2 base = s.tile.xy + (ndc.xy * 0.5 + 0.5) * s.tile.z;
    float reference = ndc.z - 0.0015; // constant depth bias against acne

    const int TAPS = 8;
    float angle = temporal_dither(gl_FragCoord.xy) * 6.28318530718;
    float radius = s.tile.z * 0.015; // penumbra width as a fraction of the tile
    vec2 lo = s.tile.xy;
    vec2 hi = s.tile.xy + vec2(s.tile.z);
    float sum = 0.0;
    for (int i = 0; i < TAPS; ++i)
    {
        vec2 uv = clamp(base + vogel_disc(i, TAPS, angle) * radius, lo, hi);
        sum += texture(light_shadow_atlas, vec3(uv, reference));
    }
    return sum / float(TAPS);
}

#endif // PUNCTUAL_SHADOW_COMMON_GLSL
