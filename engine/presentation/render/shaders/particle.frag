#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

// Particle billboard fragment shader (design §5.4): a soft round sprite that is occluded by the
// scene. It samples the scene depth (never bound as an attachment, only read) and discards where
// the billboard lies behind opaque geometry — the sky/cloud pattern — so particles do not shine
// through walls. Reverse-Z: a larger depth is nearer, so a fragment whose depth is below the
// stored scene depth is behind that surface.
//
// The lit bucket (true-alpha smoke/dust) is shaded as a camera-facing hemisphere by the
// directional sun (world-space, no camera-relative conversion) plus the clustered punctual
// lights and the environment's SH ambient. The punctual lights and the froxel grid live in
// camera-relative space, so the sprite carries its camera-relative centre and view-space depth
// down from the vertex stage (in_light) and the fragment maps that to a cluster and loops only
// that cluster's lights — the same Forward+ path the meshes use, minus the BRDF (a puff is a
// diffuse blob, not a surface). Additive/host billboards stay emissive (unlit flag).
//
// The lit bucket also receives the sun's cascade shadow, so a puff standing in the shadow of
// geometry goes dark instead of staying uniformly sunlit. The cascade matrices are already
// camera-relative, so the same carried centre serves both the cluster lookup and the shadow
// projection.
//
// The particle *material* — the sprite texture and its flipbook cell, the soft-particle fade
// distance, and whether the particle is lit — arrives per emitter through the vertex stage rather
// than per draw, because a bucket mixes emitters: the additive list holds every non-alpha sprite
// whatever its author asked for. An untextured particle keeps the built-in radial dot; a textured
// one hands its falloff over to the texture's own alpha, which is what a puff or a spark sheet is
// authored with.

#include "blue_noise.glsl"
#include "clustered_lighting_common.glsl"
#include "gi_common.glsl"
#include "particle_common.glsl"
#include "particle_shadow.glsl"

// The bindless texture heap (set 1), the same slot and the same array the mesh materials sample
// from, so a sprite texture is registered once and addressed by the very index a material map is.
layout(set = 1, binding = 0) uniform sampler2D bindless_textures[];

layout(set = 0, binding = 1) uniform sampler2D scene_depth;

// The clustered punctual-light engine, a subset of pbr.frag's scene-set bindings re-declared on
// this pass's own set: the light array, the per-cluster count grid and index list the light-cull
// pass wrote this frame, and a truncated ClusterBlock (only the froxel fields this pass reads,
// layout-compatible with the full block). The IBL SH is the environment's diffuse ambient.
layout(std430, set = 0, binding = 3) readonly buffer LightBuffer { PunctualLight lights[]; } light_buffer;
layout(std430, set = 0, binding = 4) readonly buffer ClusterGrid { uint counts[]; } cluster_grid;
layout(std430, set = 0, binding = 5) readonly buffer LightIndexList { uint indices[]; } light_index_list;
layout(set = 0, binding = 6) uniform ClusterBlock
{
    vec4 grid;   // w = active light count this frame
    vec4 depth;  // near, far, log-slice scale, log-slice bias
    vec4 screen; // render w, h, tile size x, tile size y
} cluster;
layout(std430, set = 0, binding = 7) readonly buffer IrradianceSh { vec4 coeff[9]; } irradiance_sh;

// The sun's shadow cascades. The block itself comes in at binding 10 from shadow_common.glsl
// (included through particle_shadow.glsl) — the scene set's own numbering, reused here because
// this pass's set has those slots free, so the cascade block needs no re-declaration.
layout(set = 0, binding = 11) uniform sampler2DShadow shadow_atlas;

layout(push_constant) uniform Push
{
    mat4 view_projection;
    vec4 camera_right;   // xyz world camera right; w = eye.x (unused in fragment)
    vec4 camera_up;      // xyz world camera up;    w = eye.y (unused in fragment)
    vec4 sun_direction;  // xyz to-sun;             w = eye.z (unused in fragment)
    vec4 sun_radiance;   // rgb sun colour * intensity; w = IBL ambient scale for lit particles
} pc;

layout(location = 0) in vec2 in_uv;   // local quad coordinate: the round falloff and the normal
layout(location = 1) in vec4 in_color;
layout(location = 2) in vec4 in_light; // xyz = camera-relative centre, w = view-space depth
layout(location = 3) in vec2 in_atlas_uv;          // where the sprite texture is sampled
layout(location = 4) flat in uvec2 in_material;    // x = bindless slot, y = Vfx::RenderFlags
layout(location = 5) flat in float in_soft_fade;   // metres the soft fade ramps over

layout(location = 0) out vec4 out_color;

// Sums the unshadowed diffuse contribution of every punctual light whose cluster this sprite is
// in. A particle is a diffuse blob, so there is no BRDF and no per-light shadow — just the shared
// windowed inverse-square (and spot cone) falloff times N·L against the hemisphere normal. Runs
// on the lit bucket only; unlit buckets never call it.
vec3 clustered_particle_light(vec3 normal, vec3 camrel, float view_z, vec2 frag_coord)
{
    if (cluster.grid.w < 0.5) // no lights this frame
        return vec3(0.0);

    uint index = cluster_index(frag_coord, view_z, cluster.depth, cluster.screen);
    uint count = min(cluster_grid.counts[index], MAX_LIGHTS_PER_CLUSTER);
    uint base = index * MAX_LIGHTS_PER_CLUSTER;

    vec3 sum = vec3(0.0);
    for (uint i = 0u; i < count; ++i)
    {
        PunctualLight light = light_buffer.lights[light_index_list.indices[base + i]];
        vec3 light_dir;
        float distance_to_light;
        float attenuation = punctual_attenuation(light, camrel, light_dir, distance_to_light);
        if (attenuation <= 0.0)
            continue;
        float n_dot_l = max(dot(normal, light_dir), 0.0);
        sum += light.color_intensity.xyz * light.color_intensity.w * (attenuation * n_dot_l);
    }
    return sum;
}

void main()
{
    ivec2 pixel = ivec2(gl_FragCoord.xy);
    float stored_depth = texelFetch(scene_depth, pixel, 0).r;
    if (gl_FragCoord.z < stored_depth)
        discard; // behind opaque geometry (reverse-Z: smaller depth is farther)

    uint flags = in_material.y;
    vec2 offset = in_uv * 2.0 - 1.0;
    vec3 rgb = in_color.rgb;
    float alpha = in_color.a;
    if ((flags & RENDER_TEXTURED) != 0u)
    {
        // The texture's own alpha *is* the sprite's shape, so the round falloff steps aside —
        // applying both would vignette every authored sheet a second time.
        vec4 texel = texture(bindless_textures[nonuniformEXT(in_material.x)], in_atlas_uv);
        rgb *= texel.rgb;
        alpha *= texel.a;
    }
    else
    {
        float radial = max(0.0, 1.0 - dot(offset, offset));
        alpha *= radial * radial;
    }

    // Soft particles: instead of the billboard ending in the hard line where it cuts into the
    // floor, fade it out over the last few centimetres before contact. Reverse-Z with an infinite
    // far plane, so a stored depth linearises to near/depth and the sky (depth 0) is infinitely
    // far — which is the right answer here, since nothing was hit. The fragment's own view depth
    // is the interpolated 1/w, exact rather than taken from the sprite's centre.
    if ((flags & RENDER_SOFT) != 0u && in_soft_fade > 0.0)
    {
        float scene_distance = stored_depth > 0.0 ? cluster.depth.x / stored_depth : 1e9;
        float fragment_distance = 1.0 / gl_FragCoord.w;
        alpha *= clamp((scene_distance - fragment_distance) / in_soft_fade, 0.0, 1.0);
    }
    if (alpha <= 0.001)
        discard;

    // Lit particles receive the sun, the clustered punctual lights, and the SH ambient; the rest
    // stay emissive. Authored per emitter, so an additive spark shower and a lit smoke column can
    // share a draw. The sprite is shaded as a camera-facing hemisphere: the offset gives a
    // spherical normal, so a light rakes across the puff.
    if ((flags & RENDER_LIT) != 0u)
    {
        vec3 forward = normalize(cross(pc.camera_right.xyz, pc.camera_up.xyz));
        vec3 normal = normalize(pc.camera_right.xyz * offset.x + pc.camera_up.xyz * offset.y +
                                forward * sqrt(max(0.0, 1.0 - dot(offset, offset))));
        // Sun: world-space directional, so no camera-relative conversion — but its shadow
        // cascades are camera-relative, so the visibility lookup uses the carried centre.
        // The tap rotation is a frame-static screen-space hash: the pass binds no temporal
        // block, and particles are soft enough that a still pattern beats an unresolved one.
        float n_dot_l = max(dot(normal, pc.sun_direction.xyz), 0.0);
        float rotation = interleaved_gradient_noise(gl_FragCoord.xy) * 6.28318530718;
        float visibility = particle_sun_shadow(shadow_atlas, in_light.xyz, in_light.w, rotation);
        vec3 sun = pc.sun_radiance.rgb * (n_dot_l * visibility);
        // Clustered punctual lights: camera-relative, from the sprite's carried centre + depth.
        vec3 punctual = clustered_particle_light(normal, in_light.xyz, in_light.w, gl_FragCoord.xy);
        // Ambient: the captured environment's SH irradiance, scaled like the mesh path (0 = IBL off).
        vec3 ambient = gi_sh_irradiance(irradiance_sh.coeff, normal) * pc.sun_radiance.w;
        rgb *= sun + punctual + ambient;
    }

    // Premultiplied output: additive reads (rgb*a) + dst, alpha reads (rgb*a) + dst*(1-a).
    out_color = vec4(rgb * alpha, alpha);
}
