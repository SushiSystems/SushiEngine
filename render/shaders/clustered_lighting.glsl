// Clustered Forward+ punctual lighting, shared by the shading passes.
//
// The froxel grid is built once per frame by cluster_build.comp: for every cluster it
// writes how many lights touch it (cluster_light_count) and their indices into the
// scene light buffer (light_indices, a fixed slot of MAX_LIGHTS_PER_CLUSTER per
// cluster). A shading pass maps its pixel to a cluster from screen position + view
// depth, then loops only that cluster's lights. Include this AFTER pbr_common.glsl —
// it calls the BRDF there — and after the scene set is declared.
//
// The grid dimensions and per-cluster capacity are compile-time on both sides; they
// MUST match cluster_config.hpp. The per-frame quantities (light count, depth range,
// tile size) arrive in ClusterBlock.

#ifndef CLUSTERED_LIGHTING_GLSL
#define CLUSTERED_LIGHTING_GLSL

#define CLUSTER_X 16u
#define CLUSTER_Y 9u
#define CLUSTER_Z 24u
#define MAX_LIGHTS_PER_CLUSTER 64u
#define MAX_DECALS_PER_CLUSTER 16u
#define LIGHT_TYPE_SPOT 1.0

// One packed punctual light, matching the GpuLight lanes LightSystem writes. Positions
// are camera-relative (eye already subtracted), the same space the mesh fragment shades
// in, so a light-to-fragment vector is a plain difference.
struct PunctualLight
{
    vec4 position_range;  // xyz = camera-relative position, w = range
    vec4 color_intensity; // xyz = linear colour, w = radiance scale
    vec4 direction_type;  // xyz = spot axis, w = light type (0 point, 1 spot)
    vec4 cone;            // x = cos(outer), y = 1/(cos(inner)-cos(outer)), zw spare
};

layout(std430, set = 0, binding = 14) readonly buffer LightBuffer
{
    PunctualLight lights[];
} light_buffer;

layout(std430, set = 0, binding = 15) readonly buffer ClusterGrid
{
    uint cluster_light_count[];
} cluster_grid;

layout(std430, set = 0, binding = 16) readonly buffer LightIndexList
{
    uint light_indices[];
} light_index_list;

layout(set = 0, binding = 17) uniform ClusterBlock
{
    vec4 grid;   // x,y,z = grid dims, w = active light count
    vec4 depth;  // near, far, log-slice scale, log-slice bias
    vec4 screen; // render w, h, tile size x, tile size y
    vec4 counts; // x = active decal count, yzw spare
} cluster;

// The shared punctual (spot) shadow atlas and the per-caster matrices. A light's
// position_range/cone lanes carry a shadow record index (cone.z, -1 when unshadowed);
// the record projects the fragment into the caster's atlas tile.
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

// Projected box decals, culled into the same froxel grid as the lights. A decal is an
// oriented box; the shading pass projects the fragment into it and blends the tint.
struct Decal
{
    vec4 center_radius; // xyz = camera-relative centre, w = bounding radius (cull only)
    vec4 right_hx;      // xyz = unit right axis, w = half extent along it
    vec4 up_hy;         // xyz = unit up axis, w = half extent
    vec4 forward_hz;    // xyz = unit forward axis, w = half extent
    vec4 color_opacity; // xyz = tint, w = opacity
    vec4 maps;          // x = albedo, y = normal, z = orm bindless index (0xFFFFFFFF = none)
};

layout(std430, set = 0, binding = 20) readonly buffer DecalBuffer
{
    Decal decals[];
} decal_buffer;

layout(std430, set = 0, binding = 21) readonly buffer DecalGrid
{
    uint cluster_decal_count[];
} decal_grid;

layout(std430, set = 0, binding = 22) readonly buffer DecalIndexList
{
    uint decal_indices[];
} decal_index_list;

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

// One spot caster's visibility at a fragment: project into its atlas tile and filter.
// A single 2×2 hardware PCF tap leaves a hard, stair-stepped penumbra edge; a small
// Vogel-disc spread of comparison taps (each itself a free 2×2 average) softens it into
// a real penumbra. The disc rotates per pixel with the frame so the temporal resolve
// averages the residual — punctual lights only shade meshes, which carry motion vectors,
// so that resolve applies. Taps are clamped into the caster's own tile so none reads a
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

// Which cluster a pixel falls in, from its screen position and positive view-space
// depth. Mirrors the froxel bounds the build pass tests against, so a pixel and the
// build agree on cluster membership.
uint cluster_index_for(vec2 frag_coord, float view_z)
{
    uint cx = uint(clamp(frag_coord.x / max(cluster.screen.z, 1.0), 0.0, float(CLUSTER_X) - 1.0));
    uint cy = uint(clamp(frag_coord.y / max(cluster.screen.w, 1.0), 0.0, float(CLUSTER_Y) - 1.0));
    float slice = floor(log(max(view_z, cluster.depth.x)) * cluster.depth.z + cluster.depth.w);
    uint cz = uint(clamp(slice, 0.0, float(CLUSTER_Z) - 1.0));
    return cx + cy * CLUSTER_X + cz * CLUSTER_X * CLUSTER_Y;
}

// One punctual light's direct contribution, base BRDF only (the advanced lobes stay a
// sun-path feature for now). Falloff is windowed inverse-square (the Karis window), so
// a light reaches exactly zero at its range instead of being clipped hard.
vec3 shade_punctual(PunctualLight light, vec3 n, vec3 view_dir, vec3 world_pos,
                    vec3 albedo, vec3 f0, float roughness, float metallic, vec3 compensation)
{
    vec3 to_light = light.position_range.xyz - world_pos;
    float dist2 = dot(to_light, to_light);
    float inv_dist = inversesqrt(max(dist2, 1e-8));
    vec3 light_dir = to_light * inv_dist;

    float range = light.position_range.w;
    float attenuation = 1.0 / max(dist2, 1e-4);
    float ratio = dist2 / max(range * range, 1e-4);
    float window = clamp(1.0 - ratio * ratio, 0.0, 1.0);
    attenuation *= window * window;

    if (light.direction_type.w > 0.5) // spot
    {
        float cos_angle = dot(-light_dir, light.direction_type.xyz);
        float spot = clamp((cos_angle - light.cone.x) * light.cone.y, 0.0, 1.0);
        attenuation *= spot * spot;
    }

    if (attenuation <= 0.0)
        return vec3(0.0);

    // Shadow: a caster's cone.z holds its atlas record index (-1 = unshadowed). A spot has
    // one record; a point light has six (a cube), and the fragment's direction from the
    // light picks the face to add onto that base record.
    int shadow_record = int(light.cone.z);
    if (shadow_record >= 0)
    {
        int record = shadow_record;
        if (light.direction_type.w < 0.5) // point light: select the cube face
            record += cube_shadow_face(world_pos - light.position_range.xyz);
        attenuation *= sample_punctual_shadow(record, world_pos);
        if (attenuation <= 0.0)
            return vec3(0.0);
    }

    float n_dot_l = max(dot(n, light_dir), 0.0);
    if (n_dot_l <= 0.0)
        return vec3(0.0);

    vec3 half_vec = normalize(view_dir + light_dir);
    float n_dot_v = max(dot(n, view_dir), 1e-4);
    float n_dot_h = max(dot(n, half_vec), 0.0);
    float v_dot_h = max(dot(view_dir, half_vec), 0.0);

    vec3 f = f_schlick(v_dot_h, f0);
    vec3 specular = d_ggx(n_dot_h, roughness) *
                    v_smith_ggx_correlated(n_dot_v, n_dot_l, roughness) * f * compensation;
    vec3 diffuse = (vec3(1.0) - f) * (1.0 - metallic) * diffuse_lambert(albedo);

    vec3 radiance = light.color_intensity.xyz * light.color_intensity.w;
    return (diffuse + specular) * radiance * n_dot_l * attenuation;
}

// Blends every decal whose cluster this pixel is in over the surface, before shading. A
// decal projects along its forward axis: the fragment is taken into the box's local frame,
// and where it lands inside the box its tint (or a projected albedo texture) is blended
// over the surface albedo and, if it carries an ORM map, its occlusion/roughness/metallic
// override the surface's — so the decal is lit as material, not pasted on. All terms fade
// toward the box faces so the projection has no hard seam. Reads the bindless heap, so it
// must be included after `bindless_textures[]` is declared (pbr.frag does so).
void apply_clustered_decals(vec2 frag_coord, float view_z, vec3 world_pos, inout vec3 albedo,
                            inout float roughness, inout float metallic, inout float occlusion)
{
    if (cluster.counts.x < 0.5) // no decals this frame
        return;

    uint index = cluster_index_for(frag_coord, view_z);
    uint count = min(decal_grid.cluster_decal_count[index], MAX_DECALS_PER_CLUSTER);
    uint base = index * MAX_DECALS_PER_CLUSTER;

    for (uint i = 0u; i < count; ++i)
    {
        Decal decal = decal_buffer.decals[decal_index_list.decal_indices[base + i]];
        vec3 offset = world_pos - decal.center_radius.xyz;
        // Local coordinates in [-1,1] across each half extent.
        vec3 local = vec3(dot(offset, decal.right_hx.xyz) / max(decal.right_hx.w, 1e-4),
                          dot(offset, decal.up_hy.xyz) / max(decal.up_hy.w, 1e-4),
                          dot(offset, decal.forward_hz.xyz) / max(decal.forward_hz.w, 1e-4));
        if (abs(local.x) > 1.0 || abs(local.y) > 1.0 || abs(local.z) > 1.0)
            continue;
        // Soft edge: fade near the box faces so there is no hard border.
        float edge = (1.0 - smoothstep(0.8, 1.0, abs(local.x))) *
                     (1.0 - smoothstep(0.8, 1.0, abs(local.y))) *
                     (1.0 - smoothstep(0.8, 1.0, abs(local.z)));

        // The right/up axes are the decal's projection tangents, so their local
        // coordinates are the texture uv (mapped from [-1,1] to [0,1]).
        vec2 duv = local.xy * 0.5 + 0.5;

        vec3 tint = decal.color_opacity.xyz;
        float alpha = 1.0;
        uint albedo_idx = floatBitsToUint(decal.maps.x);
        if (albedo_idx != 0xFFFFFFFFu)
        {
            vec4 tex = texture(bindless_textures[nonuniformEXT(albedo_idx)], duv);
            tint = tex.rgb;
            alpha = tex.a; // a textured decal's own alpha cuts its silhouette
        }
        float weight = decal.color_opacity.w * edge * alpha;
        albedo = mix(albedo, tint, weight);

        uint orm_idx = floatBitsToUint(decal.maps.z);
        if (orm_idx != 0xFFFFFFFFu)
        {
            vec3 orm = texture(bindless_textures[nonuniformEXT(orm_idx)], duv).rgb;
            occlusion = mix(occlusion, orm.r, weight);
            roughness = mix(roughness, clamp(orm.g, 0.045, 1.0), weight);
            metallic = mix(metallic, orm.b, weight);
        }
    }
}

// Sums the direct contribution of every punctual light whose cluster this pixel is in.
vec3 accumulate_clustered_lighting(vec2 frag_coord, float view_z, vec3 n, vec3 view_dir,
                                   vec3 world_pos, vec3 albedo, vec3 f0, float roughness,
                                   float metallic, vec3 compensation)
{
    if (cluster.grid.w < 0.5) // no lights this frame
        return vec3(0.0);

    uint index = cluster_index_for(frag_coord, view_z);
    uint count = min(cluster_grid.cluster_light_count[index], MAX_LIGHTS_PER_CLUSTER);
    uint base = index * MAX_LIGHTS_PER_CLUSTER;

    vec3 result = vec3(0.0);
    for (uint i = 0u; i < count; ++i)
    {
        uint light_index = light_index_list.light_indices[base + i];
        result += shade_punctual(light_buffer.lights[light_index], n, view_dir, world_pos,
                                 albedo, f0, roughness, metallic, compensation);
    }
    return result;
}

#endif // CLUSTERED_LIGHTING_GLSL
