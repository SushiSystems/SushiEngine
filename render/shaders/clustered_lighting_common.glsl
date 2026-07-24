// The binding-free primitives of the Forward+ cluster grid, shared by every shader that
// reads it. Only structs, the compile-time grid dimensions, and pure functions that take
// their froxel parameters as arguments live here; the storage/UBO bindings themselves (the
// light buffer, the per-cluster count grid, the index list, the ClusterBlock) are declared
// by each including shader at whatever binding numbers its own descriptor set uses. That is
// what lets pbr.frag bind them on the shared scene set (bindings 14–22) while particle.frag
// binds a smaller subset on its own set — with one and only one copy of the cluster math.
//
// The grid dimensions and per-cluster capacity are compile-time on both sides; they MUST
// match cluster_config.hpp. The per-frame quantities (depth range, tile size, light count)
// are passed in, sourced from whatever ClusterBlock the caller declares.

#ifndef CLUSTERED_LIGHTING_COMMON_GLSL
#define CLUSTERED_LIGHTING_COMMON_GLSL

#define CLUSTER_X 16u
#define CLUSTER_Y 9u
#define CLUSTER_Z 24u
#define MAX_LIGHTS_PER_CLUSTER 64u
#define LIGHT_TYPE_SPOT 1.0

// One packed punctual light, matching the GpuLight lanes LightSystem writes. Positions
// are camera-relative (eye already subtracted), the same space the mesh fragment shades
// in, so a light-to-fragment vector is a plain difference.
struct PunctualLight
{
    vec4 position_range;  // xyz = camera-relative position, w = range
    vec4 color_intensity; // xyz = linear colour, w = radiance scale
    vec4 direction_type;  // xyz = spot axis, w = light type (0 point, 1 spot)
    vec4 cone;            // x = cos(outer), y = 1/(cos(inner)-cos(outer)), z = shadow record, w spare
};

// Which cluster a point falls in, from its screen position and positive view-space depth.
// The froxel bounds — the logarithmic depth slicing (depth.x = near, depth.z = log scale,
// depth.w = log bias) and the tile size (screen.zw) — arrive as parameters so a caller
// supplies them from whatever ClusterBlock binding it declares. Mirrors the froxel bounds
// cluster_build.comp tests against, so a pixel and the build agree on cluster membership.
uint cluster_index(vec2 frag_coord, float view_z, vec4 depth, vec4 screen)
{
    uint cx = uint(clamp(frag_coord.x / max(screen.z, 1.0), 0.0, float(CLUSTER_X) - 1.0));
    uint cy = uint(clamp(frag_coord.y / max(screen.w, 1.0), 0.0, float(CLUSTER_Y) - 1.0));
    float slice = floor(log(max(view_z, depth.x)) * depth.z + depth.w);
    uint cz = uint(clamp(slice, 0.0, float(CLUSTER_Z) - 1.0));
    return cx + cy * CLUSTER_X + cz * CLUSTER_X * CLUSTER_Y;
}

// The unshadowed attenuation of one light at a point: distance falloff windowed to the
// light's range (the Karis window, so a light reaches exactly zero at its range instead of
// being clipped hard), times the spot cone. Split out of any shading because both the mesh
// stochastic path and the particle path need it — the mesh once as the importance weight
// and once inside shading, the particle once for its diffuse puff.
float punctual_attenuation(PunctualLight light, vec3 world_pos, out vec3 light_dir,
                           out float distance_to_light)
{
    vec3 to_light = light.position_range.xyz - world_pos;
    float dist2 = dot(to_light, to_light);
    float inv_dist = inversesqrt(max(dist2, 1e-8));
    light_dir = to_light * inv_dist;
    distance_to_light = dist2 * inv_dist;

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
    return attenuation;
}

#endif // CLUSTERED_LIGHTING_COMMON_GLSL
