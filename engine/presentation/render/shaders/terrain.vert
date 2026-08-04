#version 450
#extension GL_GOOGLE_include_directive : require

// Planetary terrain, drawn as one instanced grid per selected node
// (docs/design/solar_system_overhaul.md §8).
//
// The output signature is mesh.vert's exactly, so the existing pbr.frag shades this
// geometry with no changes: terrain is lit, shadowed, fogged and tone-mapped by the path
// that already exists rather than by a parallel one of its own.
//
// There is no vertex buffer. A node is a fixed 33x33 lattice, so the grid position comes
// from gl_VertexIndex and the only per-instance state is the node record. Three things
// then happen that a mesh vertex shader does not do:
//
//   - the position is built from §9.2's difference form, never from a planet-space
//     coordinate, which is what keeps this single precision at planetary radius;
//   - the lattice is morphed toward the parent's before the height is sampled, which is
//     what removes both the crack and the pop at an LOD boundary;
//   - the normal comes from the height field rather than from the ellipsoid, because the
//     ellipsoid normal would shade a perfectly smooth sphere with the relief invisible.

#include "temporal_common.glsl"
#include "terrain_common.glsl"

layout(set = 0, binding = 0) uniform SceneBlock
{
    mat4 view;
    mat4 proj;
    vec4 cam_forward;
    vec4 cam_right;
    vec4 cam_up;
    vec4 planet_center;
    vec4 planet_radii;
    vec4 sun_dir;
    vec4 sun_color;
    vec4 ambient;
    vec4 rayleigh;
    vec4 scatter;
    vec4 ground_albedo;
    vec4 ocean_color;
    vec4 cloud_params;
    vec4 star_params;
    vec4 misc;
} scene;

layout(push_constant) uniform Push
{
    uint material_index;
    uint entity_id;
    uint spare0;
    uint spare1;
} pc;

layout(location = 0) out vec3 v_world_position;
layout(location = 1) out vec3 v_world_normal;
layout(location = 2) out vec4 v_world_tangent;
layout(location = 3) out vec2 v_uv0;
layout(location = 4) out vec2 v_uv1;
layout(location = 5) out vec4 v_color;
layout(location = 6) out vec4 v_current_clip;
layout(location = 7) out vec4 v_previous_clip;
layout(location = 8) flat out uint v_material_index;
layout(location = 9) flat out uint v_entity_id;

// The elevation at a grid parameter, decoded from the node's slot.
float terrain_height(TerrainNodeRecord node, vec2 parameter)
{
    vec2 uv = node.uv_rect.xy + parameter * node.uv_rect.zw;
    float unit = texture(terrain_height_slots, vec3(uv, node.decode.z)).r;
    return mix(node.decode.x, node.decode.y, unit);
}

// The displaced surface point at a grid parameter: the reference surface plus the
// elevation along the geodetic normal, which is the datum every elevation model states
// its heights against.
vec3 terrain_point(TerrainNodeRecord node, int face, vec2 parameter)
{
    vec3 position;
    vec3 direction;
    terrain_surface(node.centre.xyz, face, node.grid_morph.xy, node.origin.w, parameter,
                    node.origin.xyz, position, direction);
    return position + terrain_geodetic_normal(direction) * terrain_height(node, parameter);
}

void main()
{
    TerrainNodeRecord node = terrain_nodes.nodes[gl_InstanceIndex];
    const int face = int(node.centre.w + 0.5);

    uint index = uint(gl_VertexIndex);
    vec2 lattice = vec2(float(index % TERRAIN_GRID_VERTICES),
                        float(index / TERRAIN_GRID_VERTICES));

    // The morph weight needs a distance and the distance needs a position, so the
    // un-morphed position is evaluated first and thrown away. Both evaluations are a few
    // dozen operations; the alternative -- morphing on the node's distance rather than the
    // vertex's -- makes the weight discontinuous across a node boundary, which is the
    // crack this exists to remove.
    vec3 unmorphed;
    vec3 unused_direction;
    terrain_surface(node.centre.xyz, face, node.grid_morph.xy, node.origin.w,
                    lattice / float(TERRAIN_GRID_CELLS), node.origin.xyz, unmorphed,
                    unused_direction);

    float span = max(node.grid_morph.w - node.grid_morph.z, 1.0e-6);
    float weight = clamp((length(unmorphed) - node.grid_morph.z) / span, 0.0, 1.0);

    // The CDLOD morph: an odd lattice index slides onto the even one below it as the
    // weight rises, so at full weight the lattice *is* the parent's and the swap happens
    // between two geometries that have already become identical.
    vec2 morphed = lattice - fract(lattice * 0.5) * 2.0 * weight;
    vec2 parameter = morphed / float(TERRAIN_GRID_CELLS);

    vec3 position;
    vec3 direction;
    terrain_surface(node.centre.xyz, face, node.grid_morph.xy, node.origin.w, parameter,
                    node.origin.xyz, position, direction);
    float height = terrain_height(node, parameter);
    position += terrain_geodetic_normal(direction) * height;

    // The surface normal from the height field, by forward differences one cell along each
    // grid axis. The step is taken in the *morphed* lattice so the normal belongs to the
    // geometry actually drawn rather than to the one it is morphing away from.
    float step_size = 1.0 / float(TERRAIN_GRID_CELLS);
    vec2 along_s = parameter + vec2(step_size, 0.0);
    vec2 along_t = parameter + vec2(0.0, step_size);
    vec3 tangent_s = terrain_point(node, face, along_s) - position;
    vec3 tangent_t = terrain_point(node, face, along_t) - position;

    // cross(t, s) is outward on every cube face -- the basis in terrain_face_direction
    // has one handedness throughout, which is worth having checked rather than assumed.
    vec3 normal = normalize(cross(tangent_t, tangent_s));
    if (dot(normal, terrain_geodetic_normal(direction)) < 0.0)
        normal = -normal;

    // Everything above is in the body's own frame, which is where the cube-sphere map and
    // the elevations are defined. The scene frame is the camera's, and the two differ by
    // the body's orientation and spin -- a rotation, applied once, here.
    mat3 to_scene = mat3(terrain_body.body_to_scene);
    position = to_scene * position;
    normal = to_scene * normal;
    vec3 scene_tangent = to_scene * tangent_s;

    v_world_position = position;
    v_world_normal = normal;
    v_world_tangent = vec4(normalize(scene_tangent), 1.0);
    // The tile parameter, so a later material pass can sample the class and colour slots
    // from the same rectangle this vertex read its height through.
    v_uv0 = node.uv_rect.xy + parameter * node.uv_rect.zw;
    v_uv1 = parameter;
    v_color = vec4(1.0);

    vec4 clip = scene.proj * scene.view * vec4(position, 1.0);
    v_current_clip = clip;
    // Terrain carries no per-object previous transform, so it restates itself against last
    // frame's eye the way any camera-relative geometry does -- the case temporal.eye_delta
    // exists for.
    v_previous_clip = temporal.previous_view_projection *
                      vec4(position + temporal.eye_delta.xyz, 1.0);
    v_material_index = pc.material_index;
    v_entity_id = pc.entity_id;
    gl_Position = clip;
}
