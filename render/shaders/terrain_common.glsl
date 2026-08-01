// The GLSL half of include/SushiEngine/terrain/cube_sphere.hpp.
//
// A transcription, not a reimplementation: the host header is the authority, and a
// conformance test holds the two to a stated tolerance. Where the two must agree exactly
// is the *shape* of the arithmetic -- §9.2's difference form is only well-conditioned in
// the order written, so rearranging it here to save an operation would silently give back
// the metre of error it exists to remove.
//
// See docs/slop/solar_system_overhaul.md §4 (the cube-sphere map) and §9 (why the
// difference is evaluated rather than the two positions).

const float TERRAIN_QUARTER_PI = 0.78539816339744831;

// Vertices and cells per side of the grid mesh one node is drawn with. Mirrors
// Terrain::NODE_GRID_CELLS.
const uint TERRAIN_GRID_CELLS = 32u;
const uint TERRAIN_GRID_VERTICES = TERRAIN_GRID_CELLS + 1u;

// Set 2 is terrain's own, because the per-frame set 0 is full at its guaranteed 32
// bindings (scene_layout.hpp) and terrain must not be the thing that overflows it. Set 0
// and set 1 stay exactly what every other scene pipeline binds, which is what lets the
// existing shading path light this geometry with no parallel code.

// One selected node, as the host packed it. Every field is small: the origin is
// camera-relative and the centre is a cube point of order one, which is §9's whole point.
struct TerrainNodeRecord
{
    vec4 origin;     // xyz = node centre's camera-relative surface point, w = grid span
    vec4 centre;     // xyz = node centre's cube point, w = cube face index
    vec4 grid_morph; // xy = grid-space origin, z = morph start metres, w = morph end
    vec4 decode;     // x/y = the slot's elevation range, z = slot layer, w = spare
    vec4 uv_rect;    // xy = uv offset into the slot, zw = uv scale
};

layout(std430, set = 2, binding = 0) readonly buffer TerrainNodeBlock
{
    TerrainNodeRecord nodes[];
} terrain_nodes;

// The height slot pool: one normalized R16 layer per resident tile, decoded against the
// per-node range above. A node whose own tile is not resident is pointed at an ancestor's
// layer with a scaled uv_rect instead, which is why the rect is per node rather than
// implied by the layer.
layout(set = 2, binding = 1) uniform sampler2DArray terrain_height_slots;

layout(set = 2, binding = 2) uniform TerrainBodyBlock
{
    vec4 semi_axes;      // xyz = the reference ellipsoid's semi-axes, metres; w = spare
    // Rotation only, no translation: every position below is already camera-relative, and
    // rotating a difference of two body-fixed points is the same rotation. This is what
    // carries terrain out of the body's own frame -- where the cube-sphere map, the
    // ellipsoid and the elevations all live -- into the scene frame the camera is in, and
    // it is what makes terrain turn with the body's spin rather than under it.
    mat4 body_to_scene;
} terrain_body;

// The body's semi-axes. Single precision is harmless here even though the values are
// planetary: they only ever scale the *difference* vector below, whose magnitude is a
// node's own span, so their rounding is a relative error on a small quantity.
vec3 terrain_semi_axes() { return terrain_body.semi_axes.xyz; }

// The cube point a face coordinate names, in the cubemap face basis. Mirrors
// Terrain::face_direction; the branch is uniform across an instance, so it costs a
// scalar select rather than divergence.
vec3 terrain_face_direction(int face, float s, float t)
{
    if (face == 0) return vec3( 1.0,   -t,   -s);
    if (face == 1) return vec3(-1.0,   -t,    s);
    if (face == 2) return vec3(   s,  1.0,    t);
    if (face == 3) return vec3(   s, -1.0,   -t);
    if (face == 4) return vec3(   s,   -t,  1.0);
    return               vec3(  -s,   -t, -1.0);
}

// normalize(centre + offset) - normalize(centre), with no subtraction of nearly-equal
// quantities left in it. Mirrors Terrain::normalized_difference.
//
// The naive form evaluates both normalizations and subtracts, which at planetary radius
// lands about a metre off in float32 -- three and a half cells at the deepest level. This
// form's error is proportional to the offset instead of to the body's radius, which is
// what makes the whole vertex path single precision.
vec3 terrain_normalized_difference(vec3 centre, vec3 offset)
{
    vec3 sum = centre + offset;
    float length_centre = length(centre);
    float length_sum = length(sum);
    float excess = 2.0 * dot(centre, offset) + dot(offset, offset);
    float scale = excess / (length_sum * length_centre * (length_centre + length_sum));
    return offset / length_sum - centre * scale;
}

// Where a node's grid parameter lands, and which way is up there.
//
// `centre_cube` is the node centre's un-normalized cube point (the `c` of §9.2) and
// `origin` is that centre's surface point already made camera-relative on the host in
// double -- the pair is what keeps this function's inputs and outputs small.
//
// @param centre_cube The node centre's cube point.
// @param face        Which cube face, 0..5.
// @param grid_min    The node's grid-space origin (s, t).
// @param span        The node's grid-space extent, both axes.
// @param parameter   Position within the node, each component in [0, 1].
// @param origin      The node centre's camera-relative surface point, metres.
// @param position    Receives the camera-relative point on the reference surface.
// @param direction   Receives the unit direction from the body centre.
void terrain_surface(vec3 centre_cube, int face, vec2 grid_min, float span, vec2 parameter,
                     vec3 origin, out vec3 position, out vec3 direction)
{
    vec2 grid = grid_min + parameter * span;
    vec2 warped = tan(grid * TERRAIN_QUARTER_PI);
    vec3 point = terrain_face_direction(face, warped.x, warped.y);
    vec3 difference = terrain_normalized_difference(centre_cube, point - centre_cube);
    direction = normalize(centre_cube) + difference;
    position = origin + terrain_semi_axes() * difference;
}

// The outward geodetic normal at a unit direction.
//
// The gradient of the ellipsoid's implicit form depends only on the direction, never on
// the absolute position -- which is the reason it can be evaluated here at all, since the
// absolute position is the one quantity this shader never has.
vec3 terrain_geodetic_normal(vec3 direction)
{
    return normalize(direction / terrain_semi_axes());
}
