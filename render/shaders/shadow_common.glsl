// Where the sun's shadow cascades sit this frame, and how a surface samples them.
//
// A block of its own for the same reason the temporal one is: the scene block describes
// the world and is declared as a truncated prefix by most shaders, so appending to it is
// fragile, while this has different readers and changes for a different reason.
//
// Every matrix here is camera-relative, matching every position the shading works in —
// the cascades are fitted around the eye, which is the origin.
//
// This file is only the block and the arithmetic every stage can do. The sampling
// itself lives in shadow_sampling.glsl, because it needs samplers and the fragment
// position — and the shadow pass's own vertex shader, which needs nothing but these
// matrices, cannot compile a file that mentions either.

#define MAX_SHADOW_CASCADES 4

layout(set = 0, binding = 10) uniform ShadowBlock
{
    mat4 cascade_view_projection[MAX_SHADOW_CASCADES];
    vec4 splits;       // view distance each cascade reaches, metres
    vec4 texel_size;   // world metres one shadow texel covers, per cascade
    vec4 depth_range;  // world metres the [0,1] stored depth spans, per cascade
    vec4 params;       // x = cascade count, y = atlas tile uv scale, w = cascade blend
    vec4 filter_size;  // x = min radius, y = max radius, z = penumbra per metre
    vec4 bias;         // x = depth bias, y = normal bias, z = contact metres, w = contact steps
    vec4 flags;        // x = shadows on, y = contact on, z = ray traced, w = cascade resolution
} shadows;

