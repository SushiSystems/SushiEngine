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

#ifndef SHADOW_COMMON_GLSL
#define SHADOW_COMMON_GLSL

#define MAX_SHADOW_CASCADES 4

layout(set = 0, binding = 10) uniform ShadowBlock
{
    mat4 cascade_view_projection[MAX_SHADOW_CASCADES];
    vec4 splits;       // view distance each cascade reaches, metres
    vec4 texel_size;   // world metres one shadow texel covers, per cascade
    vec4 depth_range;  // world metres the [0,1] stored depth spans, per cascade
    vec4 params;       // x = cascade count, y = atlas tile uv scale, z = PCSS filter taps, w = cascade blend
    vec4 filter_size;  // x = min radius, y = max radius, z = penumbra per metre, w = blocker-search taps
    vec4 bias;         // x = depth bias, y = normal bias, z = contact metres, w = contact steps
    vec4 flags;        // x = shadows on, y = contact on, z = ray traced, w = cascade resolution
} shadows;

// A Vogel disc: tap i of count, spread by the golden angle so every prefix of the
// sequence already tiles the disc evenly. That even coverage is what makes a low tap
// count read smooth instead of grainy: a fixed Poisson set prints a visible speckle
// whenever a tier drops the tap count, and this needs no lookup table.
// `rotation` (radians) turns the whole disc per pixel: on meshes it advances each frame
// so the temporal resolve averages the residual, and on the analytic ground it is a
// stable screen-space value so the pattern holds still instead of shimmering unresolved.
vec2 vogel_disc(int i, int count, float rotation)
{
    float radius = sqrt((float(i) + 0.5) / float(count));
    float theta = float(i) * 2.399963229728653 + rotation; // golden angle
    return vec2(radius * cos(theta), radius * sin(theta));
}

// Which cascade covers a point this far down the view axis. Linear search over at most
// four entries, which is cheaper than any cleverness at this length and, unlike a
// depth-derived index, stays correct when the splits are retuned at runtime.
int select_shadow_cascade(float view_depth)
{
    int count = int(shadows.params.x);
    for (int i = 0; i < count - 1; ++i)
    {
        if (view_depth < shadows.splits[i])
            return i;
    }
    return count - 1;
}

// The atlas is a two-by-two grid of tiles, so a cascade's tile is its index read as two
// bits. One image means one descriptor and one pass.
vec2 shadow_tile_origin(int cascade)
{
    return vec2(float(cascade & 1), float(cascade >> 1)) * shadows.params.y;
}

// Keeps a filter tap inside its own tile. A tap reaching past the edge would read a
// completely unrelated cascade's depth, which is the one way a two-by-two atlas can go
// wrong that four separate images cannot.
vec2 shadow_tile_clamp(vec2 tile_uv, vec2 texel)
{
    float scale = shadows.params.y;
    return clamp(tile_uv, texel * 0.5, vec2(scale) - texel * 0.5);
}

// One shadow atlas texel in atlas UV. The atlas is a two-by-two grid of cascade tiles, so
// a tile's own resolution covers half the atlas on each axis.
vec2 shadow_atlas_texel()
{
    return vec2(1.0 / (shadows.flags.w * 2.0));
}

#endif // SHADOW_COMMON_GLSL

