// Everything relating this frame to the frame before it, shared by the geometry pass
// that writes motion vectors and by the passes that consume them.
//
// Kept separate from the scene block on purpose: that one describes the world and is
// declared as a truncated prefix by most shaders, which makes appending to it fragile.
// This one is small enough that every shader needing any of it declares all of it.
//
// Every matrix here is camera-relative and translation-free, matching the scene block's
// view — the previous frame's eye is folded into the transforms the motion buffer packs,
// never into this matrix, so the two never both apply it.

layout(set = 0, binding = 9) uniform TemporalBlock
{
    mat4 previous_view_projection;
    vec4 jitter;      // xy = this frame's NDC jitter, zw = the previous frame's
    vec4 resolution;  // xy = internal render extent px, zw = output extent px
    vec4 blend;       // x = still feedback, y = moving feedback, z = sharpness, w = history valid
    vec4 thresholds;  // x = luminance, y = velocity, z = clamp history, w = spare
} temporal;

// Clip space to texture space. The projection is Y-flipped for Vulkan, so the same
// mapping serves both axes and no separate flip is needed anywhere downstream.
vec2 clip_to_uv(vec4 clip)
{
    return clip.xy / clip.w * 0.5 + 0.5;
}

// Where this pixel's surface was last frame, as a UV displacement. The current clip
// position carries this frame's sub-pixel jitter and the previous one carries none, so
// the jitter is removed here — otherwise every static pixel would report the jitter
// pattern as motion and the temporal resolve would chase it.
vec2 motion_vector(vec4 current_clip, vec4 previous_clip)
{
    vec2 current_uv = clip_to_uv(current_clip) - temporal.jitter.xy * 0.5;
    vec2 previous_uv = clip_to_uv(previous_clip);
    return current_uv - previous_uv;
}

// The interleaved-gradient hash every stochastic pass shares lives here now; the
// frame-independent form and the TPDF dither come with it. This block keeps only the
// temporal-block-dependent helpers below, which the shared file deliberately does not
// reach for (it binds no UBO).
#include "blue_noise.glsl"

// A per-frame scramble in [0,1), derived from the sub-pixel jitter. Each phase of the
// jitter sequence lands on a distinct value, so stochastic samplers advance exactly
// when the temporal resolve exists to average them — and hold still when anti-aliasing
// is off and the jitter is zero, where animated noise would only shimmer, unresolved.
float temporal_noise_phase()
{
    vec2 jitter_pixels = temporal.jitter.xy * temporal.resolution.xy;
    return fract(dot(jitter_pixels, vec2(5.588238, 7.213456)));
}

// The gradient noise advanced by this frame's phase: a stochastic sampler's per-pixel
// offset that is spatially even within a frame and decorrelated across frames.
float temporal_dither(vec2 pixel)
{
    return fract(interleaved_gradient_noise(pixel) + temporal_noise_phase());
}
