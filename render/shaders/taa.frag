#version 450
#extension GL_GOOGLE_include_directive : require

// The temporal resolve: this frame's jittered samples blended into the accumulated
// history, reprojected by the motion vectors the geometry pass wrote.
//
// The three mechanisms that make it work rather than smear:
//   * velocity dilation — the motion vector is taken from the closest surface in a 3x3
//     neighbourhood, so a moving silhouette drags its own edge instead of leaving a
//     trail of background motion behind it;
//   * neighbourhood clipping in YCoCg — the reprojected history is clipped into the
//     colour range this frame's neighbourhood actually contains, which is what discards
//     a sample of a surface that has since been occluded. YCoCg because clipping in a
//     luma/chroma basis keeps the box tight around the real colour distribution, where
//     an RGB box is loose and lets ghosts through;
//   * tone weighting — the blend is done on values compressed by 1/(1+luma), so one
//     very bright sample cannot dominate the average and flash as a firefly.
//
// Pixels with no geometry (the sky and the clouds over it) carry no motion vector, so
// their reprojection is derived from the view ray instead: an infinitely distant
// direction moves under camera rotation alone, which the previous world-to-clip
// applies exactly.
//
// When the render extent is below the output extent this is also the upscale: the
// history lives on the output grid, and because every frame samples a different
// sub-pixel position, accumulating render-resolution samples into it recovers detail no
// spatial filter could. Reconstruction is a Catmull-Rom fetch of both inputs rather
// than a full sample-weighting solve, so it trades some sharpness for a great deal of
// simplicity and stability.

#include "temporal_common.glsl"

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

layout(set = 0, binding = 1) uniform sampler2D scene_texture;    // this frame, render extent
layout(set = 0, binding = 2) uniform sampler2D history_texture;  // last frame, output extent
layout(set = 0, binding = 3) uniform sampler2D velocity_texture; // render extent
layout(set = 0, binding = 4) uniform sampler2D depth_texture;    // render extent, reverse-Z

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

// The strength of the neighbourhood clip, in standard deviations. Around one is tight
// enough to kill ghosting and loose enough not to reject the legitimate variation a
// jittered sample has against its neighbours.
const float CLIP_SIGMA = 1.25;

vec3 rgb_to_ycocg(vec3 c)
{
    return vec3(0.25 * c.r + 0.5 * c.g + 0.25 * c.b,
                0.5 * c.r - 0.5 * c.b,
                -0.25 * c.r + 0.5 * c.g - 0.25 * c.b);
}

vec3 ycocg_to_rgb(vec3 c)
{
    float t = c.x - c.z;
    return vec3(t + c.y, c.x + c.z, t - c.y);
}

float luminance(vec3 c)
{
    return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

// Karis' reversible tone weighting: compress before averaging so a single very bright
// sample carries its fair share rather than all of it, then expand afterwards.
vec3 tone_compress(vec3 c) { return c / (1.0 + luminance(c)); }
vec3 tone_expand(vec3 c) { return c / max(1.0 - luminance(c), 1e-4); }

// Clips a point to a box along the line joining it to the box centre. Clipping, not
// clamping: a clamp moves the sample to the nearest corner and shifts its hue, while
// clipping keeps it on the line and only shortens it.
vec3 clip_to_box(vec3 point, vec3 centre, vec3 extent)
{
    vec3 offset = point - centre;
    vec3 unit = offset / max(extent, vec3(1e-5));
    vec3 magnitude = abs(unit);
    float largest = max(magnitude.x, max(magnitude.y, magnitude.z));
    return largest > 1.0 ? centre + offset / largest : point;
}

// Bicubic Catmull-Rom through five bilinear taps. Reads the history without the
// softening a plain bilinear fetch would compound frame after frame, which is what
// keeps a stationary image from dissolving.
vec3 sample_catmull_rom(sampler2D image, vec2 uv, vec2 extent)
{
    vec2 texel = 1.0 / extent;
    vec2 sample_position = uv * extent;
    vec2 centre = floor(sample_position - 0.5) + 0.5;
    vec2 f = sample_position - centre;

    vec2 w0 = f * (-0.5 + f * (1.0 - 0.5 * f));
    vec2 w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
    vec2 w2 = f * (0.5 + f * (2.0 - 1.5 * f));
    vec2 w3 = f * f * (-0.5 + 0.5 * f);

    vec2 w12 = w1 + w2;
    vec2 offset12 = w2 / max(w12, vec2(1e-5));

    vec2 uv0 = (centre - 1.0) * texel;
    vec2 uv3 = (centre + 2.0) * texel;
    vec2 uv12 = (centre + offset12) * texel;

    vec3 result = texture(image, vec2(uv12.x, uv0.y)).rgb * (w12.x * w0.y);
    result += texture(image, vec2(uv0.x, uv12.y)).rgb * (w0.x * w12.y);
    result += texture(image, vec2(uv12.x, uv12.y)).rgb * (w12.x * w12.y);
    result += texture(image, vec2(uv3.x, uv12.y)).rgb * (w3.x * w12.y);
    result += texture(image, vec2(uv12.x, uv3.y)).rgb * (w12.x * w3.y);

    float weight = w12.x * w0.y + w0.x * w12.y + w12.x * w12.y + w3.x * w12.y +
                   w12.x * w3.y;
    return max(result / max(weight, 1e-5), vec3(0.0));
}

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec2 render_extent = temporal.resolution.xy;
    vec2 output_extent = temporal.resolution.zw;
    vec2 render_texel = 1.0 / render_extent;

    // Velocity dilation. Reverse-Z puts the nearest surface at the largest depth, so
    // the neighbour to follow is the one with the highest value.
    vec2 motion_uv = uv;
    float closest_depth = -1.0;
    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            vec2 tap = uv + vec2(x, y) * render_texel;
            float depth = texture(depth_texture, tap).r;
            if (depth > closest_depth)
            {
                closest_depth = depth;
                motion_uv = tap;
            }
        }
    }

    vec2 velocity;
    if (closest_depth <= 0.0)
    {
        // Nothing was drawn here: the whole neighbourhood is sky. Reproject the view
        // ray, which is exact for a direction at infinity and close enough for the
        // clouds, whose own depth the frame never records.
        vec3 ray = normalize(scene.cam_forward.xyz + v_ndc.x * scene.cam_right.xyz +
                             v_ndc.y * scene.cam_up.xyz);
        vec4 previous_clip = temporal.previous_view_projection * vec4(ray, 0.0);
        velocity = previous_clip.w > 0.0 ? uv - clip_to_uv(previous_clip) : vec2(0.0);
    }
    else
    {
        velocity = texture(velocity_texture, motion_uv).rg;
    }

    vec3 current = sample_catmull_rom(scene_texture, uv, render_extent);
    current = max(current, vec3(0.0));

    // The neighbourhood the history is allowed to live in, gathered at the render
    // extent because that is where this frame's samples actually are.
    vec3 moment_first = vec3(0.0);
    vec3 moment_second = vec3(0.0);
    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            vec3 tap = rgb_to_ycocg(
                tone_compress(max(texture(scene_texture, uv + vec2(x, y) * render_texel).rgb,
                                  vec3(0.0))));
            moment_first += tap;
            moment_second += tap * tap;
        }
    }
    const float SAMPLE_COUNT = 9.0;
    vec3 mean = moment_first / SAMPLE_COUNT;
    vec3 variance = max(moment_second / SAMPLE_COUNT - mean * mean, vec3(0.0));
    vec3 extent = sqrt(variance) * CLIP_SIGMA;

    vec2 history_uv = uv - velocity;
    bool history_inside = all(greaterThanEqual(history_uv, vec2(0.0))) &&
                          all(lessThanEqual(history_uv, vec2(1.0)));
    if (temporal.blend.w < 0.5 || !history_inside)
    {
        // No history at all, or it reprojects off screen: this frame is the answer.
        out_color = vec4(current, 1.0);
        return;
    }

    vec3 history = max(sample_catmull_rom(history_texture, history_uv, output_extent),
                       vec3(0.0));
    vec3 history_compressed = rgb_to_ycocg(tone_compress(history));
    if (temporal.thresholds.z > 0.5)
        history_compressed = clip_to_box(history_compressed, mean, extent);

    vec3 current_compressed = rgb_to_ycocg(tone_compress(current));

    // How much history to keep. Fast screen motion means the reprojection is least
    // trustworthy and the history least representative, so it is weighted down; a
    // still image keeps almost all of it, which is where the convergence comes from.
    float speed = length(velocity * output_extent);
    float motion = clamp(speed / 24.0, 0.0, 1.0);
    float feedback = mix(temporal.blend.x, temporal.blend.y, motion);

    vec3 resolved = mix(current_compressed, history_compressed, feedback);
    vec3 colour = tone_expand(ycocg_to_rgb(resolved));

    // Blending toward the history softens every frame; a small unsharp mask against
    // this frame's neighbourhood mean puts back what the blend took, without the
    // ringing a wider kernel would introduce.
    float sharpness = temporal.blend.z;
    if (sharpness > 0.0)
    {
        vec3 mean_rgb = tone_expand(ycocg_to_rgb(mean));
        colour = max(colour + (colour - mean_rgb) * sharpness, vec3(0.0));
    }

    out_color = vec4(colour, 1.0);
}
