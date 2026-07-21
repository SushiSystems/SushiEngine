#version 450
#extension GL_GOOGLE_include_directive : require

// Screen-space contact shadows: the centimetres of occlusion a shadow cascade cannot
// resolve, recovered by marching the depth buffer toward the sun.
//
// A cascade texel covers tens of centimetres at the near end and metres at the far end,
// so the moment two surfaces meet — a box on the ground, a wheel on a runway — the
// cascade has nothing to say and the contact reads as floating. This pass says it: a
// short march from each surface toward the light, checking whether anything already
// drawn stands between. It is bounded in metres rather than in pixels, so what it
// recovers is a physical distance and not a resolution artefact.
//
// It runs on the depth prepass's output, which is why that prepass exists: the answer
// has to be known before the surface is shaded, and only a completed depth buffer can
// give it.

#include "shadow_common.glsl"

layout(set = 0, binding = 0) uniform SceneBlock
{
    mat4 view;
    mat4 proj;
    vec4 cam_forward;
    vec4 cam_right;
    vec4 cam_up;
    vec4 planet_center;
    vec4 planet_radii;
    vec4 sun_dir;        // xyz = direction to sun, camera-relative world space
    vec4 sun_color;
    vec4 ambient;
    vec4 rayleigh;
    vec4 scatter;
    vec4 ground_albedo;
    vec4 ocean_color;
    vec4 cloud_params;
    vec4 star_params;
    vec4 misc;           // x = near plane, z = time seconds
} scene;

// Interleaved gradient noise (Jimenez, Siggraph 2014); a local copy because this pass
// binds the scene and shadow blocks only, not the temporal block the shared helper in
// temporal_common.glsl reads its frame phase from.
float interleaved_gradient_noise(vec2 pixel)
{
    return fract(52.9829189 * fract(dot(pixel, vec2(0.06711056, 0.00583715))));
}

layout(set = 0, binding = 1) uniform sampler2D depth_texture;

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out float out_visibility;

// Reverse-Z with an infinite far plane: clip.z = near and clip.w = -view.z, so the
// stored depth is near / -view.z and the inverse is exact rather than a linearisation.
float view_depth_from(float depth)
{
    return depth > 0.0 ? scene.misc.x / depth : 0.0;
}

// The view-space position a pixel's stored depth stands for.
vec3 view_position(vec2 ndc, float depth)
{
    float distance = view_depth_from(depth);
    if (distance <= 0.0)
        return vec3(0.0);
    // The camera basis rows carry tan(fov/2) already, so dividing them out of the
    // projection recovers the ray without inverting a matrix.
    float tan_x = scene.proj[0][0] != 0.0 ? 1.0 / scene.proj[0][0] : 1.0;
    float tan_y = scene.proj[1][1] != 0.0 ? 1.0 / scene.proj[1][1] : 1.0;
    return vec3(ndc.x * tan_x * distance, ndc.y * tan_y * distance, -distance);
}

void main()
{
    out_visibility = 1.0;
    if (shadows.flags.x < 0.5 || shadows.flags.y < 0.5)
        return;

    vec2 uv = v_ndc * 0.5 + 0.5;
    float depth = texture(depth_texture, uv).r;
    if (depth <= 0.0)
        return; // sky: nothing in front of it to occlude

    vec3 origin = view_position(v_ndc, depth);
    // The view carries no translation, so its rotation alone takes the camera-relative
    // world direction into view space.
    vec3 light = normalize(mat3(scene.view) * scene.sun_dir.xyz);

    int steps = clamp(int(shadows.bias.w), 2, 32);
    float reach = shadows.bias.z;
    vec3 step_vector = light * (reach / float(steps));

    // A per-pixel start offset breaks the march's regular stride into noise the temporal
    // resolve averages away, which is far cheaper than the extra steps that would be
    // needed to hide the banding otherwise. The offset must change each frame for that
    // averaging to work — a static hash converges to speckle — so the gradient noise is
    // advanced along the golden ratio by a frame counter derived from the scene clock.
    float phase = float(int(scene.misc.z * 60.0) & 63) * 0.61803398875;
    float dither = fract(interleaved_gradient_noise(gl_FragCoord.xy) + phase);
    vec3 position = origin + step_vector * (0.5 + dither * 0.5);

    // Tolerance for "the sampled surface is the same one we started from": a thickness,
    // so the march steps over the front face it began on without treating everything
    // behind that face as an occluder.
    float thickness = reach * 0.5;

    for (int i = 0; i < steps; ++i)
    {
        vec4 clip = scene.proj * vec4(position, 1.0);
        if (clip.w <= 0.0)
            break;
        vec2 sample_ndc = clip.xy / clip.w;
        if (any(greaterThan(abs(sample_ndc), vec2(1.0))))
            break;

        vec2 sample_uv = sample_ndc * 0.5 + 0.5;
        float sampled_depth = texture(depth_texture, sample_uv).r;
        if (sampled_depth > 0.0)
        {
            float sampled_distance = view_depth_from(sampled_depth);
            float ray_distance = -position.z;
            float difference = ray_distance - sampled_distance;
            if (difference > 0.0 && difference < thickness)
            {
                // Fade with how far along the march the hit was: a hit at the very end of
                // the reach is the weakest evidence, and fading it is what keeps the
                // effect from ending in a hard line.
                out_visibility = clamp(float(i) / float(steps), 0.0, 1.0);
                return;
            }
        }
        position += step_vector;
    }
}
