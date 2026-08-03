#version 450
#extension GL_GOOGLE_include_directive : require

// The display transform: the whole grade -> tone-map -> encode chain in one fullscreen
// pass. It reads whatever the frame resolved to (temporal output, or the composited scene
// directly, possibly after depth-of-field and motion blur), applies the resolved exposure,
// a colour grade, one of three tone curves, the lens effects, and the sRGB encode onto the
// plain UNORM target the editor samples with ImGui.

layout(set = 0, binding = 1) uniform sampler2D source_texture;
layout(set = 0, binding = 2) uniform sampler2D bloom_texture;

// The post-processing parameter block. Its lanes mirror Scene::PostProcessUniforms exactly.
layout(set = 0, binding = 31) uniform PostBlock
{
    vec4 exposure; // x = linear exposure, y = tone-curve id, z = bloom intensity, w = frame index
    vec4 grade0;   // x = temperature, y = tint, z = contrast, w = saturation
    vec4 lift;     // xyz = shadow lift
    vec4 gamma;    // xyz = midtone gamma
    vec4 gain;     // xyz = highlight gain
    vec4 effects;  // x = vignette, y = chromatic aberration (px), z = film grain, w = bloom enabled
    vec4 dof;      // depth-of-field parameters (read by the DoF pass)
    vec4 motion;   // motion-blur parameters (read by the motion-blur pass)
    vec4 misc;     // x = render width, y = render height
} post;

#include "blue_noise.glsl"

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

const float GREY = 0.18;

// --- Tone curves -----------------------------------------------------------------
// Each takes a linear HDR colour and returns a linear display colour in [0,1]; the sRGB
// encode is applied once, after, so the three operators stay interchangeable.

// Narkowicz 2015 ACES filmic approximation.
vec3 tonemap_aces(vec3 x)
{
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

// Khronos PBR Neutral: compresses highlights toward white while holding material hue with
// minimal shift, the look-dev and product-render choice.
vec3 tonemap_neutral(vec3 color)
{
    const float start_compression = 0.8 - 0.04;
    const float desaturation = 0.15;
    float x = min(color.r, min(color.g, color.b));
    float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
    color -= offset;
    float peak = max(color.r, max(color.g, color.b));
    if (peak < start_compression)
        return color;
    float d = 1.0 - start_compression;
    float new_peak = 1.0 - d * d / (peak + d - start_compression);
    color *= new_peak / peak;
    float g = 1.0 - 1.0 / (desaturation * (peak - new_peak) + 1.0);
    return mix(color, new_peak * vec3(1.0), g);
}

// AgX (Troy Sobotka), the default: it desaturates toward white through the highlights so
// hue is preserved where a filmic curve skews it. The inset/outset matrices and the 6th-
// order sigmoid are the standard minimal approximation; the final power linearises AgX's
// sRGB-encoded output so it fits the shared encode below.
vec3 agx_contrast_approx(vec3 x)
{
    vec3 x2 = x * x;
    vec3 x4 = x2 * x2;
    return 15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4 - 6.868 * x2 * x + 0.4298 * x2 +
           0.1191 * x - 0.00232;
}

vec3 tonemap_agx(vec3 val)
{
    const mat3 agx_inset = mat3(0.842479062253094, 0.0423282422610123, 0.0423756549057051,
                                0.0784335999999992, 0.878468636469772, 0.0784336,
                                0.0792237451477643, 0.0791661274605434, 0.879142973793104);
    const mat3 agx_outset = mat3(1.19687900512017, -0.0528968517574562, -0.0529716355144438,
                                 -0.0980208811401368, 1.15190312990417, -0.0980434501171241,
                                 -0.0990297440797205, -0.0989611768448433, 1.15107367264116);
    const float min_ev = -12.47393;
    const float max_ev = 4.026069;

    val = agx_inset * val;
    val = clamp(log2(max(val, 1e-10)), min_ev, max_ev);
    val = (val - min_ev) / (max_ev - min_ev);
    val = agx_contrast_approx(val);
    val = agx_outset * val;
    // AgX emits an sRGB-encoded value; linearise it so the shared sRGB encode is applied once.
    return pow(max(val, vec3(0.0)), vec3(2.2));
}

vec3 apply_tonemap(vec3 hdr, int op)
{
    if (op == 1)
        return tonemap_aces(hdr);
    if (op == 2)
        return tonemap_neutral(hdr);
    return tonemap_agx(hdr);
}

// --- Colour grade ----------------------------------------------------------------

vec3 white_balance(vec3 color, float temperature, float tint)
{
    // A perceptual approximation, not a Planckian-locus fit: warmth trades blue for red,
    // tint trades magenta for green. Zero on both leaves the colour untouched.
    vec3 balance = vec3(1.0 + temperature * 0.1, 1.0 + tint * 0.1, 1.0 - temperature * 0.1);
    return color * balance;
}

vec3 lift_gamma_gain(vec3 color)
{
    color = color * post.gain.rgb + post.lift.rgb;
    color = max(color, 0.0);
    return pow(color, 1.0 / max(post.gamma.rgb, vec3(1e-3)));
}

vec3 contrast_saturation(vec3 color, float contrast, float saturation)
{
    color = (color - GREY) * contrast + GREY;
    color = max(color, 0.0);
    float luma = dot(color, vec3(0.2126, 0.7152, 0.0722));
    return mix(vec3(luma), color, saturation);
}

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec2 resolution = max(post.misc.xy, vec2(1.0));

    // Chromatic aberration: fetch the channels along the radial direction so the fringe
    // widens toward the corners. Zero width collapses to a single centred fetch.
    vec3 hdr;
    float ca = post.effects.y;
    if (ca > 0.0)
    {
        vec2 dir = (uv - 0.5) * (ca / resolution);
        hdr.r = texture(source_texture, uv + dir).r;
        hdr.g = texture(source_texture, uv).g;
        hdr.b = texture(source_texture, uv - dir).b;
    }
    else
    {
        hdr = texture(source_texture, uv).rgb;
    }

    // Bloom is added in the same scene-linear space as the scene, before exposure, so the
    // halo is exposed and tone-mapped with everything else. effects.w is set only when the
    // pyramid actually ran this frame; otherwise the bound texture is a placeholder ignored here.
    if (post.effects.w > 0.5)
        hdr += texture(bloom_texture, uv).rgb * post.exposure.z;

    hdr *= post.exposure.x;

    // Vignette darkens the corners before the tone map, so highlights roll off naturally
    // rather than being flattened after the curve.
    float vignette = post.effects.x;
    if (vignette > 0.0)
    {
        float d = length(uv - 0.5) * 1.4142136;
        hdr *= 1.0 - vignette * smoothstep(0.4, 1.0, d);
    }

    hdr = white_balance(hdr, post.grade0.x, post.grade0.y);
    hdr = lift_gamma_gain(hdr);
    hdr = contrast_saturation(hdr, post.grade0.z, post.grade0.w);

    vec3 mapped = apply_tonemap(hdr, int(post.exposure.y + 0.5));

    // Film grain in display-linear space, before the encode: an animated triangular-PDF
    // draw so it reads as photographic grain rather than a still overlay.
    float grain = post.effects.z;
    if (grain > 0.0)
    {
        uint frame = uint(post.exposure.w);
        float n = interleaved_gradient_noise(gl_FragCoord.xy, frame) - 0.5;
        mapped += n * grain;
        mapped = max(mapped, 0.0);
    }

    mapped = pow(mapped, vec3(1.0 / 2.2));

    // Dither before the 8-bit UNORM quantize. A smooth HDR gradient otherwise snaps onto the
    // 256 output levels and prints the steps between them as visible bands; a triangular-PDF
    // dither of one LSB spreads each step's rounding across its boundary so the band
    // dissolves into grain below the eye's threshold. Static in screen space: the temporal
    // resolve has already run, so an animated dither would only add a shimmer it can no
    // longer average out.
    vec3 dither = vec3(tpdf_dither(gl_FragCoord.xy, 1.0 / 255.0));
    out_color = vec4(mapped + dither, 1.0);
}
