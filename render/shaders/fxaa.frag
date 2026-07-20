#version 450

// The spatial anti-aliasing fallback: FXAA 3.11's quality path, run on the encoded LDR
// image after the display transform.
//
// It exists for the cases the temporal resolve cannot serve — the low tier, and a host
// that wants no frame-to-frame history at all — so it deliberately shares nothing with
// the temporal path: no motion vectors, no history, no jitter. One texture in, one out.
//
// Luminance is read off the gamma-encoded image on purpose. FXAA's thresholds are
// tuned for perceptual, not linear, contrast; running it on linear values would make it
// miss edges in the shadows and over-filter the highlights.

layout(set = 0, binding = 1) uniform sampler2D image;

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

// Contrast below which a pixel is left alone, as a fraction of the local maximum.
const float RELATIVE_THRESHOLD = 0.125;

// Absolute contrast floor, so near-flat regions are not filtered on noise.
const float ABSOLUTE_THRESHOLD = 0.0312;

// How far the edge search walks, in pixels, before giving up and using what it has.
const int SEARCH_STEPS = 12;

float luminance(vec3 colour)
{
    return dot(colour, vec3(0.299, 0.587, 0.114));
}

float sample_luminance(vec2 uv, vec2 texel, vec2 offset)
{
    return luminance(texture(image, uv + offset * texel).rgb);
}

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec2 texel = 1.0 / vec2(textureSize(image, 0));

    float centre = sample_luminance(uv, texel, vec2(0.0, 0.0));
    float north = sample_luminance(uv, texel, vec2(0.0, -1.0));
    float south = sample_luminance(uv, texel, vec2(0.0, 1.0));
    float west = sample_luminance(uv, texel, vec2(-1.0, 0.0));
    float east = sample_luminance(uv, texel, vec2(1.0, 0.0));

    float lowest = min(centre, min(min(north, south), min(west, east)));
    float highest = max(centre, max(max(north, south), max(west, east)));
    float contrast = highest - lowest;
    if (contrast < max(ABSOLUTE_THRESHOLD, highest * RELATIVE_THRESHOLD))
    {
        out_color = vec4(texture(image, uv).rgb, 1.0);
        return;
    }

    float north_west = sample_luminance(uv, texel, vec2(-1.0, -1.0));
    float north_east = sample_luminance(uv, texel, vec2(1.0, -1.0));
    float south_west = sample_luminance(uv, texel, vec2(-1.0, 1.0));
    float south_east = sample_luminance(uv, texel, vec2(1.0, 1.0));

    // Which way the edge runs: the axis whose second derivative is larger is the axis
    // the edge crosses, so the blend must travel along the other one.
    float horizontal = abs(north + south - 2.0 * centre) * 2.0 +
                       abs(north_east + south_east - 2.0 * east) +
                       abs(north_west + south_west - 2.0 * west);
    float vertical = abs(east + west - 2.0 * centre) * 2.0 +
                     abs(north_east + north_west - 2.0 * north) +
                     abs(south_east + south_west - 2.0 * south);
    bool is_horizontal = horizontal >= vertical;

    float negative = is_horizontal ? north : west;
    float positive = is_horizontal ? south : east;
    float negative_gradient = abs(negative - centre);
    float positive_gradient = abs(positive - centre);

    float step_length = is_horizontal ? texel.y : texel.x;
    float edge_luminance;
    float gradient;
    if (positive_gradient >= negative_gradient)
    {
        edge_luminance = (positive + centre) * 0.5;
        gradient = positive_gradient;
    }
    else
    {
        step_length = -step_length;
        edge_luminance = (negative + centre) * 0.5;
        gradient = negative_gradient;
    }

    // Walk both ways along the edge until the local contrast stops matching, which is
    // where the edge ends; the distances to the two ends give the blend factor.
    vec2 edge_uv = uv;
    if (is_horizontal)
        edge_uv.y += step_length * 0.5;
    else
        edge_uv.x += step_length * 0.5;

    vec2 walk = is_horizontal ? vec2(texel.x, 0.0) : vec2(0.0, texel.y);
    float scaled_gradient = gradient * 0.25;

    vec2 forward_uv = edge_uv + walk;
    vec2 backward_uv = edge_uv - walk;
    float forward_delta = sample_luminance(forward_uv, texel, vec2(0.0)) - edge_luminance;
    float backward_delta = sample_luminance(backward_uv, texel, vec2(0.0)) - edge_luminance;
    bool forward_done = abs(forward_delta) >= scaled_gradient;
    bool backward_done = abs(backward_delta) >= scaled_gradient;

    for (int i = 0; i < SEARCH_STEPS && !(forward_done && backward_done); ++i)
    {
        if (!forward_done)
        {
            forward_uv += walk;
            forward_delta = sample_luminance(forward_uv, texel, vec2(0.0)) - edge_luminance;
            forward_done = abs(forward_delta) >= scaled_gradient;
        }
        if (!backward_done)
        {
            backward_uv -= walk;
            backward_delta = sample_luminance(backward_uv, texel, vec2(0.0)) - edge_luminance;
            backward_done = abs(backward_delta) >= scaled_gradient;
        }
    }

    float forward_distance = is_horizontal ? forward_uv.x - uv.x : forward_uv.y - uv.y;
    float backward_distance = is_horizontal ? uv.x - backward_uv.x : uv.y - backward_uv.y;
    float nearest = min(forward_distance, backward_distance);
    float span = forward_distance + backward_distance;
    float blend = max(0.5 - nearest / max(span, 1e-6), 0.0);

    // A pixel whose own luminance is on the far side of the edge midpoint is outside
    // the span being filtered; blending it would drag the edge sideways.
    bool centre_is_darker = centre < edge_luminance;
    bool edge_is_darker = (forward_distance < backward_distance ? forward_delta
                                                                : backward_delta) < 0.0;
    if (centre_is_darker == edge_is_darker)
        blend = 0.0;

    vec2 final_uv = uv;
    if (is_horizontal)
        final_uv.y += step_length * blend;
    else
        final_uv.x += step_length * blend;

    out_color = vec4(texture(image, final_uv).rgb, 1.0);
}
