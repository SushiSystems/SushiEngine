#version 450
#extension GL_GOOGLE_include_directive : require

// Screen-space reflections: for a smooth surface, trace its mirror ray through the depth
// buffer and paint back the scene colour it lands on, so a wet runway or a polished hull
// reflects the actual geometry around it rather than only the distant IBL cube.
//
// Deliberately a sharp mirror trace, no stochastic ray jitter: the whole engine is held to
// a smooth, shimmer-free bar, and a one-sample-per-pixel stochastic SSR is noise that would
// need a temporal denoiser to hide. Instead the reflection is gated to genuinely smooth
// surfaces (rough ones keep the IBL term) and added with a Fresnel weight, so it is strong
// at grazing angles and quiet head-on — where real reflections live — and never speckles.
// The surface normal is reconstructed from depth (there is no normal G-buffer); roughness
// and reflectance come from the thin G-buffer the opaque pass writes.

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
    vec4 misc;           // x = near plane
} scene;

layout(set = 0, binding = 1) uniform sampler2D scene_color;   // the lit HDR scene
layout(set = 0, binding = 2) uniform sampler2D depth_texture; // full-res depth
layout(set = 0, binding = 3) uniform sampler2D gbuffer;       // r = roughness, g = F0
layout(set = 0, binding = 4) uniform sampler2D hiz_texture;   // level 0 = linear view distance

layout(push_constant) uniform Push
{
    vec4 p0; // x = max steps, y = thickness (m), z = roughness cutoff, w = intensity
    vec4 p1; // x = trace enabled
} pc;

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

float linear_depth(float d) { return d > 0.0 ? scene.misc.x / d : 0.0; }

vec3 view_pos(vec2 uv, float d)
{
    float dist = linear_depth(d);
    float tan_x = scene.proj[0][0] != 0.0 ? 1.0 / scene.proj[0][0] : 1.0;
    float tan_y = scene.proj[1][1] != 0.0 ? 1.0 / scene.proj[1][1] : 1.0;
    vec2 ndc = uv * 2.0 - 1.0;
    return vec3(ndc.x * tan_x * dist, ndc.y * tan_y * dist, -dist);
}

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec3 base = texture(scene_color, uv).rgb;
    out_color = vec4(base, 1.0);

    float d = texture(depth_texture, uv).r;
    if (pc.p1.x < 0.5 || d <= 0.0)
        return; // tracing off, or sky — nothing to reflect from

    vec2 gb = texture(gbuffer, uv).rg;
    float roughness = gb.r;
    float f0 = gb.g;
    float cutoff = pc.p0.z;
    // Only genuinely smooth surfaces reflect; rougher ones keep their IBL specular.
    float smooth_weight = 1.0 - smoothstep(cutoff * 0.6, cutoff, roughness);
    if (smooth_weight <= 0.0)
        return;

    vec2 texel = 1.0 / vec2(textureSize(depth_texture, 0));
    vec3 P = view_pos(uv, d);
    vec3 V = normalize(-P);

    // Geometric view normal from the smaller depth delta on each axis (no normal buffer).
    vec3 Pl = view_pos(uv - vec2(texel.x, 0.0), texture(depth_texture, uv - vec2(texel.x, 0.0)).r);
    vec3 Pr = view_pos(uv + vec2(texel.x, 0.0), texture(depth_texture, uv + vec2(texel.x, 0.0)).r);
    vec3 Pd = view_pos(uv - vec2(0.0, texel.y), texture(depth_texture, uv - vec2(0.0, texel.y)).r);
    vec3 Pu = view_pos(uv + vec2(0.0, texel.y), texture(depth_texture, uv + vec2(0.0, texel.y)).r);
    vec3 ddx = (abs(Pr.z - P.z) < abs(P.z - Pl.z)) ? (Pr - P) : (P - Pl);
    vec3 ddy = (abs(Pu.z - P.z) < abs(P.z - Pd.z)) ? (Pu - P) : (P - Pd);
    vec3 N = normalize(cross(ddx, ddy));
    if (dot(N, V) < 0.0)
        N = -N;

    vec3 R = reflect(-V, N); // reflection direction, view space
    if (R.z > 0.0)
        return; // pointing back toward the camera plane: unreliable, skip

    float thickness = pc.p0.y;
    int max_steps = int(pc.p0.x);
    float t = 0.0;
    float step = max(0.05, -P.z * 0.02); // ~2% of view distance
    bool hit = false;
    vec2 hit_uv = uv;

    for (int i = 0; i < max_steps; ++i)
    {
        t += step;
        vec3 Q = P + R * t;
        if (Q.z >= -scene.misc.x)
            break; // in front of the near plane
        vec4 clip = scene.proj * vec4(Q, 1.0);
        if (clip.w <= 0.0)
            break;
        vec2 suv = clip.xy / clip.w * 0.5 + 0.5;
        if (any(lessThan(suv, vec2(0.0))) || any(greaterThan(suv, vec2(1.0))))
            break; // left the screen

        float surf = textureLod(hiz_texture, suv, 0.0).r; // nearest surface distance (level 0)
        float ray = -Q.z;
        float delta = ray - surf; // > 0: the ray is behind the surface
        if (delta > 0.0)
        {
            if (delta < thickness)
            {
                hit = true;
                hit_uv = suv;
            }
            else
            {
                // Overshot past the surface: binary-refine the crossing.
                float ta = t, tb = t - step;
                for (int j = 0; j < 6; ++j)
                {
                    float tm = (ta + tb) * 0.5;
                    vec3 Qm = P + R * tm;
                    vec4 cm = scene.proj * vec4(Qm, 1.0);
                    vec2 um = cm.xy / cm.w * 0.5 + 0.5;
                    float dl = -Qm.z - textureLod(hiz_texture, um, 0.0).r;
                    if (dl > 0.0) { ta = tm; hit_uv = um; }
                    else tb = tm;
                }
                vec3 Qf = P + R * ta;
                hit = (-Qf.z - textureLod(hiz_texture, hit_uv, 0.0).r) < thickness;
            }
            break;
        }
        step *= 1.3; // grow the step in the empty stretch
    }

    if (!hit)
        return;

    vec3 reflected = textureLod(scene_color, hit_uv, 0.0).rgb;

    // Fresnel: quiet head-on, strong at grazing, exactly where reflections read.
    float n_dot_v = max(dot(N, V), 1e-3);
    float fresnel = f0 + (1.0 - f0) * pow(1.0 - n_dot_v, 5.0);

    // Fade toward the screen edges so a reflection does not stop hard at the border.
    vec2 edge = smoothstep(vec2(0.0), vec2(0.08), hit_uv) *
                (1.0 - smoothstep(vec2(0.92), vec2(1.0), hit_uv));
    float weight = smooth_weight * fresnel * edge.x * edge.y * pc.p0.w;

    out_color = vec4(base + reflected * weight, 1.0);
}
