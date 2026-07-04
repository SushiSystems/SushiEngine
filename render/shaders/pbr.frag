#version 450

// Physically-based metallic-roughness shading for scene meshes. A Cook-Torrance
// specular BRDF (GGX distribution, Smith geometry, Schlick Fresnel) plus a Lambert
// diffuse, lit by the scene's single directional sun and an ambient floor. Material
// parameters arrive per-draw in the push constant; the sun and ambient come from the
// scene uniform block. Output is linear HDR (tonemapped in a later pass); the draw's
// picking id is written to the integer id target.

layout(set = 0, binding = 0) uniform SceneBlock
{
    mat4 view;
    mat4 proj;
    vec4 cam_forward;    // xyz basis, w = camera position x
    vec4 cam_right;      // xyz basis, w = camera position y
    vec4 cam_up;         // xyz basis, w = camera position z
    vec4 planet_center;
    vec4 planet_radii;
    vec4 sun_dir;        // xyz = direction to sun, w = intensity
    vec4 sun_color;      // xyz = colour, w = exposure
    vec4 ambient;        // xyz = ambient radiance
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
    mat4 model;
    vec4 albedo_metallic;
    vec4 emissive_roughness;
    vec4 outline_shift;
    uint entity_id;
    uint selected;
} pc;

layout(location = 0) in vec3 v_world_position;
layout(location = 1) in vec3 v_world_normal;

layout(location = 0) out vec4 out_color;
layout(location = 1) out uint out_id;

const float PI = 3.14159265359;

float distribution_ggx(float n_dot_h, float roughness)
{
    float a = roughness * roughness;
    float a2 = a * a;
    float d = n_dot_h * n_dot_h * (a2 - 1.0) + 1.0;
    return a2 / max(PI * d * d, 1e-7);
}

float geometry_smith(float n_dot_v, float n_dot_l, float roughness)
{
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    float gv = n_dot_v / (n_dot_v * (1.0 - k) + k);
    float gl = n_dot_l / (n_dot_l * (1.0 - k) + k);
    return gv * gl;
}

vec3 fresnel_schlick(float cos_theta, vec3 f0)
{
    return f0 + (1.0 - f0) * pow(clamp(1.0 - cos_theta, 0.0, 1.0), 5.0);
}

void main()
{
    vec3 albedo = pc.albedo_metallic.xyz;
    float metallic = pc.albedo_metallic.w;
    float roughness = clamp(pc.emissive_roughness.w, 0.045, 1.0);
    vec3 emissive = pc.emissive_roughness.xyz;

    // Meshes shade in camera-relative space: v_world_position is the fragment's offset
    // from the camera (the model matrix had the eye subtracted before upload), so the
    // camera sits at the origin of this frame and the view direction is just -position.
    vec3 n = normalize(v_world_normal);
    vec3 view_dir = normalize(-v_world_position);
    // Double-sided: flip the normal toward the viewer so back faces (e.g. cloth) light.
    if (dot(n, view_dir) < 0.0)
        n = -n;

    vec3 light_dir = normalize(scene.sun_dir.xyz);
    vec3 half_vec = normalize(view_dir + light_dir);

    float n_dot_v = max(dot(n, view_dir), 1e-4);
    float n_dot_l = max(dot(n, light_dir), 0.0);
    float n_dot_h = max(dot(n, half_vec), 0.0);
    float v_dot_h = max(dot(view_dir, half_vec), 0.0);

    vec3 f0 = mix(vec3(0.04), albedo, metallic);
    float d = distribution_ggx(n_dot_h, roughness);
    float g = geometry_smith(n_dot_v, n_dot_l, roughness);
    vec3 f = fresnel_schlick(v_dot_h, f0);

    vec3 specular = (d * g * f) / max(4.0 * n_dot_v * n_dot_l, 1e-4);
    vec3 kd = (vec3(1.0) - f) * (1.0 - metallic);
    vec3 diffuse = kd * albedo / PI;

    vec3 radiance = scene.sun_color.xyz * scene.sun_dir.w;
    vec3 lit = (diffuse + specular) * radiance * n_dot_l;
    vec3 ambient = scene.ambient.xyz * albedo;

    out_color = vec4(lit + ambient + emissive, 1.0);
    out_id = pc.entity_id;
}
