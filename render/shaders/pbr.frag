#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

// Physically-based shading for scene meshes. The draw carries only a material index;
// every parameter and every map is read out of the global material array, and the
// maps themselves are sampled straight out of the bindless heap. Unset maps resolve
// to neutral defaults on the CPU side, so this shader never branches on whether a map
// exists — only on behaviour a default cannot stand in for (parallax, the optional
// lobes), which travels as flag bits.
//
// Output is linear HDR, tonemapped in a later pass; the draw's picking id goes to the
// integer id target.

#include "pbr_common.glsl"
#include "temporal_common.glsl"
#include "shadow_sampling.glsl"

layout(set = 0, binding = 0) uniform SceneBlock
{
    mat4 view;
    mat4 proj;
    vec4 cam_forward;
    vec4 cam_right;
    vec4 cam_up;
    vec4 planet_center;
    vec4 planet_radii;
    vec4 sun_dir;        // xyz = direction to sun, w = intensity
    vec4 sun_color;      // xyz = colour, w = exposure
    vec4 ambient;        // xyz = ambient radiance
    vec4 rayleigh;
    vec4 scatter;
    vec4 ground_albedo;
    vec4 ocean_color;
    vec4 cloud_global;
    vec4 star_params;
    vec4 misc;
    vec4 sky_counts;
    vec4 planet_frame;
    vec4 cloud_light;    // x = light absorption
    vec4 ibl_params;     // x = intensity, y = specular mip count, z = ambient mode
    // Declared only so the cloud shadow can reach the deck's weather scale; the block
    // continues past this point with the body and star arrays, which nothing here reads.
    vec4 cloud_deck_a[6];
    vec4 cloud_deck_b[6];
    vec4 cloud_deck_c[6];
    vec4 cloud_deck_d[6];
} scene;

#include "cloud_shadow_common.glsl"

struct GpuMaterial
{
    vec4 base_color;
    vec4 emissive;               // rgb premultiplied by intensity, a = normal scale
    vec4 surface;                // metallic, roughness, occlusion strength, alpha cutoff
    vec4 parallax;               // height scale, steps, ior, detail normal scale
    vec4 main_transform;         // tiling.xy, offset.xy
    vec4 detail_transform;
    vec4 anisotropy_clearcoat;   // anisotropy, rotation, clearcoat, clearcoat roughness
    vec4 sheen;                  // rgb colour, a = roughness
    vec4 transmission;           // transmission, thickness
    vec4 subsurface;
    uvec4 maps_a;                // albedo, metallic-roughness, normal, height
    uvec4 maps_b;                // occlusion, emissive, detail albedo, detail normal
    uvec4 maps_c;                // detail mask, flags
};

layout(std430, set = 0, binding = 7) readonly buffer MaterialBlock
{
    GpuMaterial materials[];
} material_block;

layout(set = 0, binding = 1) uniform samplerCube irradiance_cube;
layout(set = 0, binding = 2) uniform samplerCube specular_cube;
layout(set = 0, binding = 3) uniform sampler2D brdf_lut;
// The sun's cascades, as a two-by-two atlas sampled through a comparison sampler, and
// the screen-space contact term that recovers what the cascades are too coarse to see.
layout(set = 0, binding = 11) uniform sampler2DShadow shadow_atlas;
layout(set = 0, binding = 12) uniform sampler2D shadow_atlas_depth;
layout(set = 0, binding = 4) uniform sampler2D ray_shadow_texture;
layout(set = 0, binding = 5) uniform sampler2D contact_shadow_texture;
layout(set = 0, binding = 6) uniform sampler2D cloud_weather_texture;

// The environment's diffuse ambient as 9 spherical-harmonic coefficients, projected from
// the same captured cube the specular chain comes from: nine storage reads and a degree-two
// polynomial in the normal replace a filtered cubemap fetch, and probe blending later
// becomes a blend of coefficients. The band factors and 1/pi were baked in at projection,
// so this returns the same magnitude the irradiance cube used to.
layout(std430, set = 0, binding = 13) readonly buffer IrradianceSh
{
    vec4 coeff[9];
} irradiance_sh;

vec3 evaluate_sh(vec3 n)
{
    float y[9];
    y[0] = 0.282095;
    y[1] = 0.488603 * n.y;
    y[2] = 0.488603 * n.z;
    y[3] = 0.488603 * n.x;
    y[4] = 1.092548 * n.x * n.y;
    y[5] = 1.092548 * n.y * n.z;
    y[6] = 0.315392 * (3.0 * n.z * n.z - 1.0);
    y[7] = 1.092548 * n.x * n.z;
    y[8] = 0.546274 * (n.x * n.x - n.y * n.y);
    vec3 result = vec3(0.0);
    for (int i = 0; i < 9; ++i)
        result += irradiance_sh.coeff[i].rgb * y[i];
    return max(result, vec3(0.0)); // clamp the small negative lobe SH can ring into
}

layout(set = 1, binding = 0) uniform sampler2D bindless_textures[];

layout(push_constant) uniform Push
{
    mat4 model;
    vec4 albedo_metallic;
    vec4 emissive_roughness;
    vec4 outline_shift;
    uint entity_id;
    uint selected;
    uint material_index;
    uint motion_index;
} pc;

layout(location = 0) in vec3 v_world_position;
layout(location = 1) in vec3 v_world_normal;
layout(location = 2) in vec4 v_world_tangent;
layout(location = 3) in vec2 v_uv0;
layout(location = 4) in vec2 v_uv1;
layout(location = 5) in vec4 v_color;
layout(location = 6) in vec4 v_current_clip;
layout(location = 7) in vec4 v_previous_clip;

layout(location = 0) out vec4 out_color;
layout(location = 1) out uint out_id;
layout(location = 2) out vec2 out_velocity;

#define MATERIAL_HAS_PARALLAX      (1u << 0)
#define MATERIAL_PACKED_OCCLUSION  (1u << 1)
#define MATERIAL_EMISSIVE          (1u << 2)
#define MATERIAL_PARALLAX_SHADOWS  (1u << 3)
#define MATERIAL_PARALLAX_CLIP     (1u << 4)
#define MATERIAL_HAS_DETAIL        (1u << 5)
#define MATERIAL_CUTOUT            (1u << 6)
#define MATERIAL_ANISOTROPY        (1u << 7)
#define MATERIAL_CLEARCOAT         (1u << 8)
#define MATERIAL_SHEEN             (1u << 9)
#define MATERIAL_TRANSMISSION      (1u << 10)

vec4 sample_map(uint index, vec2 uv)
{
    return texture(bindless_textures[nonuniformEXT(index)], uv);
}

vec4 sample_map_grad(uint index, vec2 uv, vec2 ddx, vec2 ddy)
{
    return textureGrad(bindless_textures[nonuniformEXT(index)], uv, ddx, ddy);
}

vec2 apply_transform(vec2 uv, vec4 st)
{
    return uv * st.xy + st.zw;
}

// Parallax occlusion mapping: march the view ray through the height field in tangent
// space until it passes below the surface, then refine by intersecting the last two
// samples. Steps scale with view angle — a grazing ray crosses more of the field and
// needs more of them, a head-on ray barely needs any.
vec2 parallax_occlusion(uint height_map, vec2 uv, vec3 view_tangent, float scale,
                        float max_steps, out float height_out)
{
    float steps = mix(max_steps, max(4.0, max_steps * 0.25),
                      clamp(abs(view_tangent.z), 0.0, 1.0));
    float layer_depth = 1.0 / steps;
    vec2 delta = view_tangent.xy / max(abs(view_tangent.z), 1e-3) * scale / steps;

    vec2 ddx = dFdx(uv);
    vec2 ddy = dFdy(uv);
    vec2 current_uv = uv;
    float current_layer = 0.0;
    float current_height = 1.0 - sample_map_grad(height_map, current_uv, ddx, ddy).r;

    for (int i = 0; i < 64; ++i)
    {
        if (current_layer >= current_height || float(i) >= steps)
            break;
        current_uv -= delta;
        current_height = 1.0 - sample_map_grad(height_map, current_uv, ddx, ddy).r;
        current_layer += layer_depth;
    }

    vec2 previous_uv = current_uv + delta;
    float after = current_height - current_layer;
    float before = (1.0 - sample_map_grad(height_map, previous_uv, ddx, ddy).r) -
                   current_layer + layer_depth;
    float weight = after / max(after - before, 1e-4);
    height_out = current_layer;
    return mix(current_uv, previous_uv, weight);
}

// Marches the height field toward the light and reports how much of the way it is
// blocked — the self-shadowing that makes a parallax surface read as displaced
// rather than as a moving texture.
float parallax_shadow(uint height_map, vec2 uv, vec3 light_tangent, float scale,
                      float surface_height)
{
    if (light_tangent.z <= 0.0)
        return 1.0;
    const float steps = 8.0;
    float layer_depth = surface_height / steps;
    vec2 delta = light_tangent.xy / max(light_tangent.z, 1e-3) * scale / steps;

    vec2 ddx = dFdx(uv);
    vec2 ddy = dFdy(uv);
    float shadow = 0.0;
    vec2 current_uv = uv;
    float current_layer = surface_height;
    for (int i = 0; i < 8; ++i)
    {
        current_uv += delta;
        current_layer -= layer_depth;
        float height = 1.0 - sample_map_grad(height_map, current_uv, ddx, ddy).r;
        if (height < current_layer)
            shadow = max(shadow, (current_layer - height) * (1.0 - float(i) / steps));
    }
    return 1.0 - clamp(shadow * 4.0, 0.0, 1.0);
}

vec3 decode_normal(uint map, vec2 uv, float scale)
{
    vec3 encoded = sample_map(map, uv).xyz * 2.0 - 1.0;
    encoded.xy *= scale;
    // Reconstruct Z rather than trusting the stored blue channel: it survives BC5 and
    // any two-channel compression unchanged.
    encoded.z = sqrt(max(1.0 - dot(encoded.xy, encoded.xy), 0.0));
    return encoded;
}

void main()
{
    GpuMaterial material = material_block.materials[pc.material_index];
    uint flags = material.maps_c.y;

    // Meshes shade in camera-relative space: v_world_position is the fragment's offset
    // from the camera, so the camera sits at the origin of this frame and the view
    // direction is just -position.
    vec3 geometric_normal = normalize(v_world_normal);
    vec3 view_dir = normalize(-v_world_position);
    // Double-sided: flip toward the viewer so back faces (cloth, foliage) still light.
    if (dot(geometric_normal, view_dir) < 0.0)
        geometric_normal = -geometric_normal;

    // A mesh with no authored tangent gets one from the UV derivatives, so an imported
    // asset without tangents still normal-maps correctly.
    vec3 tangent = v_world_tangent.xyz;
    float handedness = v_world_tangent.w;
    if (dot(tangent, tangent) < 1e-8)
    {
        vec3 dpdx = dFdx(v_world_position);
        vec3 dpdy = dFdy(v_world_position);
        vec2 duvdx = dFdx(v_uv0);
        vec2 duvdy = dFdy(v_uv0);
        float determinant = duvdx.x * duvdy.y - duvdy.x * duvdx.y;
        tangent = abs(determinant) > 1e-12
                      ? (dpdx * duvdy.y - dpdy * duvdx.y) / determinant
                      : vec3(1.0, 0.0, 0.0);
        handedness = 1.0;
    }
    tangent = normalize(tangent - geometric_normal * dot(geometric_normal, tangent));
    vec3 bitangent = cross(geometric_normal, tangent) * (handedness < 0.0 ? -1.0 : 1.0);
    mat3 tangent_to_world = mat3(tangent, bitangent, geometric_normal);

    vec2 uv = apply_transform(v_uv0, material.main_transform);
    float surface_height = 0.0;
    if ((flags & MATERIAL_HAS_PARALLAX) != 0u)
    {
        vec3 view_tangent = normalize(transpose(tangent_to_world) * view_dir);
        uv = parallax_occlusion(material.maps_a.w, uv, view_tangent, material.parallax.x,
                                material.parallax.y, surface_height);
        if ((flags & MATERIAL_PARALLAX_CLIP) != 0u &&
            (uv.x < 0.0 || uv.y < 0.0 || uv.x > 1.0 || uv.y > 1.0))
            discard;
    }

    vec4 base = sample_map(material.maps_a.x, uv) * material.base_color * v_color;
    if ((flags & MATERIAL_CUTOUT) != 0u && base.a < material.surface.w)
        discard;

    vec3 albedo = base.rgb;
    vec4 packed = sample_map(material.maps_a.y, uv);
    float metallic = packed.b * material.surface.x;
    float roughness = clamp(packed.g * material.surface.y, 0.045, 1.0);
    float occlusion = (flags & MATERIAL_PACKED_OCCLUSION) != 0u
                          ? packed.r
                          : sample_map(material.maps_b.x, uv).r;
    occlusion = mix(1.0, occlusion, material.surface.z);

    vec3 tangent_normal = decode_normal(material.maps_a.z, uv, material.emissive.w);

    if ((flags & MATERIAL_HAS_DETAIL) != 0u)
    {
        vec2 detail_uv = apply_transform(v_uv0, material.detail_transform);
        float mask = sample_map(material.maps_c.x, uv).r;
        // Unity's overlay convention: the detail albedo is a 0.5-centred modulation,
        // so a flat grey detail map leaves the base untouched.
        vec3 detail_albedo = sample_map(material.maps_b.z, detail_uv).rgb;
        albedo *= mix(vec3(1.0), detail_albedo * 2.0, mask);

        vec3 detail_normal =
            decode_normal(material.maps_b.w, detail_uv, material.parallax.w * mask);
        // Whiteout blend: add the tangent-space XY and keep the base Z, which composes
        // two normal maps without the flattening a plain average causes.
        tangent_normal = normalize(vec3(tangent_normal.xy + detail_normal.xy,
                                        tangent_normal.z * detail_normal.z));
    }

    vec3 n = normalize(tangent_to_world * tangent_normal);
    if (dot(n, view_dir) < 0.0)
        n = normalize(mix(n, geometric_normal, 0.5));

    vec3 light_dir = normalize(scene.sun_dir.xyz);
    vec3 half_vec = normalize(view_dir + light_dir);
    float n_dot_v = max(dot(n, view_dir), 1e-4);
    float n_dot_l = max(dot(n, light_dir), 0.0);
    float n_dot_h = max(dot(n, half_vec), 0.0);
    float v_dot_h = max(dot(view_dir, half_vec), 0.0);

    // Dielectric F0 from the index of refraction, so glass and skin are not forced to
    // the 0.04 default every plastic uses.
    float dielectric_f0 = pow((material.parallax.z - 1.0) / (material.parallax.z + 1.0), 2.0);
    vec3 f0 = mix(vec3(dielectric_f0), albedo, metallic);
    vec2 dfg = texture(brdf_lut, vec2(n_dot_v, roughness)).rg;
    vec3 compensation = energy_compensation(f0, dfg);

    float visibility = 1.0;
    if ((flags & MATERIAL_PARALLAX_SHADOWS) != 0u && n_dot_l > 0.0)
    {
        vec3 light_tangent = normalize(transpose(tangent_to_world) * light_dir);
        visibility = parallax_shadow(material.maps_a.w, uv, light_tangent,
                                     material.parallax.x, surface_height);
    }

    // The sun's own visibility, at three scales. The cascades carry the body of the
    // shadow; the screen-space march recovers the contact they are too coarse to
    // resolve; the geometric normal, not the mapped one, offsets the cascade lookup,
    // because a normal map perturbs shading and not the surface a shadow texel lands on.
    if (n_dot_l > 0.0)
    {
        vec2 screen_uv = gl_FragCoord.xy / max(temporal.resolution.xy, vec2(1.0));
        if (shadows.flags.z > 0.5)
        {
            // Traced: exact, so it replaces the cascades rather than joining them, and
            // it resolves contact on its own — the screen-space march would only add its
            // own approximation on top of an answer that has none.
            visibility *= texture(ray_shadow_texture, screen_uv).r;
        }
        else
        {
            float view_depth = dot(scene.cam_forward.xyz, v_world_position);
            visibility *= sample_sun_shadow(shadow_atlas, shadow_atlas_depth,
                                            v_world_position, normalize(v_world_normal),
                                            light_dir, view_depth);
            if (shadows.flags.y > 0.5)
                visibility *= texture(contact_shadow_texture, screen_uv).r;
        }
        // The deck overhead shades this surface exactly as it shades the analytic
        // ground, so a mesh standing on that ground darkens with it rather than staying
        // lit inside a cloud shadow.
        visibility *= cloud_sun_transmittance(cloud_weather_texture, v_world_position,
                                              light_dir);
    }

    vec3 f = f_schlick(v_dot_h, f0);
    float distribution;
    if ((flags & MATERIAL_ANISOTROPY) != 0u)
    {
        float angle = material.anisotropy_clearcoat.y * 6.28318530718;
        vec3 anisotropic_tangent = normalize(tangent * cos(angle) + bitangent * sin(angle));
        vec3 anisotropic_bitangent = cross(n, anisotropic_tangent);
        float aspect = sqrt(1.0 - abs(material.anisotropy_clearcoat.x) * 0.9);
        float at = max(roughness * roughness / aspect, 1e-4);
        float ab = max(roughness * roughness * aspect, 1e-4);
        distribution = d_ggx_anisotropic(n_dot_h, dot(anisotropic_tangent, half_vec),
                                         dot(anisotropic_bitangent, half_vec), at, ab);
    }
    else
    {
        distribution = d_ggx(n_dot_h, roughness);
    }

    vec3 specular = distribution * v_smith_ggx_correlated(n_dot_v, n_dot_l, roughness) * f *
                    compensation;
    vec3 diffuse = (vec3(1.0) - f) * (1.0 - metallic) * diffuse_lambert(albedo);

    if ((flags & MATERIAL_SHEEN) != 0u)
    {
        float sheen_distribution = d_charlie(n_dot_h, material.sheen.a);
        specular += material.sheen.rgb * sheen_distribution *
                    v_ashikhmin(n_dot_v, n_dot_l);
    }

    float clearcoat_attenuation = 1.0;
    vec3 clearcoat_specular = vec3(0.0);
    if ((flags & MATERIAL_CLEARCOAT) != 0u)
    {
        float clearcoat = material.anisotropy_clearcoat.z;
        float clearcoat_roughness = material.anisotropy_clearcoat.w;
        // The coat sits on top of the base layer, so what it reflects it also removes
        // from what reaches the base — that is the attenuation, not an extra term.
        float clearcoat_fresnel = f_schlick(v_dot_h, vec3(0.04)).x * clearcoat;
        clearcoat_specular = vec3(d_ggx(n_dot_h, clearcoat_roughness) * v_kelemen(v_dot_h) *
                                  clearcoat_fresnel);
        clearcoat_attenuation = 1.0 - clearcoat_fresnel;
    }

    vec3 radiance = scene.sun_color.xyz * scene.sun_dir.w;
    vec3 direct = ((diffuse + specular) * clearcoat_attenuation + clearcoat_specular) *
                  radiance * n_dot_l * visibility;

    // Image-based lighting: the prefiltered environment replaces the flat ambient
    // constant, which is what makes a metal read as metal at all.
    vec3 indirect;
    if (scene.ibl_params.z > 0.5)
    {
        vec3 reflection = reflect(-view_dir, n);
        float mip = roughness * max(scene.ibl_params.y - 1.0, 0.0);
        vec3 prefiltered = textureLod(specular_cube, reflection, mip).rgb;
        vec3 irradiance = evaluate_sh(n);
        vec3 fresnel_ibl = f_schlick_roughness(n_dot_v, f0, roughness);
        float specular_ao = specular_occlusion(n_dot_v, occlusion, roughness);

        vec3 indirect_diffuse = irradiance * albedo * (1.0 - metallic) *
                                (vec3(1.0) - fresnel_ibl) * occlusion;
        vec3 indirect_specular = prefiltered * (fresnel_ibl * dfg.x + dfg.y) * compensation *
                                 specular_ao;
        indirect = (indirect_diffuse + indirect_specular) * scene.ibl_params.x;
    }
    else
    {
        indirect = scene.ambient.xyz * albedo * occlusion;
    }

    vec3 emissive = vec3(0.0);
    if ((flags & MATERIAL_EMISSIVE) != 0u)
        emissive = sample_map(material.maps_b.y, uv).rgb * material.emissive.rgb;

    if ((flags & MATERIAL_TRANSMISSION) != 0u)
    {
        // Thin-surface transmission: light arriving from behind wraps through the
        // surface, tinted by the subsurface colour and attenuated by thickness.
        float wrap = max(dot(-n, light_dir), 0.0);
        float attenuation = exp(-material.transmission.y * 4.0);
        indirect += material.subsurface.rgb * albedo * radiance * wrap *
                    material.transmission.x * attenuation;
    }

    out_color = vec4(direct + indirect + emissive, base.a);
    out_id = pc.entity_id;
    // Measured from the geometric position, not the parallax-displaced one: the
    // displacement is a shading trick with no surface behind it to reproject.
    out_velocity = motion_vector(v_current_clip, v_previous_clip);
}
