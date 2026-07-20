// The renderer's BRDF. A Cook-Torrance metallic-roughness base — GGX distribution,
// height-correlated Smith visibility, Schlick Fresnel — extended past single
// scattering: rough metals lose a large fraction of their energy to the microfacet
// masking term, and the Kulla-Conty compensation below puts it back using the same
// split-sum BRDF LUT the image-based specular already needs. The optional lobes
// (anisotropy, clearcoat, sheen) follow the glTF KHR_materials_* definitions so an
// imported asset shades the way its author saw it.

const float PBR_PI = 3.14159265359;

float d_ggx(float n_dot_h, float roughness)
{
    float a = roughness * roughness;
    float a2 = a * a;
    float d = n_dot_h * n_dot_h * (a2 - 1.0) + 1.0;
    return a2 / max(PBR_PI * d * d, 1e-7);
}

// Anisotropic GGX (Burley): the roughness is split along the tangent and bitangent,
// which is what stretches a brushed-metal highlight along the grain.
float d_ggx_anisotropic(float n_dot_h, float t_dot_h, float b_dot_h, float at, float ab)
{
    float d = t_dot_h * t_dot_h / (at * at) + b_dot_h * b_dot_h / (ab * ab) +
              n_dot_h * n_dot_h;
    return 1.0 / max(PBR_PI * at * ab * d * d, 1e-7);
}

// Height-correlated Smith visibility, already divided by the 4 n.v n.l denominator.
float v_smith_ggx_correlated(float n_dot_v, float n_dot_l, float roughness)
{
    float a2 = roughness * roughness * roughness * roughness;
    float v = n_dot_l * sqrt(n_dot_v * n_dot_v * (1.0 - a2) + a2);
    float l = n_dot_v * sqrt(n_dot_l * n_dot_l * (1.0 - a2) + a2);
    return 0.5 / max(v + l, 1e-7);
}

// Kelemen visibility: the cheap term the clearcoat layer uses, where the second
// specular lobe is always smooth enough that the full Smith term is not worth it.
float v_kelemen(float v_dot_h)
{
    return 0.25 / max(v_dot_h * v_dot_h, 1e-7);
}

// Charlie distribution + Ashikhmin visibility: the retroreflective rim that makes
// cloth read as cloth rather than as a rough dielectric.
float d_charlie(float n_dot_h, float roughness)
{
    float inverse = 1.0 / max(roughness, 1e-3);
    float sin2 = 1.0 - n_dot_h * n_dot_h;
    return (2.0 + inverse) * pow(sin2, inverse * 0.5) / (2.0 * PBR_PI);
}

float v_ashikhmin(float n_dot_v, float n_dot_l)
{
    return 1.0 / max(4.0 * (n_dot_l + n_dot_v - n_dot_l * n_dot_v), 1e-7);
}

vec3 f_schlick(float cos_theta, vec3 f0)
{
    float f = pow(clamp(1.0 - cos_theta, 0.0, 1.0), 5.0);
    return f0 + (1.0 - f0) * f;
}

// Fresnel that accounts for roughness: a rough surface never reaches the full
// grazing-angle reflectance a smooth one does, and using the smooth form for the
// image-based term is what makes rough materials look like they have a halo.
vec3 f_schlick_roughness(float cos_theta, vec3 f0, float roughness)
{
    float f = pow(clamp(1.0 - cos_theta, 0.0, 1.0), 5.0);
    return f0 + (max(vec3(1.0 - roughness), f0) - f0) * f;
}

// Kulla-Conty multi-scatter compensation. `dfg` is the split-sum BRDF LUT's (scale,
// bias) pair for this view angle and roughness; their sum is the fraction of energy
// the single-scattering lobe accounts for, so the reciprocal of what is left is the
// gain that restores the rest.
vec3 energy_compensation(vec3 f0, vec2 dfg)
{
    float single = dfg.x + dfg.y;
    return 1.0 + f0 * (1.0 / max(single, 1e-3) - 1.0);
}

// Specular occlusion from ambient occlusion and view angle (Lagarde's form): AO is
// a diffuse quantity, and applying it unmodified to the specular term over-darkens
// grazing reflections.
float specular_occlusion(float n_dot_v, float ao, float roughness)
{
    return clamp(pow(n_dot_v + ao, exp2(-16.0 * roughness - 1.0)) - 1.0 + ao, 0.0, 1.0);
}

vec3 diffuse_lambert(vec3 albedo)
{
    return albedo / PBR_PI;
}
