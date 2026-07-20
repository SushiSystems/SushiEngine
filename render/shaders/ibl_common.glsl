// Shared sampling maths for the image-based lighting precompute: the low-discrepancy
// sequence the integrals are estimated with, GGX importance sampling, and the mapping
// from a cubemap face texel to a world direction.

const float IBL_PI = 3.14159265359;

// Van der Corput radical inverse: the second dimension of the Hammersley sequence,
// which distributes samples far more evenly than a random one at these sample counts.
float radical_inverse(uint bits)
{
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10;
}

vec2 hammersley(uint i, uint count)
{
    return vec2(float(i) / float(count), radical_inverse(i));
}

// Samples a half-vector from the GGX distribution around `n`, so the estimator
// concentrates its samples where the lobe actually has energy.
vec3 importance_sample_ggx(vec2 xi, vec3 n, float roughness)
{
    float a = roughness * roughness;
    float phi = 2.0 * IBL_PI * xi.x;
    float cos_theta = sqrt((1.0 - xi.y) / (1.0 + (a * a - 1.0) * xi.y));
    float sin_theta = sqrt(1.0 - cos_theta * cos_theta);

    vec3 h = vec3(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);
    vec3 up = abs(n.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
    vec3 tangent = normalize(cross(up, n));
    vec3 bitangent = cross(n, tangent);
    return normalize(tangent * h.x + bitangent * h.y + n * h.z);
}

// The world direction a cubemap face texel looks along, in Vulkan's face convention:
// u runs left to right and v runs top to bottom of each face image.
vec3 cube_direction(uint face, vec2 uv)
{
    vec2 c = uv * 2.0 - 1.0;
    if (face == 0u)
        return normalize(vec3(1.0, -c.y, -c.x));
    if (face == 1u)
        return normalize(vec3(-1.0, -c.y, c.x));
    if (face == 2u)
        return normalize(vec3(c.x, 1.0, c.y));
    if (face == 3u)
        return normalize(vec3(c.x, -1.0, -c.y));
    if (face == 4u)
        return normalize(vec3(c.x, -c.y, 1.0));
    return normalize(vec3(-c.x, -c.y, -1.0));
}

// Smith visibility with the IBL k, which differs from the direct-lighting one because
// the view and light directions are decorrelated by the environment integral.
float g_smith_ibl(float n_dot_v, float n_dot_l, float roughness)
{
    float k = roughness * roughness / 2.0;
    float gv = n_dot_v / (n_dot_v * (1.0 - k) + k);
    float gl = n_dot_l / (n_dot_l * (1.0 - k) + k);
    return gv * gl;
}
