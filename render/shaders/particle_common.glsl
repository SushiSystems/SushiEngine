// Shared particle definitions for the VFX compute and draw shaders (design §5).
// Included by particle_emit.comp, particle_simulate.comp, and particle.vert; the struct
// layouts mirror Vfx::GpuParticle (80 bytes) and Scene::GpuEmitter (224 bytes) field for
// field, read with scalar std430 members so the byte layout matches the host upload exactly.

// One live particle — mirrors Vfx::GpuParticle (five vec4s, 80 bytes).
struct Particle
{
    float px, py, pz;
    float life;
    float vx, vy, vz;
    float age;
    float cr, cg, cb;
    float alpha;
    float size;
    float rotation;
    float lifetime;
    float angular_velocity;
    uint  seed;
    uint  emitter_index;
    uint  flipbook_frame;
    float birth_size;
};

// One active emitter — mirrors Scene::GpuEmitter (mat4 + ten vec4s, 224 bytes).
struct Emitter
{
    mat4  model;
    uint  shape;
    uint  shape_flags;
    uint  update_flags;
    uint  capacity;
    float shape_radius;
    float shape_cone_angle;
    float shape_arc;
    float drag_coefficient;
    float box0, box1, box2;
    float turbulence_frequency;
    float gx, gy, gz;
    float turbulence_amplitude;
    float cr, cg, cb;
    float pad_color;
    float lifetime_min, lifetime_max, speed_min, speed_max;
    float size_min, size_max, rotation_min, rotation_max;
    float angular_min, angular_max, pad_a, pad_b;
    int   size_curve_lut;
    int   color_gradient_lut;
    uint  spawn_base;
    uint  spawn_count;
    uint  seed;
    uint  frame;
    uint  flipbook_rows;
    uint  flipbook_columns;
    uint  blend;
    uint  sort;
    uint  pad0;
    uint  pad1;
};

// Vfx::BlendMode values.
const uint BLEND_ADDITIVE = 0u;
const uint BLEND_ALPHA = 1u;
const uint BLEND_PREMULTIPLIED = 2u;

// Vfx::UpdateFlags / ShapeFlags bits.
const uint UPDATE_GRAVITY = 1u;
const uint UPDATE_DRAG = 2u;
const uint UPDATE_TURBULENCE = 4u;
const uint UPDATE_SIZE_OVER_LIFE = 8u;
const uint UPDATE_COLOR_OVER_LIFE = 16u;
const uint SHAPE_EMIT_FROM_SHELL = 1u;

// Vfx::EmitterShape values.
const uint SHAPE_POINT = 0u;
const uint SHAPE_SPHERE = 1u;
const uint SHAPE_HEMISPHERE = 2u;
const uint SHAPE_CONE = 3u;
const uint SHAPE_BOX = 4u;
const uint SHAPE_CIRCLE = 5u;

const uint CURVE_LUT_WIDTH = 64u;
const uint GRADIENT_LUT_WIDTH = 64u;
const float TWO_PI = 6.2831853;

// PCG hash for a stateless-per-thread RNG.
uint pcg_hash(uint value)
{
    uint state = value * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

float rand(inout uint seed)
{
    seed = pcg_hash(seed);
    return float(seed) * (1.0 / 4294967296.0);
}

float rand_range(inout uint seed, float lo, float hi)
{
    return lo + (hi - lo) * rand(seed);
}

// A uniformly distributed unit vector.
vec3 rand_unit_sphere(inout uint seed)
{
    float z = rand_range(seed, -1.0, 1.0);
    float phi = rand_range(seed, 0.0, TWO_PI);
    float r = sqrt(max(0.0, 1.0 - z * z));
    return vec3(r * cos(phi), z, r * sin(phi));
}

// Samples a birth position and emit direction in the emitter's local frame (up = +Y).
void sample_shape(inout uint seed, Emitter e, out vec3 local_position, out vec3 local_direction)
{
    bool shell = (e.shape_flags & SHAPE_EMIT_FROM_SHELL) != 0u;
    if (e.shape == SHAPE_SPHERE || e.shape == SHAPE_HEMISPHERE)
    {
        vec3 dir = rand_unit_sphere(seed);
        if (e.shape == SHAPE_HEMISPHERE && dir.y < 0.0)
            dir.y = -dir.y;
        float r = shell ? e.shape_radius : e.shape_radius * pow(rand(seed), 1.0 / 3.0);
        local_position = dir * r;
        local_direction = dir;
    }
    else if (e.shape == SHAPE_CONE)
    {
        float base = sqrt(rand(seed)) * e.shape_radius;
        float phi = rand_range(seed, 0.0, e.shape_arc);
        local_position = vec3(base * cos(phi), 0.0, base * sin(phi));
        float cos_max = cos(e.shape_cone_angle);
        float cos_theta = rand_range(seed, cos_max, 1.0);
        float sin_theta = sqrt(max(0.0, 1.0 - cos_theta * cos_theta));
        float dphi = rand_range(seed, 0.0, TWO_PI);
        local_direction = vec3(sin_theta * cos(dphi), cos_theta, sin_theta * sin(dphi));
    }
    else if (e.shape == SHAPE_BOX)
    {
        local_position = vec3(rand_range(seed, -e.box0, e.box0), rand_range(seed, -e.box1, e.box1),
                              rand_range(seed, -e.box2, e.box2));
        local_direction = vec3(0.0, 1.0, 0.0);
    }
    else if (e.shape == SHAPE_CIRCLE)
    {
        float phi = rand_range(seed, 0.0, e.shape_arc);
        float r = shell ? e.shape_radius : e.shape_radius * sqrt(rand(seed));
        local_position = vec3(r * cos(phi), 0.0, r * sin(phi));
        local_direction = vec3(cos(phi), 0.0, sin(phi));
    }
    else // SHAPE_POINT
    {
        local_position = vec3(0.0);
        local_direction = vec3(0.0, 1.0, 0.0);
    }
}

// Trilinear value noise for curl turbulence.
float value_hash(vec3 p)
{
    p = fract(p * 0.3183099 + vec3(0.1, 0.2, 0.3));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

float value_noise(vec3 p)
{
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float n000 = value_hash(i + vec3(0, 0, 0));
    float n100 = value_hash(i + vec3(1, 0, 0));
    float n010 = value_hash(i + vec3(0, 1, 0));
    float n110 = value_hash(i + vec3(1, 1, 0));
    float n001 = value_hash(i + vec3(0, 0, 1));
    float n101 = value_hash(i + vec3(1, 0, 1));
    float n011 = value_hash(i + vec3(0, 1, 1));
    float n111 = value_hash(i + vec3(1, 1, 1));
    float x00 = mix(n000, n100, f.x);
    float x10 = mix(n010, n110, f.x);
    float x01 = mix(n001, n101, f.x);
    float x11 = mix(n011, n111, f.x);
    return mix(mix(x00, x10, f.y), mix(x01, x11, f.y), f.z);
}

// Divergence-free curl of a noise potential.
vec3 curl_noise(vec3 p)
{
    float e = 0.1;
    vec3 dx = vec3(e, 0.0, 0.0);
    vec3 dy = vec3(0.0, e, 0.0);
    vec3 dz = vec3(0.0, 0.0, e);
    float p1_y1 = value_noise(p + dy + vec3(0, 0, 41));
    float p1_y0 = value_noise(p - dy + vec3(0, 0, 41));
    float p1_z1 = value_noise(p + dz + vec3(0, 17, 0));
    float p1_z0 = value_noise(p - dz + vec3(0, 17, 0));
    float p2_z1 = value_noise(p + dz + vec3(23, 0, 0));
    float p2_z0 = value_noise(p - dz + vec3(23, 0, 0));
    float p2_x1 = value_noise(p + dx + vec3(0, 7, 0));
    float p2_x0 = value_noise(p - dx + vec3(0, 7, 0));
    float p3_x1 = value_noise(p + dx + vec3(0, 0, 3));
    float p3_x0 = value_noise(p - dx + vec3(0, 0, 3));
    float p3_y1 = value_noise(p + dy + vec3(0, 0, 29));
    float p3_y0 = value_noise(p - dy + vec3(0, 0, 29));
    float inv = 1.0 / (2.0 * e);
    return vec3(((p3_y1 - p3_y0) - (p2_z1 - p2_z0)) * inv, ((p1_z1 - p1_z0) - (p3_x1 - p3_x0)) * inv,
                ((p2_x1 - p2_x0) - (p1_y1 - p1_y0)) * inv);
}
