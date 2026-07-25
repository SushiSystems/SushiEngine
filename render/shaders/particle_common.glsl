#ifndef PARTICLE_COMMON_GLSL
#define PARTICLE_COMMON_GLSL

// Shared particle definitions for the VFX compute and draw shaders (design §5).
// Included by every particle compute and draw shader; the struct layouts mirror
// Vfx::GpuParticle and Scene::GpuEmitter field for field, read with scalar std430 members so the
// byte layout matches the host upload exactly.

// Force fields one emitter can carry; mirrors Vfx::MAX_FORCE_FIELDS. Declared before the Emitter
// struct because it sizes a member of it.
#define MAX_FORCE_FIELDS 4u

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

// One active emitter — mirrors Scene::GpuEmitter (mat4 + thirteen vec4s + the force-field table).
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
    float angular_min, angular_max, velocity_stretch, pad_b;
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
    uint  alignment;
    uint  mesh_slot;
    uint  force_field_count;
    float collision_restitution;
    float collision_friction;
    float collision_thickness;
    uint  render_flags;
    uint  texture;
    float soft_fade_distance;
    float pad_material;
    // Three vec4s per field: (centre.xyz, strength), (axis.xyz, radius),
    // (kind, falloff, pad, pad). Already in world space — the host baked the emitter transform in.
    vec4  force_fields[MAX_FORCE_FIELDS * 3];
};

// Vfx::ForceFieldKind values.
const uint FIELD_POINT = 0u;
const uint FIELD_VORTEX = 1u;
const uint FIELD_DRAG = 2u;

// Vfx::BlendMode values.
const uint BLEND_ADDITIVE = 0u;
const uint BLEND_ALPHA = 1u;
const uint BLEND_PREMULTIPLIED = 2u;

// Vfx::RenderFlags bits — the particle material, per emitter.
const uint RENDER_SOFT = 1u;
const uint RENDER_LIT = 2u;
const uint RENDER_TEXTURED = 4u;

// Vfx::RenderAlignment values.
const uint ALIGN_FACE_CAMERA = 0u;
const uint ALIGN_VELOCITY_STRETCHED = 1u;
const uint ALIGN_RIBBON = 2u;
const uint ALIGN_MESH = 3u;

// Distinct meshes drawable as mesh particles in one frame; mirrors
// Scene::ParticleSystem::MAX_MESH_EMITTERS. Each owns an equal slice of the mesh draw list and one
// VkDrawIndexedIndirectCommand (five uints), because one draw can bind only one mesh.
const uint MAX_MESH_EMITTERS = 4u;
const uint NO_MESH_SLOT = 0xFFFFFFFFu;

// Positions kept per particle for the ribbon path, newest first. Mirrors
// Scene::ParticleSystem::TRAIL_POINTS; the strip is TRAIL_POINTS - 1 quads long.
const uint TRAIL_POINTS = 8u;
const uint RIBBON_VERTICES = (TRAIL_POINTS - 1u) * 6u;

// Vfx::UpdateFlags / ShapeFlags bits.
const uint UPDATE_GRAVITY = 1u;
const uint UPDATE_DRAG = 2u;
const uint UPDATE_TURBULENCE = 4u;
const uint UPDATE_SIZE_OVER_LIFE = 8u;
const uint UPDATE_COLOR_OVER_LIFE = 16u;
const uint UPDATE_FORCE_FIELDS = 32u;
const uint UPDATE_COLLISION = 64u;
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

// The emitter's placed force fields, evaluated at @p position.
//
// The mirror of CpuDeterministicBackend::integrate's field loop — the two backends read the same
// authored record, so they have to agree on what it means. A field's weight is 1 at its centre and
// 0 at its rim, raised to the authored falloff: nothing outside the radius is touched, and nothing
// spikes to infinity at the centre the way a raw inverse-square would. Drag fields do not
// accelerate; they return through @p out_damp a factor to scale velocity by after integration.
vec3 particle_force_fields(Emitter e, vec3 position, float dt, out float out_damp)
{
    vec3 acceleration = vec3(0.0);
    out_damp = 1.0;
    for (uint f = 0u; f < e.force_field_count; ++f)
    {
        vec4 centre_strength = e.force_fields[f * 3u];
        vec4 axis_radius = e.force_fields[f * 3u + 1u];
        vec4 kind_falloff = e.force_fields[f * 3u + 2u];

        vec3 to_centre = centre_strength.xyz - position;
        float distance_squared = dot(to_centre, to_centre);
        if (distance_squared > axis_radius.w * axis_radius.w)
            continue;
        float distance = sqrt(distance_squared);
        float weight = pow(1.0 - distance / axis_radius.w, kind_falloff.y);
        uint kind = uint(kind_falloff.x);

        if (kind == FIELD_DRAG)
        {
            out_damp -= centre_strength.w * weight * dt;
            continue;
        }
        if (distance < 1e-4)
            continue; // at the exact centre there is no direction to push along

        if (kind == FIELD_POINT)
        {
            acceleration += to_centre * (centre_strength.w * weight / distance);
        }
        else // FIELD_VORTEX: push along axis x (particle - centre)
        {
            vec3 tangent = cross(axis_radius.xyz, -to_centre);
            float length_squared = dot(tangent, tangent);
            if (length_squared < 1e-12)
                continue; // on the axis itself, where there is nothing to swirl
            acceleration += tangent * (centre_strength.w * weight / sqrt(length_squared));
        }
    }
    return acceleration;
}

// The camera-relative direction through a pixel, scaled so its forward component is exactly one.
// Multiplying it by a linear view depth gives that pixel's camera-relative position. The up term is
// negated because the projection is Y-flipped for Vulkan, so a larger v is further down the screen.
vec3 particle_view_ray(vec2 uv, vec3 camera_right, vec3 camera_up, vec3 camera_forward,
                       float tan_x, float tan_y)
{
    vec2 ndc = uv * 2.0 - 1.0;
    return camera_right * (ndc.x * tan_x) - camera_up * (ndc.y * tan_y) + camera_forward;
}

// Bounces a particle off whatever the depth pyramid says is on screen.
//
// A screen-space test against the depth the renderer already produced last frame, so it needs no
// collision geometry and no broadphase. What it cannot do follows from the same fact: it only knows
// surfaces the camera can see, so particles off screen, behind the viewer, or hidden behind
// something nearer pass straight through. That is the right trade for sparks skittering off a floor
// in view, and the reason the deterministic backend has no counterpart to this.
//
// @return true when the particle was in contact and @p position / @p velocity were changed.
bool particle_depth_collide(sampler2D depth_pyramid, Emitter e, mat4 view_projection,
                            vec3 camera_right, vec3 camera_up, float tan_x, float tan_y,
                            inout vec3 position, inout vec3 velocity)
{
    vec4 clip = view_projection * vec4(position, 1.0);
    if (clip.w <= 1e-4)
        return false; // behind the eye
    vec2 uv = clip.xy / clip.w * 0.5 + 0.5;
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))))
        return false; // off screen

    float scene_depth = textureLod(depth_pyramid, uv, 0.0).r;
    float penetration = clip.w - scene_depth;
    // Above the surface, or so far behind it that the particle is past the object rather than
    // resting on it — the depth buffer records a surface, not a solid.
    if (penetration <= 0.0 || penetration > e.collision_thickness)
        return false;

    vec3 camera_forward = normalize(cross(camera_right, camera_up));
    vec2 texel = 1.0 / vec2(textureSize(depth_pyramid, 0));
    float depth_x = textureLod(depth_pyramid, uv + vec2(texel.x, 0.0), 0.0).r;
    float depth_y = textureLod(depth_pyramid, uv + vec2(0.0, texel.y), 0.0).r;

    vec3 normal;
    float edge = e.collision_thickness * 4.0;
    if (abs(depth_x - scene_depth) > edge || abs(depth_y - scene_depth) > edge)
    {
        // A depth discontinuity: the neighbours are on a different surface, so the gradient would
        // give a normal pointing nowhere real. Facing the camera is the safe answer at a silhouette.
        normal = -camera_forward;
    }
    else
    {
        vec3 here = particle_view_ray(uv, camera_right, camera_up, camera_forward, tan_x, tan_y) *
                    scene_depth;
        vec3 right_sample = particle_view_ray(uv + vec2(texel.x, 0.0), camera_right, camera_up,
                                              camera_forward, tan_x, tan_y) * depth_x;
        vec3 down_sample = particle_view_ray(uv + vec2(0.0, texel.y), camera_right, camera_up,
                                             camera_forward, tan_x, tan_y) * depth_y;
        vec3 gradient = cross(right_sample - here, down_sample - here);
        if (dot(gradient, gradient) < 1e-12)
            return false; // degenerate footprint, nothing trustworthy to bounce off
        normal = normalize(gradient);
        if (dot(normal, camera_forward) > 0.0)
            normal = -normal; // a visible surface faces the eye
    }

    float into = dot(velocity, normal);
    if (into < 0.0)
    {
        vec3 normal_velocity = normal * into;
        vec3 tangent_velocity = velocity - normal_velocity;
        velocity = tangent_velocity * (1.0 - e.collision_friction) -
                   normal_velocity * e.collision_restitution;
    }
    // Lift the particle back out, so the next step starts in contact rather than deeper in.
    position += normal * penetration;
    return true;
}

// Where a quad corner sits relative to its particle's centre, in world space.
//
// The one place the billboard is expanded, shared by every draw path (the additive list, the
// depth-sorted alpha list, and the host-uploaded deterministic billboards) so an alignment mode
// is implemented once. @p corner is the unit quad corner in [-1,1]^2 and @p size is the
// particle's half-extent, matching the pool's convention.
//
// FaceCamera spins the corner by the particle's roll and lays it on the camera plane.
// VelocityStretched instead aims the quad's long axis down the particle's screen-projected
// velocity and lengthens it by the speed — the streak a spark reads as. The roll is dropped
// there: the velocity already fixes the orientation, and spinning a streak would only wobble it.
// Two degenerate cases fall back to camera-facing, which is what a zero-length streak *is*: a
// particle that is barely moving, and one flying straight at the eye (whose screen-projected
// velocity is null and whose stretch direction would be undefined).
vec3 particle_quad_offset(Particle p, vec2 corner, vec3 camera_right, vec3 camera_up,
                          uint alignment, float velocity_stretch)
{
    if (alignment == ALIGN_VELOCITY_STRETCHED)
    {
        vec3 velocity = vec3(p.vx, p.vy, p.vz);
        float speed = length(velocity);
        if (speed > 1e-4)
        {
            vec3 forward = normalize(cross(camera_right, camera_up));
            vec3 along = velocity - forward * dot(velocity, forward);
            if (dot(along, along) > 1e-8)
            {
                vec3 along_dir = normalize(along);
                vec3 across_dir = normalize(cross(along_dir, forward));
                return along_dir * (corner.y * (p.size + speed * velocity_stretch)) +
                       across_dir * (corner.x * p.size);
            }
        }
    }

    float c = cos(p.rotation);
    float s = sin(p.rotation);
    vec2 rotated = vec2(corner.x * c - corner.y * s, corner.x * s + corner.y * c);
    return (camera_right * rotated.x + camera_up * rotated.y) * p.size;
}

// The texture coordinate a quad corner samples its emitter's sprite atlas at.
//
// Without a flipbook this is the plain unit-quad mapping. With one, the atlas is a rows x columns
// grid walked in reading order — cell 0 top-left, which is v = 0 in Vulkan's image space — and the
// simulate pass has already chosen the particle's cell from its normalised age. The cell is only
// picked here, never interpolated, so the sub-image swaps cleanly instead of smearing between
// frames.
//
// @param corner  The unit quad corner in [-1,1]^2.
// @param frame   The particle's flipbook cell.
// @param rows    Atlas rows (1 = no flipbook).
// @param columns Atlas columns (1 = no flipbook).
vec2 particle_sprite_uv(vec2 corner, uint frame, uint rows, uint columns)
{
    vec2 uv = corner * 0.5 + 0.5;
    uint cells = max(rows * columns, 1u);
    if (cells <= 1u || columns == 0u)
        return uv;
    uint cell = min(frame, cells - 1u);
    vec2 cell_size = vec2(1.0 / float(columns), 1.0 / float(rows));
    vec2 origin = vec2(float(cell % columns), float(cell / columns)) * cell_size;
    return origin + uv * cell_size;
}

// The emitter's material, handed down to the fragment stage.
//
// One place, because four vertex shaders feed the same fragment: the two GPU sprite paths, the
// ribbon strip, and the host-uploaded deterministic billboards. The last of those has no emitter
// at all and passes zeroes, which reads as "untextured, unlit, not soft" — exactly what a
// gameplay particle with no authored material should be.
void particle_material(Emitter e, out uint texture_index, out uint render_flags, out float soft_fade)
{
    texture_index = e.texture;
    render_flags = e.render_flags;
    soft_fade = e.soft_fade_distance;
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

#endif // PARTICLE_COMMON_GLSL
