#version 450
#extension GL_GOOGLE_include_directive : require

// Ribbon/trail vertex shader (design §7.10, VFX3b): vertex-less, like the sprite paths, but one
// instance expands into a whole strip instead of one quad. The draw is RIBBON_VERTICES vertices
// per instance — six per segment, TRAIL_POINTS - 1 segments — and this shader places each corner
// from the particle's recorded trail, which the simulate pass shifts one sample per frame with the
// newest position at index 0.
//
// The strip is camera-facing in the only sense a ribbon can be: each sample's offset is
// perpendicular to both the local trail direction and the eye ray, so the band always presents its
// width to the camera and twists with the path. Width and alpha taper toward the tail, which is
// what makes a trail read as trailing rather than as a fixed-width tube.
//
// The fragment shader is shared with the sprites, whose radial falloff expects a round sprite. A
// ribbon wants a soft edge across the band and none along it, so this shader emits v = 0.5: the
// radial term collapses to 1 - u², a soft-edged band, and no ribbon-specific fragment shader is
// needed.

#include "particle_common.glsl"

// The emitter table is read for the material only — the alignment that sends a particle to this
// bucket is already implied by being in it.
layout(std430, set = 0, binding = 0) readonly buffer DrawList { Particle draw[]; };
layout(std430, set = 0, binding = 12) readonly buffer Emitters { Emitter emitters[]; };
layout(std430, set = 0, binding = 13) readonly buffer Trail { vec4 trail[]; };

layout(push_constant) uniform Push
{
    mat4 view_projection; // camera view * projection (float)
    vec4 camera_right;    // xyz world-space camera right; w = eye.x
    vec4 camera_up;       // xyz world-space camera up;    w = eye.y
    vec4 sun_direction;   // xyz direction to the sun;     w = eye.z
    vec4 sun_radiance;    // rgb sun colour * intensity; w = lit/ambient flag
} pc;

layout(location = 0) out vec2 out_uv;
layout(location = 1) out vec4 out_color;
// See particle.vert: camera-relative position (xyz) + positive view depth (w) for the lighting.
layout(location = 2) out vec4 out_light;
// See particle.vert. A ribbon's texture runs *along* the trail rather than filling a square, so
// the atlas coordinate is (across the band, distance down the trail) — a strip texture reads head
// to tail, which is what a trail material is drawn as.
layout(location = 3) out vec2 out_atlas_uv;
layout(location = 4) flat out uvec2 out_material;
layout(location = 5) flat out float out_soft_fade;

// The six corners of one segment quad, as (sample offset, side): sample 0 is the newer end of the
// segment and 1 the older, side -1/+1 is the band's edge.
const vec2 SEGMENT[6] = vec2[](vec2(0.0, -1.0), vec2(0.0, 1.0), vec2(1.0, 1.0),
                               vec2(0.0, -1.0), vec2(1.0, 1.0), vec2(1.0, -1.0));

void main()
{
    Particle p = draw[gl_InstanceIndex];
    Emitter e = emitters[p.emitter_index];
    // Set at emit time and never rewritten, so the draw-list copy still names the pool slot whose
    // trail this is.
    uint base = p.seed * TRAIL_POINTS;
    // A beam shares this whole stage with a trail and differs only in where the samples come
    // from: an authored span evaluated at the sample's parameter, rather than the particle's
    // own recorded history.
    bool is_beam = e.alignment == ALIGN_BEAM;

    uint segment = uint(gl_VertexIndex) / 6u;
    vec2 corner = SEGMENT[uint(gl_VertexIndex) % 6u];
    uint sample_index = min(segment + uint(corner.x), TRAIL_POINTS - 1u);

    uint newer = max(sample_index, 1u) - 1u;
    uint older = min(sample_index + 1u, TRAIL_POINTS - 1u);
    float span = 1.0 / float(TRAIL_POINTS - 1u);

    vec4 here;
    // The local direction from the neighbours, so a corner shared by two segments resolves to the
    // same offset from both and the band has no crease at the seam.
    vec4 ahead;
    vec4 behind;
    if (is_beam)
    {
        here = particle_beam_sample(e, float(sample_index) * span, p.seed);
        ahead = particle_beam_sample(e, float(newer) * span, p.seed);
        behind = particle_beam_sample(e, float(older) * span, p.seed);
    }
    else
    {
        here = trail[base + sample_index];
        ahead = trail[base + newer];
        behind = trail[base + older];
    }

    vec3 eye = vec3(pc.camera_right.w, pc.camera_up.w, pc.sun_direction.w);
    vec3 camera_relative = here.xyz - eye;

    vec3 along = ahead.xyz - behind.xyz;
    vec3 side;
    if (dot(along, along) > 1e-10)
    {
        vec3 view_ray = normalize(camera_relative);
        vec3 across = cross(normalize(along), view_ray);
        // Degenerate only where the trail runs straight at or away from the eye, which is where a
        // ribbon has no width to show anyway; the camera axis keeps it from collapsing to a line.
        side = dot(across, across) > 1e-10 ? normalize(across) : pc.camera_right.xyz;
    }
    else
    {
        side = pc.camera_right.xyz; // a trail that has not moved yet
    }

    // Taper: the tail is thinner and fainter than the head. `here.w` is the sample's own recorded
    // size, so a size-over-life curve is already baked into the strip's profile. A beam is not a
    // trail — both its ends are authored — so it keeps its width and alpha the whole way.
    float t = float(sample_index) / float(TRAIL_POINTS - 1u);
    float taper = is_beam ? 1.0 : 1.0 - t;
    vec3 world = here.xyz + side * (corner.y * here.w * taper);

    out_uv = vec2(corner.y * 0.5 + 0.5, 0.5);
    out_atlas_uv = particle_sprite_uv(vec2(corner.y, t * 2.0 - 1.0), p.flipbook_frame,
                                      e.flipbook_rows, e.flipbook_columns);
    particle_material(e, out_material.x, out_material.y, out_soft_fade);
    out_color = vec4(p.cr, p.cg, p.cb, p.alpha * taper);
    gl_Position = pc.view_projection * vec4(world, 1.0);
    out_light = vec4(camera_relative, gl_Position.w);
}
