#version 450

// Editor-only reference grid. A fullscreen pass that ray-marches the local horizon
// plane (world y = 0, which in this engine's topocentric frame is the local ground) and
// paints an adaptive lat/long-free XZ grid onto it. It is entirely analytic: the line
// coverage comes from the screen-space derivative of the world position, so the lines are
// anti-aliased and hold perfectly still under camera motion — unlike the 1px hardware
// line primitives it replaces, which crawled and shimmered in daylight. The cell size
// follows a base-10 ladder keyed off the on-screen pixel footprint, so lines stay a
// roughly constant density whether the camera is a metre or a kilometre away, and the two
// bracketing decades cross-fade so nothing pops as you zoom.
//
// It composites in-shader (there is no hardware alpha blend in this engine) over the HDR
// post colour, and is occluded by scene geometry by comparing against the depth buffer.
// Registered only when the editor asks for it; a shipped runtime never runs this pass.

// Only the camera prefix of the scene block is needed; std140 keeps the offsets.
layout(set = 0, binding = 0) uniform SceneBlock
{
    mat4 view;
    mat4 proj;
    vec4 cam_forward; // xyz = unit forward, w = camera world pos x
    vec4 cam_right;   // xyz = right * tan(fovx/2), w = camera world pos y
    vec4 cam_up;      // xyz = up * tan(fovy/2), w = camera world pos z
} scene;

layout(set = 0, binding = 1) uniform sampler2D color_texture; // HDR post colour to composite over
layout(set = 0, binding = 2) uniform sampler2D depth_texture; // reverse-Z scene depth

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

// One grid level's coverage: ~1px-wide AA lines on a `cell`-metre lattice. `dP` is the
// world-space size of one pixel, so dividing by the cell gives the derivative in cell
// units and the line stays one pixel wide at every distance.
float grid_level(vec2 p, vec2 dP, float cell)
{
    vec2 g = p / cell;
    vec2 d = max(dP / cell, vec2(1e-6));
    vec2 a = abs(fract(g - 0.5) - 0.5) / d;
    return 1.0 - clamp(min(a.x, a.y), 0.0, 1.0);
}

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec3 base = texture(color_texture, uv).rgb;

    // Camera-relative frame: the eye is the origin, world axes. The eye's world position
    // rides in the .w lanes of the ray basis.
    vec3 rd = normalize(scene.cam_forward.xyz + v_ndc.x * scene.cam_right.xyz +
                        v_ndc.y * scene.cam_up.xyz);
    vec3 eye = vec3(scene.cam_forward.w, scene.cam_right.w, scene.cam_up.w);

    // Intersect the world y = 0 plane, which sits at camera-relative height -eye.y.
    if (abs(rd.y) < 1e-6)
    {
        out_color = vec4(base, 1.0);
        return;
    }
    float t = -eye.y / rd.y;
    if (t <= 0.0)
    {
        out_color = vec4(base, 1.0);
        return;
    }
    vec3 hit = rd * t;            // camera-relative hit
    vec2 p = hit.xz + eye.xz;     // world XZ (fine for the editor's near-origin scenes)

    // Occlusion: project the hit and compare against the stored reverse-Z depth. A larger
    // stored value is nearer, so geometry in front of the plane hides the grid.
    vec4 clip = scene.proj * scene.view * vec4(hit, 1.0);
    float grid_depth = clip.z / clip.w;
    float scene_depth = texture(depth_texture, uv).r;
    float occlusion = scene_depth > grid_depth + 1e-6 ? 0.0 : 1.0;

    // Adaptive base-10 ladder: pick the decade whose cell is ~12px on screen, cross-fade
    // the finer decade out into the coarser one so zooming never pops.
    vec2 dP = fwidth(p);
    float footprint = max(dP.x, dP.y);
    float level = log2(max(footprint, 1e-8) * 12.0) / log2(10.0);
    float fine_cell = pow(10.0, floor(level));
    float blend = fract(level);
    float fine = grid_level(p, dP, fine_cell);
    float coarse = grid_level(p, dP, fine_cell * 10.0);
    float lines = max(fine * (1.0 - blend), coarse);

    // The world axes: the X axis (world Z = 0) reads red, the Z axis (world X = 0) blue.
    float axis_x = 1.0 - clamp(abs(p.y) / max(dP.y, 1e-6), 0.0, 1.0);
    float axis_z = 1.0 - clamp(abs(p.x) / max(dP.x, 1e-6), 0.0, 1.0);

    vec3 line_color = vec3(0.42, 0.45, 0.52);
    line_color = mix(line_color, vec3(0.90, 0.27, 0.27), axis_x);
    line_color = mix(line_color, vec3(0.27, 0.45, 0.95), axis_z);
    float coverage = max(lines, max(axis_x, axis_z));

    // Fade the grid out toward the horizon, where the plane is seen edge-on and a fixed
    // filter would smear into a solid sheet.
    coverage *= smoothstep(0.0, 0.12, abs(rd.y));
    coverage *= occlusion;

    out_color = vec4(mix(base, line_color, coverage * 0.55), 1.0);
}
