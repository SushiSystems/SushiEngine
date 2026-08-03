#version 460
#extension GL_EXT_ray_query : require

// Ray-traced sun shadows: the exact answer, for the top quality tier.
//
// A shadow cascade is a sampled approximation of this — it rasterises the scene from the
// light, quantises the result onto a texel grid, then spends bias and filtering hiding
// what that quantisation cost. Tracing the ray asks the geometry directly, so there is no
// resolution, no cascade boundary, no acne and no peter-panning.
//
// It writes a screen-space visibility mask rather than being folded into the material
// shader, which is deliberate: only this shader then needs the ray-query extension, so
// the material shader stays one build that runs on every device. Turning the tier on adds
// a pass; it does not fork the shading path.
//
// The acceleration structure holds the rasterised meshes only. The analytic planet ground
// is ray-marched in the sky pass and was never put into it, so ground shadowing stays with
// the cascades — this mask covers the geometry standing on that ground.
//
// Everything is camera-relative, matching the transforms the structure was built with:
// the eye is the origin, which is what keeps a planet-scale scene inside single precision.

layout(set = 0, binding = 0) uniform accelerationStructureEXT scene_structure;
layout(set = 0, binding = 1) uniform sampler2D depth_texture;

layout(push_constant) uniform Push
{
    vec4 forward;  // xyz = unit view axis,           w = near plane
    vec4 right;    // xyz = right * tan(fovx / 2),    w = ray length
    vec4 up;       // xyz = up * tan(fovy / 2),       w = ray start offset
    vec4 sun;      // xyz = direction toward the sun, camera-relative
} pc;

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out float out_visibility;

void main()
{
    out_visibility = 1.0;

    vec2 uv = v_ndc * 0.5 + 0.5;
    float depth = texture(depth_texture, uv).r;
    if (depth <= 0.0)
        return; // sky: no surface here to shadow

    // Reverse-Z with an infinite far plane: the stored depth is near / -view.z, so the
    // distance along the view axis is an exact inverse rather than a linearisation.
    float distance_along_view = pc.forward.w / depth;

    // The camera basis already carries the field of view, so this vector has a forward
    // component of exactly one and scaling it by the along-axis distance lands on the
    // surface — in camera-relative world space, which is the space the structure is in.
    vec3 position = (pc.forward.xyz + v_ndc.x * pc.right.xyz + v_ndc.y * pc.up.xyz) *
                    distance_along_view;

    // Offset along the ray before tracing. A surface reconstructed from a quantised depth
    // sits within a fraction of a texel of the real one, and a ray starting exactly on it
    // would hit the surface it started from. This is the traced equivalent of a shadow
    // map's depth bias — and unlike that one it is measured in metres, and it grows with
    // distance because a pixel covers more of the world the further away it is.
    float start = pc.up.w * max(distance_along_view * 0.002, 1.0);

    rayQueryEXT query;
    rayQueryInitializeEXT(query, scene_structure,
                          gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
                          0xFF, position, start, pc.sun.xyz, pc.right.w);
    // Every instance is opaque and the ray stops at its first hit, so the traversal needs
    // no proceed loop: one call runs it to completion.
    rayQueryProceedEXT(query);
    if (rayQueryGetIntersectionTypeEXT(query, true) !=
        gl_RayQueryCommittedIntersectionNoneEXT)
        out_visibility = 0.0;
}
