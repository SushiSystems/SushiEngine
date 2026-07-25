#version 450
#extension GL_GOOGLE_include_directive : require

// Mesh-particle fragment shader (design §7.11, VFX4). Solid, opaque, depth-tested geometry, so
// unlike the sprite fragment there is no soft falloff and no manual depth discard — the depth test
// does that job properly here, which is the whole reason mesh particles draw with the opaque
// geometry rather than in the transparency pass.
//
// The shading is deliberately not pbr.frag: a mesh particle carries no material, and reaching the
// bindless material heap would tie this pass to the whole scene set. It is a diffuse surface lit by
// the sun — shadowed through the same cheap cascade sampler the lit sprites use — plus a flat
// ambient. A mesh particle that needs a real material is a mesh instance, not a particle.

#include "blue_noise.glsl"
#include "particle_shadow.glsl"

layout(set = 0, binding = 11) uniform sampler2DShadow shadow_atlas;

layout(push_constant) uniform Push
{
    mat4 view_projection;
    vec4 sun_direction; // xyz to-sun;                w = eye.x
    vec4 sun_radiance;  // rgb sun colour * intensity; w = eye.y
    vec4 ambient;       // rgb flat ambient;           w = eye.z
    uvec4 slice;        // x = first particle, y = particles in the slice
} pc;

layout(location = 0) in vec3 in_normal;
layout(location = 1) in vec4 in_color;
layout(location = 2) in vec4 in_light; // xyz = camera-relative position, w = view depth

layout(location = 0) out vec4 out_color;

void main()
{
    vec3 normal = normalize(in_normal);
    float n_dot_l = max(dot(normal, pc.sun_direction.xyz), 0.0);
    float rotation = interleaved_gradient_noise(gl_FragCoord.xy) * 6.28318530718;
    float visibility = particle_sun_shadow(shadow_atlas, in_light.xyz, in_light.w, rotation);

    vec3 lit = in_color.rgb * (pc.sun_radiance.rgb * (n_dot_l * visibility) + pc.ambient.rgb);
    out_color = vec4(lit, 1.0);
}
