/**************************************************************************/
/* shadow_uniforms.cpp                                                    */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "scene/shadow_uniforms.hpp"

#include <algorithm>
#include <cmath>

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            namespace
            {
                /**
                 * @brief Extra depth pushed behind a cascade's sphere, in sphere radii.
                 *
                 * The light's near plane has to sit behind everything that can cast into
                 * the cascade, not merely behind the cascade itself — a tall object
                 * outside the view frustum still shadows into it.
                 */
                constexpr double CASTER_MARGIN = 3.0;

                /**
                 * @brief Half the angle the sun subtends from the ground, radians.
                 *
                 * About a quarter of a degree. It is the whole reason a real shadow is
                 * soft at all: the penumbra at a given point is the gap to its blocker
                 * times this, which is why a shadow is crisp where an object touches the
                 * ground and diffuse where it stands well above it.
                 */
                constexpr float SUN_ANGULAR_RADIUS = 0.00465f;

                /** @brief Normalises a vector, returning a fallback if it has no length. */
                Vector3 safe_normalize(const Vector3& v, const Vector3& fallback) noexcept
                {
                    const double length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
                    if (length < 1e-12)
                        return fallback;
                    return Vector3{v.x / length, v.y / length, v.z / length};
                }

                /** @brief Cross product a x b. */
                Vector3 cross_product(const Vector3& a, const Vector3& b) noexcept
                {
                    return Vector3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                                   a.x * b.y - a.y * b.x};
                }

                /** @brief Dot product a . b. */
                double dot_product(const Vector3& a, const Vector3& b) noexcept
                {
                    return a.x * b.x + a.y * b.y + a.z * b.z;
                }
            } // namespace

            std::uint32_t fit_shadow_cascades(const CameraView& camera, const double eye[3],
                                              const Vector3& sun,
                                              const ShadowSettings& settings,
                                              ShadowUniforms& uniforms) noexcept
            {
                uniforms = ShadowUniforms{};
                for (std::uint32_t cascade = 0; cascade < MAX_SHADOW_CASCADES; ++cascade)
                    for (int i = 0; i < 16; ++i)
                        uniforms.cascade_view_projection[cascade][i] = i % 5 == 0 ? 1.0f : 0.0f;

                const std::uint32_t count =
                    std::min(std::max(settings.cascade_count, 1u), MAX_SHADOW_CASCADES);
                const float resolution = static_cast<float>(std::max(settings.resolution, 64u));

                uniforms.params[0] = static_cast<float>(count);
                uniforms.params[1] = 0.5f; // two-by-two atlas
                uniforms.params[3] = std::min(std::max(settings.cascade_blend, 0.0f), 0.5f);
                uniforms.filter[0] = std::max(settings.filter_radius, 0.5f);
                uniforms.filter[1] = std::max(settings.max_filter_radius, uniforms.filter[0]);
                // How wide a penumbra grows per metre of gap between blocker and
                // receiver: the tangent of the sun's angular radius, scaled by the
                // author's exaggeration.
                uniforms.filter[2] = SUN_ANGULAR_RADIUS * std::max(settings.softness, 0.0f);
                uniforms.bias[0] = settings.depth_bias;
                uniforms.bias[1] = settings.normal_bias;
                uniforms.bias[2] = settings.contact_distance;
                uniforms.bias[3] = static_cast<float>(settings.contact_steps);
                uniforms.flags[3] = resolution;
                if (!settings.enabled)
                    return 0;
                uniforms.flags[0] = 1.0f;
                uniforms.flags[1] = settings.contact_shadows ? 1.0f : 0.0f;

                // The camera basis in world space, read straight out of the view matrix's
                // rows. The eye itself is deliberately not read: the whole fit is
                // camera-relative, so the eye is the origin.
                const Mat4& view = camera.view;
                const Mat4& projection = camera.projection;
                const Vector3 right{view.m[0], view.m[4], view.m[8]};
                const Vector3 up{view.m[1], view.m[5], view.m[9]};
                const Vector3 forward{-view.m[2], -view.m[6], -view.m[10]};
                const double tan_half_x = projection.m[0] != 0.0 ? 1.0 / projection.m[0] : 1.0;
                const double tan_half_y =
                    projection.m[5] != 0.0 ? 1.0 / (-projection.m[5]) : 1.0;

                const Vector3 light = safe_normalize(sun, Vector3{0.0, 1.0, 0.0});
                // Any axis not parallel to the light works; picking the less aligned of
                // two fixed axes keeps the light basis continuous as the sun moves.
                const Vector3 reference = std::fabs(light.y) > 0.95
                                              ? Vector3{0.0, 0.0, 1.0}
                                              : Vector3{0.0, 1.0, 0.0};
                const Vector3 light_x =
                    safe_normalize(cross_product(reference, light), Vector3{1.0, 0.0, 0.0});
                const Vector3 light_y = cross_product(light, light_x);

                const double near_plane = std::max(static_cast<double>(camera.near_plane), 1e-3);
                const double far_plane =
                    std::max(static_cast<double>(settings.distance), near_plane * 4.0);
                const double blend =
                    std::min(std::max(static_cast<double>(settings.split_blend), 0.0), 1.0);

                double split_near = near_plane;
                for (std::uint32_t cascade = 0; cascade < count; ++cascade)
                {
                    const double fraction = static_cast<double>(cascade + 1) / count;
                    // Practical split: the logarithmic term is what perspective wants,
                    // the uniform term is what keeps the far cascades from starving.
                    const double logarithmic =
                        near_plane * std::pow(far_plane / near_plane, fraction);
                    const double uniform = near_plane + (far_plane - near_plane) * fraction;
                    const double split_far = logarithmic * blend + uniform * (1.0 - blend);

                    // The cascade's bounding sphere, from the eight frustum corners. A
                    // sphere rather than the corners themselves because its extent does
                    // not change as the camera rotates, which is half of what keeps the
                    // shadow edges from crawling.
                    Vector3 corners[8];
                    for (int i = 0; i < 8; ++i)
                    {
                        const double distance = (i & 4) != 0 ? split_far : split_near;
                        const double x = ((i & 1) != 0 ? 1.0 : -1.0) * tan_half_x * distance;
                        const double y = ((i & 2) != 0 ? 1.0 : -1.0) * tan_half_y * distance;
                        corners[i] = Vector3{forward.x * distance + right.x * x + up.x * y,
                                             forward.y * distance + right.y * x + up.y * y,
                                             forward.z * distance + right.z * x + up.z * y};
                    }

                    Vector3 centre{0.0, 0.0, 0.0};
                    for (const Vector3& corner : corners)
                        centre = Vector3{centre.x + corner.x * 0.125, centre.y + corner.y * 0.125,
                                         centre.z + corner.z * 0.125};
                    double radius = 0.0;
                    for (const Vector3& corner : corners)
                    {
                        const Vector3 offset{corner.x - centre.x, corner.y - centre.y,
                                             corner.z - centre.z};
                        radius = std::max(radius, std::sqrt(dot_product(offset, offset)));
                    }
                    radius = std::max(radius, 1e-3);

                    // The other half: snapping the centre to whole shadow texels in light
                    // space, so a sub-texel camera movement cannot shift what each texel
                    // covers and set the edges shimmering.
                    // Anchored to the world, not to the camera. The fit is camera-relative,
                    // so a grid snapped in that space would be carried along by the camera
                    // and would stabilise nothing; adding the eye back before the snap and
                    // removing it after puts the grid in the world, where it belongs. Both
                    // steps are in double, so a planet-scale eye costs no precision.
                    const Vector3 eye_offset{eye[0], eye[1], eye[2]};
                    const double eye_x = dot_product(eye_offset, light_x);
                    const double eye_y = dot_product(eye_offset, light_y);
                    const double texel = 2.0 * radius / resolution;
                    const double centre_x =
                        std::floor((dot_product(centre, light_x) + eye_x) / texel) * texel - eye_x;
                    const double centre_y =
                        std::floor((dot_product(centre, light_y) + eye_y) / texel) * texel - eye_y;
                    const double centre_z = dot_product(centre, light);
                    centre = Vector3{
                        light_x.x * centre_x + light_y.x * centre_y + light.x * centre_z,
                        light_x.y * centre_x + light_y.y * centre_y + light.y * centre_z,
                        light_x.z * centre_x + light_y.z * centre_y + light.z * centre_z};

                    // World to light-clip, composed directly: an orthographic box of side
                    // 2r centred on the sphere, seen from a plane placed *toward the sun*
                    // far enough out to sit in front of every caster that can reach the
                    // cascade. @c light points toward the sun, so the origin plane is
                    // centre + light * offset, and depth therefore runs along -light: it
                    // is zero at that plane and grows with distance *away* from the sun,
                    // which is the only orientation under which "a smaller stored depth
                    // occludes" is true. Getting this axis backwards records the surface
                    // furthest from the light instead of the nearest and inverts every
                    // shadow test that follows.
                    //
                    // Depth maps to [0, 1] linearly, which is why the shadow pipeline
                    // clears to one and compares LESS rather than following the reverse-Z
                    // the main camera uses; an orthographic depth is already linear, so
                    // reverse-Z would buy it no precision.
                    const double origin_offset = radius * (1.0 + CASTER_MARGIN);
                    const double depth_range = origin_offset + radius;
                    const Vector3 origin{centre.x + light.x * origin_offset,
                                         centre.y + light.y * origin_offset,
                                         centre.z + light.z * origin_offset};

                    const double inverse_extent = 1.0 / radius;
                    const double inverse_depth = 1.0 / depth_range;
                    float* m = uniforms.cascade_view_projection[cascade];
                    m[0] = static_cast<float>(light_x.x * inverse_extent);
                    m[4] = static_cast<float>(light_x.y * inverse_extent);
                    m[8] = static_cast<float>(light_x.z * inverse_extent);
                    m[12] = static_cast<float>(-dot_product(origin, light_x) * inverse_extent);
                    m[1] = static_cast<float>(light_y.x * inverse_extent);
                    m[5] = static_cast<float>(light_y.y * inverse_extent);
                    m[9] = static_cast<float>(light_y.z * inverse_extent);
                    m[13] = static_cast<float>(-dot_product(origin, light_y) * inverse_extent);
                    // z = dot(origin - world, light) / range: zero on the origin plane,
                    // growing away from the sun.
                    m[2] = static_cast<float>(-light.x * inverse_depth);
                    m[6] = static_cast<float>(-light.y * inverse_depth);
                    m[10] = static_cast<float>(-light.z * inverse_depth);
                    m[14] = static_cast<float>(dot_product(origin, light) * inverse_depth);
                    m[3] = 0.0f;
                    m[7] = 0.0f;
                    m[11] = 0.0f;
                    m[15] = 1.0f;

                    uniforms.splits[cascade] = static_cast<float>(split_far);
                    uniforms.texel_size[cascade] = static_cast<float>(texel);
                    uniforms.depth_range[cascade] = static_cast<float>(depth_range);
                    split_near = split_far;
                }

                // Cascades the settings did not ask for reach no further than the last
                // one, so the shader's search never selects them.
                for (std::uint32_t cascade = count; cascade < MAX_SHADOW_CASCADES; ++cascade)
                {
                    uniforms.splits[cascade] = uniforms.splits[count - 1];
                    uniforms.texel_size[cascade] = uniforms.texel_size[count - 1];
                    uniforms.depth_range[cascade] = uniforms.depth_range[count - 1];
                }
                return count;
            }
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
