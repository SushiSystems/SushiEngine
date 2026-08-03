/**************************************************************************/
/* terrain_frame.hpp                                                      */
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

#pragma once

/**
 * @file terrain_frame.hpp
 * @brief The change of frame between the camera and the body whose ground it is looking at.
 *
 * The scene is anchored to an observer standing on a body; the terrain lives in that
 * body's own fixed frame, where its elevations were baked and where its ellipsoid sits at
 * the origin (`docs/slop/solar_system_overhaul.md` §9). Something has to carry the camera
 * and its frustum across that boundary once per frame, and this is it.
 *
 * Deliberately free of Vulkan and of @ref PlanetTerrain: it is arithmetic over an
 * environment and a camera, and both the arithmetic and the convention it commits to
 * (which way the matrix reads, which half-space a plane keeps) are worth checking without
 * a device, a pack, or a frame in flight.
 *
 * The rotation itself is not derived here. It arrives on the environment as
 * `Environment::planet_body_axes`, filled by the ephemeris, which is the only layer that
 * knows a body's pole and where its prime meridian points right now.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/environment/environment.hpp>
#include <SushiEngine/render/scene_view.hpp>
#include <SushiEngine/terrain/quadtree.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Terrain
        {
            /** @brief Where the body is being looked at from, this frame. */
            struct TerrainFrameView
            {
                /** @brief Camera position in the body's own fixed frame, metres. */
                Vector3 camera_body_fixed{Vector3{0.0, 0.0, 0.0}};

                /**
                 * @brief Body-fixed to scene rotation, column-major, translation-free.
                 *
                 * Supplied by the caller rather than derived here: the body's pole and its
                 * prime meridian belong to the astro layer, and terrain has no business
                 * reimplementing them.
                 */
                float body_to_scene[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                           0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

                double viewport_height_pixels = 1080.0;
                double vertical_field_of_view_radians = 1.0471975511965976;

                std::uint64_t frame_index = 0;
                std::uint32_t frame_slot = 0;

                /** @brief Frustum in the *body-fixed* camera-relative frame, or null. */
                const SushiEngine::Terrain::FrustumPlanes* frustum = nullptr;
            };

            namespace Detail
            {
                /** @brief Re-expresses a scene direction in the body's fixed axes. */
                inline Vector3 into_body(const Vector3 axes[3], const Vector3& scene) noexcept
                {
                    return Vector3{dot(scene, axes[0]), dot(scene, axes[1]),
                                   dot(scene, axes[2])};
                }

                /**
                 * @brief Writes one half-space, normal pointing into the volume.
                 *
                 * Takes the normal in scene axes and turns it into the body's, which is all
                 * that is needed: both frames share the camera as their origin, so a pure
                 * rotation leaves the offset alone.
                 */
                inline void write_plane(double plane[4], const Vector3 axes[3],
                                        const Vector3& normal, double offset) noexcept
                {
                    const Vector3 body = into_body(axes, normalize(normal));
                    plane[0] = body.x;
                    plane[1] = body.y;
                    plane[2] = body.z;
                    plane[3] = offset;
                }
            } // namespace Detail

            /**
             * @brief Expresses this frame's camera in the dominant body's fixed frame.
             *
             * Fills @p view with the camera position, the body-fixed to scene rotation as
             * the shader wants it, and the projection terms the screen-space error needs,
             * and fills @p frustum with the view volume rotated into the same body-fixed
             * axes. Both are outputs rather than a returned pair because @p view points at
             * @p frustum, and an owning return value would leave that pointer dangling at
             * the first copy.
             *
             * @param camera          This frame's camera.
             * @param environment     The environment the ephemeris filled.
             * @param eye             The camera's scene position, metres.
             * @param viewport_height Render height in pixels; sets the error budget.
             * @param frame_index     Monotonic frame counter, for residency ageing.
             * @param frame_slot      Which frame-in-flight slot is being recorded.
             * @param frustum         Receives the body-fixed frustum.
             * @param view            Receives the frame view, pointing at @p frustum.
             */
            inline void build_terrain_frame(const CameraView& camera,
                                            const Environment& environment, const double eye[3],
                                            std::uint32_t viewport_height,
                                            std::uint64_t frame_index, std::uint32_t frame_slot,
                                            SushiEngine::Terrain::FrustumPlanes& frustum,
                                            TerrainFrameView& view)
            {
                const Vector3* axes = environment.planet_body_axes;

                // The camera relative to the body's centre, then rotated into the body's own
                // axes. Double throughout: the offset is a planetary radius, and the
                // selector's whole precision argument rests on it arriving intact.
                const Vector3 offset{eye[0] - environment.planet_center.x,
                                     eye[1] - environment.planet_center.y,
                                     eye[2] - environment.planet_center.z};
                view.camera_body_fixed = Detail::into_body(axes, offset);

                // Column-major, as the shader's mat4 reads it: each body-fixed axis becomes
                // one column, so mat3(body_to_scene) * v carries v back to the scene.
                for (int axis = 0; axis < 3; ++axis)
                {
                    view.body_to_scene[axis * 4 + 0] = static_cast<float>(axes[axis].x);
                    view.body_to_scene[axis * 4 + 1] = static_cast<float>(axes[axis].y);
                    view.body_to_scene[axis * 4 + 2] = static_cast<float>(axes[axis].z);
                    view.body_to_scene[axis * 4 + 3] = 0.0f;
                }
                view.body_to_scene[12] = 0.0f;
                view.body_to_scene[13] = 0.0f;
                view.body_to_scene[14] = 0.0f;
                view.body_to_scene[15] = 1.0f;

                // The camera basis, read out of the view matrix's rows exactly as the shadow
                // cascade fit reads it (`scene/shadow_uniforms.cpp`), and the two half-angles
                // out of the projection's diagonal. The projection is Y-flipped, hence the
                // sign on the second.
                const Matrix4& matrix = camera.view;
                const Vector3 right{matrix.m[0], matrix.m[4], matrix.m[8]};
                const Vector3 up{matrix.m[1], matrix.m[5], matrix.m[9]};
                const Vector3 forward{-matrix.m[2], -matrix.m[6], -matrix.m[10]};
                const double tan_half_x =
                    camera.projection.m[0] != 0.0 ? 1.0 / camera.projection.m[0] : 1.0;
                const double tan_half_y =
                    camera.projection.m[5] != 0.0 ? 1.0 / (-camera.projection.m[5]) : 1.0;

                view.viewport_height_pixels = static_cast<double>(viewport_height);
                view.vertical_field_of_view_radians = 2.0 * std::atan(std::fabs(tan_half_y));
                view.frame_index = frame_index;
                view.frame_slot = frame_slot;

                // The five planes that bound a perspective view with no far clip. A point is
                // inside the left plane when its camera-space x is at least -tan * z, which
                // is the half-space of `right + tan_half_x * forward` through the eye; the
                // other three sides are that statement mirrored. The frustum is symmetric,
                // so whichever sign the basis carries does not matter.
                Detail::write_plane(frustum.plane[0], axes, forward,
                                    -static_cast<double>(camera.near_plane));
                Detail::write_plane(frustum.plane[1], axes, right + forward * tan_half_x, 0.0);
                Detail::write_plane(frustum.plane[2], axes, right * -1.0 + forward * tan_half_x,
                                    0.0);
                Detail::write_plane(frustum.plane[3], axes, up + forward * tan_half_y, 0.0);
                Detail::write_plane(frustum.plane[4], axes, up * -1.0 + forward * tan_half_y,
                                    0.0);
                // The far plane is left zeroed, which rejects nothing: the engine draws with
                // an infinite far plane, so there is no far half-space to test.
                for (int component = 0; component < 4; ++component)
                    frustum.plane[5][component] = 0.0;

                view.frustum = &frustum;
            }
        } // namespace Terrain
    } // namespace Render
} // namespace SushiEngine
