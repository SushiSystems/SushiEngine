/**************************************************************************/
/* skeleton_debug_draw.hpp                                               */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#ifndef SUSHIENGINE_EDITOR_SKELETON_DEBUG_DRAW_HPP
#define SUSHIENGINE_EDITOR_SKELETON_DEBUG_DRAW_HPP

#include <cstddef>
#include <vector>

#include <imgui.h>

#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/scene_view.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief A rigged glTF loaded for skeleton preview, and its bind-pose joint layout.
         *
         * Owns an @ref SushiEngine::Animation::AnimationDatabase holding the imported
         * skeleton, and caches the bind-pose model-space position of every joint (a forward
         * scan over the topologically sorted parents) so the viewport overlay only has to
         * transform and project them. This is the A0 "load a rigged glTF, see its rest pose"
         * surface: no Animator, no clip — just the rest skeleton drawn in the scene.
         */
        class SkeletonPreview
        {
            public:
                /**
                 * @brief Imports a glTF skin and computes its bind-pose joint layout.
                 * @param path Path to a `.gltf` or `.glb` file with a skin.
                 * @return True on success; false if the file has no importable skeleton.
                 */
                bool load_gltf(const char* path);

                /** @brief Drops the loaded skeleton, if any. */
                void clear();

                /** @brief Whether a skeleton is loaded and ready to draw. */
                bool loaded() const noexcept { return skeleton_.valid(); }

                /** @brief The loaded skeleton view (invalid when nothing is loaded). */
                const SushiEngine::Animation::SkeletonView& skeleton() const noexcept
                {
                    return skeleton_;
                }

                /** @brief Bind-pose model-space joint positions, one per joint. */
                const std::vector<SushiEngine::Vector3>& joint_positions() const noexcept
                {
                    return joint_positions_;
                }

                /** @brief The world placement of the skeleton (identity = at the origin). */
                const SushiEngine::Mat4& world() const noexcept { return world_; }

                /** @brief Sets where in the world the skeleton is drawn. */
                void set_world(const SushiEngine::Mat4& world) { world_ = world; }

                /**
                 * @brief Overrides the drawn joint positions with a posed set (model space).
                 *
                 * The Animation window feeds an evaluated pose here so the overlay shows the
                 * skeleton animating as the clip is scrubbed, rather than its rest pose. Ignored
                 * unless the count matches the loaded skeleton's joint count.
                 *
                 * @param positions One model-space position per joint.
                 */
                void set_pose_positions(const std::vector<SushiEngine::Vector3>& positions)
                {
                    if (positions.size() == joint_positions_.size())
                        joint_positions_ = positions;
                }

                /** @brief Restores the drawn positions to the rest (bind) pose. */
                void restore_bind() { joint_positions_ = bind_positions_; }

            private:
                SushiEngine::Animation::AnimationDatabase database_;
                SushiEngine::Animation::SkeletonView skeleton_{};
                std::vector<SushiEngine::Vector3> joint_positions_;
                std::vector<SushiEngine::Vector3> bind_positions_;
                SushiEngine::Mat4 world_{};
        };

        /**
         * @brief Draws a skeleton's bones, joints, and names over a viewport image.
         *
         * Projects each joint through the panel camera (the same view-projection the gizmo
         * uses), draws a line from every joint to its parent, a small octahedron at each
         * joint, and — when @p show_names — the joint name beside it. Call from inside a
         * viewport panel's ImGui window, after the scene image, with the panel's draw list.
         *
         * @param preview      The loaded skeleton; a no-op if it holds nothing.
         * @param camera_view  The panel camera's view and projection this frame.
         * @param image_origin Top-left of the rendered image in screen pixels.
         * @param width        Image width in pixels.
         * @param height       Image height in pixels.
         * @param draw_list    The ImGui draw list to paint into (the window's).
         * @param show_names   Whether to label each joint with its name.
         */
        void draw_skeleton_overlay(const SkeletonPreview& preview,
                                   const SushiEngine::Render::CameraView& camera_view,
                                   const ImVec2& image_origin, float width, float height,
                                   ImDrawList* draw_list, bool show_names);
    } // namespace Editor
} // namespace SushiEngine

#endif
