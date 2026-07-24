/**************************************************************************/
/* skeleton_debug_draw.cpp                                               */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "skeleton_debug_draw.hpp"

#include <cmath>

#include <SushiEngine/animation/gltf_skeleton_import.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Vector3;
        using SushiEngine::Animation::NO_PARENT;
        using SushiEngine::Animation::SkeletonView;

        namespace
        {
            // Projects a world point to panel-local screen pixels through the camera's
            // view-projection — the same mapping the gizmo uses. Returns false behind the
            // camera. The projection already carries Vulkan's Y flip.
            bool project_to_screen(const SushiEngine::Mat4& view_projection, const Vector3& point,
                                   const ImVec2& origin, float width, float height, ImVec2& out)
            {
                using Scalar = SushiEngine::Scalar;
                const Scalar* m = view_projection.m;
                const Scalar x = m[0] * point.x + m[4] * point.y + m[8] * point.z + m[12];
                const Scalar y = m[1] * point.x + m[5] * point.y + m[9] * point.z + m[13];
                const Scalar w = m[3] * point.x + m[7] * point.y + m[11] * point.z + m[15];
                if (w <= Scalar(0.0001))
                    return false;
                out.x = origin.x + static_cast<float>(x / w * Scalar(0.5) + Scalar(0.5)) * width;
                out.y = origin.y + static_cast<float>(y / w * Scalar(0.5) + Scalar(0.5)) * height;
                return true;
            }

            // Transforms a point by an affine matrix (w = 1).
            Vector3 transform_point(const SushiEngine::Mat4& matrix, const Vector3& p)
            {
                const SushiEngine::Scalar* m = matrix.m;
                return Vector3{m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12],
                               m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
                               m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]};
            }
        } // namespace

        bool SkeletonPreview::load_gltf(const char* path)
        {
            clear();
            std::vector<std::byte> blob;
            if (!SushiEngine::Animation::import_gltf_skeleton(path, blob))
                return false;
            const SushiEngine::Animation::AssetId id = database_.add_skeleton(std::move(blob));
            if (id == SushiEngine::Animation::INVALID_ASSET)
                return false;
            skeleton_ = database_.skeleton(id);
            if (!skeleton_.valid())
                return false;

            // Bind-pose model-space joint positions: a forward scan (parent[i] < i), the
            // translation column of each joint's composed model matrix.
            std::vector<SushiEngine::Mat4> model(skeleton_.joint_count);
            joint_positions_.resize(skeleton_.joint_count);
            for (std::uint32_t i = 0; i < skeleton_.joint_count; ++i)
            {
                const auto& t = skeleton_.bind_translations[i];
                const auto& r = skeleton_.bind_rotations[i];
                const auto& s = skeleton_.bind_scales[i];
                const SushiEngine::Mat4 local = SushiEngine::compose_transform(
                    Vector3{t.x, t.y, t.z}, SushiEngine::Quaternion{r.x, r.y, r.z, r.w},
                    Vector3{s.x, s.y, s.z});
                model[i] = skeleton_.parents[i] == NO_PARENT
                               ? local
                               : SushiEngine::mul(model[skeleton_.parents[i]], local);
                joint_positions_[i] = Vector3{model[i].m[12], model[i].m[13], model[i].m[14]};
            }
            bind_positions_ = joint_positions_;
            return true;
        }

        void SkeletonPreview::clear()
        {
            skeleton_ = SkeletonView{};
            joint_positions_.clear();
            bind_positions_.clear();
            world_ = SushiEngine::Mat4{};
        }

        void draw_skeleton_overlay(const SkeletonPreview& preview,
                                   const SushiEngine::Render::CameraView& camera_view,
                                   const ImVec2& image_origin, float width, float height,
                                   ImDrawList* draw_list, bool show_names)
        {
            if (!preview.loaded() || draw_list == nullptr)
                return;
            const SkeletonView& skeleton = preview.skeleton();
            const std::vector<Vector3>& local_positions = preview.joint_positions();
            const SushiEngine::Mat4& world = preview.world();
            const SushiEngine::Mat4 view_projection =
                SushiEngine::mul(camera_view.projection, camera_view.view);

            // World joint positions, and a joint-marker size from the longest bone so the
            // octahedra stay proportional to the rig whatever its scale.
            std::vector<Vector3> world_positions(skeleton.joint_count);
            SushiEngine::Scalar longest_bone = 0;
            for (std::uint32_t i = 0; i < skeleton.joint_count; ++i)
            {
                world_positions[i] = transform_point(world, local_positions[i]);
                if (skeleton.parents[i] != NO_PARENT)
                {
                    const SushiEngine::Scalar bone =
                        SushiEngine::length(local_positions[i] - local_positions[skeleton.parents[i]]);
                    if (bone > longest_bone)
                        longest_bone = bone;
                }
            }
            SushiEngine::Scalar marker = longest_bone * SushiEngine::Scalar(0.12);
            if (marker <= SushiEngine::Scalar(0))
                marker = SushiEngine::Scalar(0.05);

            const ImU32 bone_color = IM_COL32(210, 210, 90, 220);
            const ImU32 joint_color = IM_COL32(90, 220, 230, 255);
            const ImU32 name_color = IM_COL32(235, 235, 235, 255);

            // Bones: a line from each joint to its parent.
            for (std::uint32_t i = 0; i < skeleton.joint_count; ++i)
            {
                if (skeleton.parents[i] == NO_PARENT)
                    continue;
                ImVec2 a, b;
                if (project_to_screen(view_projection, world_positions[i], image_origin, width, height, a) &&
                    project_to_screen(view_projection, world_positions[skeleton.parents[i]], image_origin,
                                      width, height, b))
                    draw_list->AddLine(a, b, bone_color, 2.0f);
            }

            // Joints: a small world-space octahedron (six axis tips, twelve silhouette edges).
            static const int OCTA_EDGES[12][2] = {{0, 2}, {0, 3}, {0, 4}, {0, 5}, {1, 2}, {1, 3},
                                                  {1, 4}, {1, 5}, {2, 4}, {4, 3}, {3, 5}, {5, 2}};
            for (std::uint32_t i = 0; i < skeleton.joint_count; ++i)
            {
                const Vector3& c = world_positions[i];
                const Vector3 verts[6] = {
                    Vector3{c.x + marker, c.y, c.z}, Vector3{c.x - marker, c.y, c.z},
                    Vector3{c.x, c.y + marker, c.z}, Vector3{c.x, c.y - marker, c.z},
                    Vector3{c.x, c.y, c.z + marker}, Vector3{c.x, c.y, c.z - marker}};
                ImVec2 screen[6];
                bool ok[6];
                for (int v = 0; v < 6; ++v)
                    ok[v] = project_to_screen(view_projection, verts[v], image_origin, width, height,
                                              screen[v]);
                for (const auto& edge : OCTA_EDGES)
                    if (ok[edge[0]] && ok[edge[1]])
                        draw_list->AddLine(screen[edge[0]], screen[edge[1]], joint_color, 1.5f);

                if (show_names)
                {
                    ImVec2 label;
                    if (project_to_screen(view_projection, c, image_origin, width, height, label))
                        draw_list->AddText(ImVec2(label.x + 6.0f, label.y - 6.0f), name_color,
                                           skeleton.joint_name(i));
                }
            }
        }
    } // namespace Editor
} // namespace SushiEngine
