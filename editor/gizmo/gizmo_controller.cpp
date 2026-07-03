/**************************************************************************/
/* gizmo_controller.cpp                                                   */
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
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "gizmo_controller.hpp"

#include <cmath>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Vector3;

        namespace
        {
            constexpr float TWO_PI = 6.28318530718f;
            constexpr float DEG_TO_RAD = 0.01745329252f;

            // Projects a world point to panel-local screen pixels through the camera's
            // view-projection. Returns false when the point is behind the camera. The
            // projection already carries Vulkan's Y flip, so NDC maps straight to the image.
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

            // Shortest distance in pixels from point m to segment a-b: the handle hit test.
            float distance_to_segment(const ImVec2& a, const ImVec2& b, const ImVec2& m)
            {
                const float abx = b.x - a.x, aby = b.y - a.y;
                const float length_squared = abx * abx + aby * aby;
                float t = length_squared > 0.0f
                              ? ((m.x - a.x) * abx + (m.y - a.y) * aby) / length_squared
                              : 0.0f;
                t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
                const float dx = m.x - (a.x + t * abx), dy = m.y - (a.y + t * aby);
                return std::sqrt(dx * dx + dy * dy);
            }

            // Rounds v to the nearest multiple of step (>0), leaving it unchanged otherwise.
            float snap_to(float v, float step)
            {
                return step > 0.0f ? std::round(v / step) * step : v;
            }

            const Vector3 AXES[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
            const ImU32 AXIS_COLORS[3] = {IM_COL32(230, 80, 80, 255),
                                          IM_COL32(90, 220, 90, 255),
                                          IM_COL32(90, 150, 240, 255)};

            // Rotates v by unit quaternion q (v' = q v q^-1, expanded without building a matrix).
            Vector3 rotate_vector(const SushiEngine::Quaternion& q, const Vector3& v)
            {
                const Vector3 qv{q.x, q.y, q.z};
                const Vector3 t = cross(qv, v) * SushiEngine::Scalar(2);
                return v + t * q.w + cross(qv, t);
            }

            // A world-space ray from the camera through a screen pixel, built from the view
            // matrix's orthonormal basis (right/up/forward) and the projection's x/y scale
            // terms — the inverse of project_to_screen without needing a general Mat4 inverse.
            struct Ray
            {
                Vector3 origin;
                Vector3 direction;
            };

            Ray screen_to_ray(const SushiEngine::Mat4& view, const SushiEngine::Mat4& projection,
                              const ImVec2& screen, const ImVec2& origin, float width, float height)
            {
                using Scalar = SushiEngine::Scalar;
                const Vector3 right{view.m[0], view.m[4], view.m[8]};
                const Vector3 up{view.m[1], view.m[5], view.m[9]};
                const Vector3 forward{-view.m[2], -view.m[6], -view.m[10]};
                const Vector3 eye = right * static_cast<Scalar>(-view.m[12]) +
                                 up * static_cast<Scalar>(-view.m[13]) +
                                 forward * static_cast<Scalar>(view.m[14]);

                const float ndc_x = ((screen.x - origin.x) / width) * 2.0f - 1.0f;
                const float ndc_y = ((screen.y - origin.y) / height) * 2.0f - 1.0f;
                const float x_view = ndc_x / static_cast<float>(projection.m[0]);
                const float y_view = ndc_y / static_cast<float>(projection.m[5]);

                const Vector3 direction = SushiEngine::normalize(
                    right * static_cast<Scalar>(x_view) + up * static_cast<Scalar>(y_view) + forward);
                return Ray{eye, direction};
            }

            // Intersects a ray with the plane through point on normal n; false if near-parallel.
            bool intersect_plane(const Ray& ray, const Vector3& point, const Vector3& normal, Vector3& out)
            {
                using Scalar = SushiEngine::Scalar;
                const Scalar denom = dot(ray.direction, normal);
                if (std::abs(static_cast<double>(denom)) < 1e-6)
                    return false;
                const Scalar t = dot(point - ray.origin, normal) / denom;
                if (t < Scalar(0))
                    return false;
                out = ray.origin + ray.direction * t;
                return true;
            }
        }

        GizmoController::Result GizmoController::manipulate(
            GizmoMode mode, GizmoSpace space, SushiEngine::Simulation::EntityTransform& transform,
            const SushiEngine::Render::CameraView& camera_view, const ImVec2& image_origin,
            float width, float height, bool hovered, const GizmoSnap& snap)
        {
            Result result;

            const SushiEngine::Mat4 view_projection =
                SushiEngine::mul(camera_view.projection, camera_view.view);
            const Vector3 pivot = transform.position;

            // Calculate a scale factor to keep the gizmo roughly constant size on screen.
            const SushiEngine::Scalar* m = view_projection.m;
            float w = static_cast<float>(m[3] * pivot.x + m[7] * pivot.y + m[11] * pivot.z + m[15]);
            if (std::abs(w) < 0.0001f) w = 1.0f;

            float proj_y = static_cast<float>(camera_view.projection.m[5]);
            if (std::abs(proj_y) < 0.0001f) proj_y = 1.0f;

            float frustum_height;
            if (std::abs(static_cast<float>(camera_view.projection.m[15])) > 0.0001f)
            {
                // Orthographic projection
                frustum_height = 2.0f / std::abs(proj_y);
            }
            else
            {
                // Perspective projection
                frustum_height = std::abs(w) * 2.0f / std::abs(proj_y);
            }
            const float gizmo_scale = frustum_height * 0.15f; // Increased from 0.08f for better visibility

            // Scale always drags in local axes (see GizmoSpace); Translate/Rotate honour space_.
            const bool use_local = mode == GizmoMode::Scale || space == GizmoSpace::Local;

            ImVec2 origin_screen;
            const bool origin_ok =
                project_to_screen(view_projection, pivot, image_origin, width, height, origin_screen);

            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            const ImVec2 mouse = ImGui::GetIO().MousePos;
            const bool left_clicked = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
            const bool left_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);

            // A drag that began in a different mode or space is abandoned when either switches.
            if (axis_ >= 0 && (!left_down || mode_ != mode || space_ != space))
                axis_ = -1;

            // The handle basis: world axes, or the selection's own axes when Local (always
            // for Scale, per GizmoSpace). Drawn live so the handles turn with the object.
            const Vector3 draw_axes[3] = {use_local ? rotate_vector(transform.rotation, AXES[0]) : AXES[0],
                                       use_local ? rotate_vector(transform.rotation, AXES[1]) : AXES[1],
                                       use_local ? rotate_vector(transform.rotation, AXES[2]) : AXES[2]};

            if (mode == GizmoMode::Rotate)
            {
                // Three axis rings sampled as polylines around the pivot. A drag intersects
                // the mouse ray with the axis's own plane through the pivot each frame and
                // measures the signed angle swept between world-space vectors — this is
                // exact regardless of which side of the axis the camera views from, unlike
                // a screen-space angle (which inverts on the far side).
                const float ring_radius = 1.0f * gizmo_scale;
                constexpr int SEGMENTS = 48;

                // Basis for each ring: two axes perpendicular to the ring's own axis.
                const int perp_a[3] = {1, 2, 0};
                const int perp_b[3] = {2, 0, 1};

                if (origin_ok && axis_ < 0)
                    for (int a = 0; a < 3; ++a)
                    {
                        ImVec2 previous;
                        bool have_previous = false;
                        for (int s = 0; s <= SEGMENTS; ++s)
                        {
                            const float t = static_cast<float>(s) / SEGMENTS * TWO_PI;
                            const Vector3 p = pivot + draw_axes[perp_a[a]] * (std::cos(t) * ring_radius) +
                                           draw_axes[perp_b[a]] * (std::sin(t) * ring_radius);
                            ImVec2 sp;
                            if (project_to_screen(view_projection, p, image_origin, width, height, sp))
                            {
                                if (have_previous)
                                    draw_list->AddLine(previous, sp, AXIS_COLORS[a], 2.0f);
                                previous = sp;
                                have_previous = true;
                            }
                            else
                            {
                                have_previous = false;
                            }
                        }
                    }

                if (axis_ >= 0)
                {
                    const Ray ray = screen_to_ray(camera_view.view, camera_view.projection, mouse,
                                                  image_origin, width, height);
                    Vector3 hit;
                    if (intersect_plane(ray, pivot, axis_world_, hit))
                    {
                        const Vector3 current_vector = SushiEngine::normalize(hit - pivot);
                        float delta = std::atan2(
                            static_cast<float>(dot(cross(start_plane_vector_, current_vector), axis_world_)),
                            static_cast<float>(dot(start_plane_vector_, current_vector)));
                        if (snap.enabled)
                            delta = snap_to(delta, snap.rotate_degrees * DEG_TO_RAD);
                        const SushiEngine::Quaternion spin = SushiEngine::quaternion_axis_angle(
                            use_local ? AXES[axis_] : axis_world_, delta);
                        transform.rotation = SushiEngine::normalize(
                            use_local ? SushiEngine::mul(start_transform_.rotation, spin)
                                      : SushiEngine::mul(spin, start_transform_.rotation));
                    }
                    result.modified = true;
                    result.consumed_click = true;
                }
                else if (origin_ok && hovered && left_clicked)
                {
                    // Grab the ring whose polyline the cursor is nearest.
                    int best = -1;
                    float best_distance = 8.0f;
                    for (int a = 0; a < 3; ++a)
                    {
                        ImVec2 previous;
                        bool have_previous = false;
                        for (int s = 0; s <= SEGMENTS; ++s)
                        {
                            const float t = static_cast<float>(s) / SEGMENTS * TWO_PI;
                            const Vector3 p = pivot + draw_axes[perp_a[a]] * (std::cos(t) * ring_radius) +
                                           draw_axes[perp_b[a]] * (std::sin(t) * ring_radius);
                            ImVec2 sp;
                            if (project_to_screen(view_projection, p, image_origin, width, height, sp))
                            {
                                if (have_previous)
                                {
                                    const float d = distance_to_segment(previous, sp, mouse);
                                    if (d < best_distance)
                                    {
                                        best_distance = d;
                                        best = a;
                                    }
                                }
                                previous = sp;
                                have_previous = true;
                            }
                            else
                            {
                                have_previous = false;
                            }
                        }
                    }
                    if (best >= 0)
                    {
                        const Vector3 grab_axis = draw_axes[best];
                        const Ray ray = screen_to_ray(camera_view.view, camera_view.projection, mouse,
                                                      image_origin, width, height);
                        Vector3 hit;
                        if (intersect_plane(ray, pivot, grab_axis, hit))
                        {
                            axis_ = best;
                            mode_ = mode;
                            space_ = space;
                            axis_world_ = grab_axis;
                            start_plane_vector_ = SushiEngine::normalize(hit - pivot);
                            start_transform_ = transform;
                            result.consumed_click = true;
                        }
                    }
                }
                return result;
            }

            // Translate and Scale share axis handles from the pivot; only the drag mapping
            // and the tip glyph differ. Scale also gets a centre handle for uniform scale.
            const float handle_length = 1.2f * gizmo_scale;
            ImVec2 tip_screen[3];
            bool tip_ok[3] = {false, false, false};

            if (origin_ok)
                for (int a = 0; a < 3; ++a)
                {
                    const Vector3 tip = pivot + draw_axes[a] * handle_length;
                    tip_ok[a] =
                        project_to_screen(view_projection, tip, image_origin, width, height, tip_screen[a]);
                    if (tip_ok[a])
                    {
                        draw_list->AddLine(origin_screen, tip_screen[a], AXIS_COLORS[a], 3.0f);
                        if (mode == GizmoMode::Scale)
                            draw_list->AddRectFilled(
                                ImVec2(tip_screen[a].x - 4.0f, tip_screen[a].y - 4.0f),
                                ImVec2(tip_screen[a].x + 4.0f, tip_screen[a].y + 4.0f), AXIS_COLORS[a]);
                    }
                }
            if (mode == GizmoMode::Scale && origin_ok)
                draw_list->AddRectFilled(ImVec2(origin_screen.x - 5.0f, origin_screen.y - 5.0f),
                                         ImVec2(origin_screen.x + 5.0f, origin_screen.y + 5.0f),
                                         IM_COL32(220, 220, 220, 255));

            if (axis_ >= 0)
            {
                if (axis_ == 3)
                {
                    // Uniform scale: vertical drag grows/shrinks all axes together.
                    const float along = start_mouse_.y - mouse.y;
                    float factor = 1.0f + along * 0.01f;
                    if (factor < 0.01f)
                        factor = 0.01f;
                    Vector3 s = start_transform_.scale * static_cast<SushiEngine::Scalar>(factor);
                    if (snap.enabled)
                        s = Vector3{snap_to(static_cast<float>(s.x), snap.scale),
                                 snap_to(static_cast<float>(s.y), snap.scale),
                                 snap_to(static_cast<float>(s.z), snap.scale)};
                    transform.scale = s;
                }
                else
                {
                    const float along = (mouse.x - start_mouse_.x) * axis_screen_.x +
                                        (mouse.y - start_mouse_.y) * axis_screen_.y;
                    float world = along * world_per_pixel_;
                    if (mode == GizmoMode::Translate)
                    {
                        if (snap.enabled)
                            world = snap_to(world, snap.translate);
                        transform.position = start_transform_.position +
                                             axis_world_ * static_cast<SushiEngine::Scalar>(world);
                    }
                    else
                    {
                        float component = (axis_ == 0 ? static_cast<float>(start_transform_.scale.x)
                                           : axis_ == 1 ? static_cast<float>(start_transform_.scale.y)
                                                        : static_cast<float>(start_transform_.scale.z)) +
                                          world;
                        if (component < 0.01f)
                            component = 0.01f;
                        if (snap.enabled)
                            component = snap_to(component, snap.scale);
                        transform.scale = start_transform_.scale;
                        if (axis_ == 0) transform.scale.x = component;
                        else if (axis_ == 1) transform.scale.y = component;
                        else transform.scale.z = component;
                    }
                }
                result.modified = true;
                result.consumed_click = true;
            }
            else if (origin_ok && hovered && left_clicked)
            {
                // Uniform centre handle (scale only) wins if the cursor is over the pivot.
                if (mode == GizmoMode::Scale)
                {
                    const float cdx = mouse.x - origin_screen.x, cdy = mouse.y - origin_screen.y;
                    if (std::sqrt(cdx * cdx + cdy * cdy) < 6.0f)
                    {
                        axis_ = 3;
                        mode_ = mode;
                        space_ = space;
                        start_mouse_ = mouse;
                        start_transform_ = transform;
                        result.consumed_click = true;
                        return result;
                    }
                }

                int best = -1;
                float best_distance = 8.0f;
                for (int a = 0; a < 3; ++a)
                {
                    if (!tip_ok[a])
                        continue;
                    const float distance = distance_to_segment(origin_screen, tip_screen[a], mouse);
                    if (distance < best_distance)
                    {
                        best_distance = distance;
                        best = a;
                    }
                }
                if (best >= 0)
                {
                    const float dx = tip_screen[best].x - origin_screen.x;
                    const float dy = tip_screen[best].y - origin_screen.y;
                    const float screen_length = std::sqrt(dx * dx + dy * dy);
                    if (screen_length > 0.001f)
                    {
                        axis_ = best;
                        mode_ = mode;
                        space_ = space;
                        axis_world_ = draw_axes[best];
                        start_mouse_ = mouse;
                        start_transform_ = transform;
                        axis_screen_ = ImVec2(dx / screen_length, dy / screen_length);
                        world_per_pixel_ = handle_length / screen_length;
                        result.consumed_click = true;
                    }
                }
            }

            return result;
        }
    } // namespace Editor
} // namespace SushiEngine
