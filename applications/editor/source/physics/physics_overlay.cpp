/**************************************************************************/
/* physics_overlay.cpp                                                    */
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

#include "physics_overlay.hpp"

#include <cmath>
#include <vector>

#include "../core/viewport_projection.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            using Simulation::EntityId;
            using Simulation::IWorldEditor;
            using Simulation::NULL_ENTITY;

            /** @brief How many segments approximate a limit arc; enough to read as a curve. */
            constexpr int ARC_SEGMENTS = 24;

            /** @brief How long a joint's axis is drawn, in metres. */
            constexpr Scalar AXIS_LENGTH = 0.4;

            /** @brief The radius the twist-limit arc is drawn at, in metres. */
            constexpr Scalar ARC_RADIUS = 0.3;

            /**
             * @brief A stable colour for an island index.
             *
             * A hash rather than a palette lookup, so an island number past the end of a
             * palette still gets a colour instead of wrapping onto one already in use. The
             * *number* is meaningless across ticks — the partition renumbers whenever it
             * changes — so what this has to preserve is only that two bodies with the same
             * index get the same colour within one frame.
             *
             * @param island The island index.
             */
            ImU32 island_color(std::uint32_t island)
            {
                std::uint32_t hash = island * 2654435761u;
                hash ^= hash >> 15;
                const int r = 90 + int(hash & 0x7Fu);
                const int g = 90 + int((hash >> 8) & 0x7Fu);
                const int b = 90 + int((hash >> 16) & 0x7Fu);
                return IM_COL32(r, g, b, 210);
            }

            /** @brief The twelve edges of a box, as index pairs into its eight corners. */
            const int BOX_EDGES[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7},
                                          {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

            /**
             * @brief Draws a world-space line, skipping it when either end is behind the camera.
             *
             * Skipping rather than clipping: clipping to the near plane is the right answer
             * and it is a different job, while drawing the unclipped line is the wrong one and
             * it draws a stripe across the whole viewport. The same rule the collider overlay
             * follows, for the same reason.
             */
            struct Projector
            {
                Matrix4 view_projection;
                ImVec2 origin;
                float width = 0;
                float height = 0;
                ImDrawList* list = nullptr;

                bool point(const Vector3& world, ImVec2& out) const
                {
                    return project_to_screen(view_projection, world, origin, width, height, out);
                }

                void line(const Vector3& a, const Vector3& b, ImU32 colour,
                          float thickness = 1.0f) const
                {
                    ImVec2 pa;
                    ImVec2 pb;
                    if (!point(a, pa) || !point(b, pb))
                        return;
                    list->AddLine(pa, pb, colour, thickness);
                }
            };

            /** @brief Draws a wire box from its two world-space corners. */
            void draw_bounds(const Projector& projector, const Vector3& low, const Vector3& high,
                             ImU32 colour)
            {
                const Vector3 corners[8] = {
                    Vector3{low.x, low.y, low.z},   Vector3{high.x, low.y, low.z},
                    Vector3{low.x, high.y, low.z},  Vector3{high.x, high.y, low.z},
                    Vector3{low.x, low.y, high.z},  Vector3{high.x, low.y, high.z},
                    Vector3{low.x, high.y, high.z}, Vector3{high.x, high.y, high.z}};
                for (const auto& edge : BOX_EDGES)
                    projector.line(corners[edge[0]], corners[edge[1]], colour);
            }

            /** @brief Any unit vector perpendicular to @p axis, chosen deterministically. */
            Vector3 perpendicular(const Vector3& axis)
            {
                // Cross with whichever cardinal axis this one is least aligned to, so the
                // result is never the cross of two parallel vectors — which is zero, and
                // normalizing zero is how an arc ends up drawn as a point.
                const Vector3 fallback = std::abs(axis.x) < Scalar(0.9) ? Vector3{1, 0, 0}
                                                                        : Vector3{0, 1, 0};
                return normalize(cross(axis, fallback));
            }

            /**
             * @brief Draws a joint's two anchors, its primary axis, and its twist-limit arc.
             *
             * The arc is the half of this that is worth the code. A hinge's limits are two
             * numbers in radians, and radians are exactly the quantity nobody can picture: an
             * arc drawn from the joint's own zero, in the joint's own plane, turns "0 to 1.7"
             * into a door that opens most of the way — and turns a sign error into a door
             * that opens backwards, visibly, before it is ever played.
             *
             * @param projector The frame's projection.
             * @param world     The world, for the partner's pose.
             * @param owner     The entity carrying the joint.
             * @param params    Its authoring.
             */
            void draw_joint_gizmo(const Projector& projector, IWorldEditor& world, EntityId owner,
                                  const Simulation::PhysicsJointParameters& params)
            {
                const Simulation::EntityTransform owner_transform = world.world_transform(owner);
                const Vector3 anchor_a =
                    owner_transform.position +
                    rotate(owner_transform.rotation, params.joint.anchor_a);

                // Amber for the joint itself, so it reads apart from the green collider
                // overlay it is usually drawn beside.
                const ImU32 joint_colour = IM_COL32(255, 190, 70, 230);
                const ImU32 partner_colour = IM_COL32(120, 180, 255, 200);

                ImVec2 screen_a;
                if (projector.point(anchor_a, screen_a))
                    projector.list->AddCircleFilled(screen_a, 4.0f, joint_colour);

                // The line between the two anchors is the joint's error, drawn: a settled
                // joint has none and the line is a point, and a joint being pulled apart is a
                // line whose length is how far apart it is being held.
                if (params.connected_body != NULL_ENTITY && world.exists(params.connected_body))
                {
                    const Simulation::EntityTransform other =
                        world.world_transform(params.connected_body);
                    const Vector3 anchor_b =
                        other.position + rotate(other.rotation, params.joint.anchor_b);
                    ImVec2 screen_b;
                    if (projector.point(anchor_b, screen_b))
                        projector.list->AddCircle(screen_b, 4.0f, partner_colour);
                    projector.line(anchor_a, anchor_b, partner_colour, 2.0f);
                }

                const Simulation::JointType type = params.joint.type;
                if (type == Simulation::JointType::Fixed || type == Simulation::JointType::Ball)
                    return;

                const Vector3 axis =
                    normalize(rotate(owner_transform.rotation, params.joint.axis_a));
                projector.line(anchor_a - axis * AXIS_LENGTH, anchor_a + axis * AXIS_LENGTH,
                               joint_colour, 2.0f);

                if (!params.joint.twist_limit.enabled)
                    return;

                // The arc lives in the plane the axis is normal to, swept from the limit's
                // lower bound to its upper one, with a spoke at each end so an author can see
                // where the range starts as well as how wide it is.
                const Vector3 u = perpendicular(axis);
                const Vector3 v = cross(axis, u);
                const Scalar lower = params.joint.twist_limit.lower;
                const Scalar upper = params.joint.twist_limit.upper;
                const auto arc_point = [&](Scalar angle)
                {
                    return anchor_a + (u * std::cos(double(angle)) + v * std::sin(double(angle))) *
                                          ARC_RADIUS;
                };
                Vector3 previous = arc_point(lower);
                for (int i = 1; i <= ARC_SEGMENTS; ++i)
                {
                    const Scalar t = Scalar(i) / Scalar(ARC_SEGMENTS);
                    const Vector3 current = arc_point(lower + (upper - lower) * t);
                    projector.line(previous, current, joint_colour);
                    previous = current;
                }
                projector.line(anchor_a, arc_point(lower), joint_colour);
                projector.line(anchor_a, arc_point(upper), joint_colour);
            }
        } // namespace

        void draw_physics_overlay(IWorldEditor& world, EntityId selected,
                                  const PhysicsOverlaySettings& settings,
                                  const Render::CameraView& camera_view,
                                  const ImVec2& image_origin, float width, float height,
                                  ImDrawList* draw_list)
        {
            if (draw_list == nullptr || !settings.any())
                return;

            Projector projector;
            projector.view_projection = mul(camera_view.projection, camera_view.view);
            projector.origin = image_origin;
            projector.width = width;
            projector.height = height;
            projector.list = draw_list;

            if (settings.bounds || settings.islands || settings.sleeping)
            {
                for (const EntityId id : world.entities())
                {
                    Simulation::RigidDebugState state;
                    if (!world.physics_body_debug(id, state))
                        continue;

                    if (settings.bounds)
                    {
                        // Island colour wins over the plain bound colour when both are on:
                        // the two are the same box, and drawing it twice in two colours would
                        // be one flickering box rather than two readings of it.
                        const ImU32 colour = settings.islands ? island_color(state.island)
                                                              : IM_COL32(80, 160, 255, 150);
                        draw_bounds(projector, state.bounds_min, state.bounds_max, colour);
                    }
                    else if (settings.islands && !state.is_static)
                    {
                        const Vector3 centre =
                            (state.bounds_min + state.bounds_max) * Scalar(0.5);
                        ImVec2 screen;
                        if (projector.point(centre, screen))
                            draw_list->AddCircleFilled(screen, 5.0f, island_color(state.island));
                    }

                    if (!settings.sleeping || !state.sleeping)
                        continue;
                    // A settled stack is *supposed* to be asleep, so the marker is quiet: a
                    // hollow ring, not a filled shape, because the interesting case is the
                    // body that is not asleep when everything around it is.
                    const Vector3 centre = (state.bounds_min + state.bounds_max) * Scalar(0.5);
                    ImVec2 screen;
                    if (projector.point(centre, screen))
                        draw_list->AddCircle(screen, 7.0f, IM_COL32(170, 170, 190, 200), 0, 1.5f);
                }
            }

            if (settings.contacts)
            {
                for (const Simulation::ContactEvent& contact : world.physics_contacts())
                {
                    // Red for a live contact, grey for one that has just ended. An `End` is
                    // drawn rather than filtered because a contact that vanishes for a tick
                    // and returns is a symptom, and a filtered stream hides exactly that.
                    const bool ended = contact.phase == Simulation::ContactPhase::End;
                    const ImU32 colour =
                        ended ? IM_COL32(140, 140, 140, 140) : IM_COL32(255, 90, 90, 220);
                    ImVec2 screen;
                    if (projector.point(contact.point, screen))
                        draw_list->AddCircleFilled(screen, ended ? 2.0f : 3.0f, colour);
                    // Scaled by the impulse the contact carried, capped, so a crash reads
                    // longer than a scrape — which is the one thing a contact point alone
                    // cannot tell an author.
                    const double impulse = double(contact.impulse);
                    const Scalar length_metres =
                        Scalar(0.15 + (impulse > 4.0 ? 0.35 : impulse * 0.0875));
                    projector.line(contact.point, contact.point + contact.normal * length_metres,
                                   colour);
                }
            }

            if (settings.joints && selected != NULL_ENTITY && world.exists(selected) &&
                world.has_joint(selected))
                draw_joint_gizmo(projector, world, selected, world.joint_params(selected));
        }
    } // namespace Editor
} // namespace SushiEngine
