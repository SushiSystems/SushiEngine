/**************************************************************************/
/* compound_manifold.hpp                                                  */
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
 * @file compound_manifold.hpp
 * @brief One body, several colliders.
 *
 * A car chassis is not a convex shape and is not a mesh: it is a dozen convex
 * pieces that move as one rigid body (§5.2). That is what a compound is, and it
 * is the reason convex decomposition (P4) produces something the runtime can
 * use rather than something it has to approximate.
 *
 * The implementation is deliberately thin, because the interesting part is
 * elsewhere. A compound has no narrowphase of its own: each part is placed by
 * the body's pose, collided through the ordinary dispatch table, and the
 * manifolds are emitted **per part pair** rather than merged. Merging would be
 * the mistake — two parts of a chassis touching a kerb at two angles are two
 * contacts with two normals, and a single averaged normal is how a car sinks
 * into geometry it should be resting on.
 *
 * The anchors are already in the *body's* frame rather than the part's, because
 * every routine takes the body centre and orientation as the anchor frame
 * rather than reading it off the shape. So the solver never learns that
 * compounds exist, which is the point.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/collision/narrowphase_dispatch.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief One part of a compound: a shape and where it sits on the body.
         *
         * The transform is body-local and fixed — a compound's parts do not move
         * relative to each other, which is exactly what distinguishes a compound
         * from an assembly of jointed bodies (§5.4's `PhysicsAssembly`, P3).
         */
        template <typename T>
        struct CompoundPart
        {
            CollisionShape<T> shape;
            Vector3T<T> local_position{Vector3T<T>{T(0), T(0), T(0)}};
            QuaternionT<T> local_orientation{QuaternionT<T>{T(0), T(0), T(0), T(1)}};
        };

        /**
         * @brief The part, placed by the body's pose.
         *
         * The part's stored `shape.center` and `shape.orientation` are ignored in
         * favour of the local transform, so a part can be authored once and
         * instanced anywhere. A plane part is left alone: a half-space is defined by
         * its own normal and offset and has no centre to move.
         */
        template <typename T>
        inline CollisionShape<T> place_compound_part(const CompoundPart<T>& part,
                                                     const Vector3T<T>& body_center,
                                                     const QuaternionT<T>& body_orientation) noexcept
        {
            CollisionShape<T> placed = part.shape;
            if (placed.type == ShapeType::plane)
                return placed;
            placed.center = body_center + rotate(body_orientation, part.local_position);
            placed.orientation = mul(body_orientation, part.local_orientation);
            return placed;
        }

        /** @brief The world-space box enclosing every part of a placed compound. */
        template <typename T>
        inline AABB<T> compound_bounds(const CompoundPart<T>* parts, std::uint32_t part_count,
                                       const Vector3T<T>& body_center,
                                       const QuaternionT<T>& body_orientation) noexcept
        {
            AABB<T> bounds{body_center, body_center};
            for (std::uint32_t i = 0; i < part_count; ++i)
            {
                const CollisionShape<T> placed =
                    place_compound_part(parts[i], body_center, body_orientation);
                if (placed.type == ShapeType::plane)
                    continue; // a half-space has no bounds worth taking a union with
                const AABB<T> part_bounds = shape_world_bounds(placed);
                bounds = i == 0 ? part_bounds : aabb_union(bounds, part_bounds);
            }
            return bounds;
        }

        /**
         * @brief Every manifold between a compound body and a single shape.
         *
         * @param parts       The compound's parts.
         * @param part_count  How many.
         * @param body_center The compound body's centre; also the anchor frame.
         * @param body_orientation The compound body's orientation.
         * @param other       The other shape, already placed in the world.
         * @param other_center The other body's centre, for its anchors.
         * @param other_orientation The other body's orientation.
         * @param emit        Called as `emit(manifold, part_index)`.
         */
        template <typename T, typename Emit>
        inline void generate_compound_manifolds(
            const CompoundPart<T>* parts, std::uint32_t part_count,
            const Vector3T<T>& body_center, const QuaternionT<T>& body_orientation,
            const CollisionShape<T>& other, const Vector3T<T>& other_center,
            const QuaternionT<T>& other_orientation, T contact_offset, T face_tolerance,
            Emit&& emit) noexcept
        {
            for (std::uint32_t i = 0; i < part_count; ++i)
            {
                const CollisionShape<T> placed =
                    place_compound_part(parts[i], body_center, body_orientation);
                const ManifoldFunction<T> entry =
                    narrowphase_table<T>().get(placed.type, other.type);
                if (entry == nullptr)
                    continue;

                // The routines anchor against the *shape's* centre, which for a part
                // is not the body's. Re-anchor to the body, because the solver applies
                // a lever arm from the centre of mass and a part's own centre is not
                // one.
                ContactManifold<T> manifold = entry(placed, other, contact_offset, face_tolerance);
                if (manifold.point_count == 0)
                    continue;
                for (std::size_t p = 0; p < manifold.point_count; ++p)
                {
                    const Vector3T<T> world_a = to_world_anchor(
                        placed.center, placed.orientation, manifold.points[p].anchor_a_local);
                    manifold.points[p].anchor_a_local =
                        to_local_anchor(body_center, body_orientation, world_a);
                    const Vector3T<T> world_b = to_world_anchor(other.center, other.orientation,
                                                                manifold.points[p].anchor_b_local);
                    manifold.points[p].anchor_b_local =
                        to_local_anchor(other_center, other_orientation, world_b);
                    // Two parts under one body are two contacts; without the part in
                    // the id, warm starting hands one part's impulse to another.
                    manifold.points[p].feature_id ^= (i + 1u) << 20;
                }
                emit(manifold, i);
            }
        }
    } // namespace Physics
} // namespace SushiEngine
