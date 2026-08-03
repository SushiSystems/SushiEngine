/**************************************************************************/
/* mesh_manifold.hpp                                                      */
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
 * @file mesh_manifold.hpp
 * @brief Colliding a convex shape with static triangle-mesh geometry.
 *
 * The geometry is not the hard part — a triangle is convex, so it already
 * collides with everything through `geometry/gjk.hpp`, and the hierarchy in
 * `geometry/mesh_bvh.hpp` names the few triangles worth testing. The hard part
 * is the seams between them, and it has a name: **ghost collisions**.
 *
 * A crate sliding across a tessellated floor is, at every instant, resting on
 * one or two triangles. When it crosses an interior edge, the narrowphase can
 * legitimately report a contact whose normal points *along* that edge rather
 * than out of the floor — a triangle in isolation has an edge, and the shape
 * really is touching it. Resolved, that normal shoves the crate upward and
 * sideways, and the crate visibly trips over a line that is not there. The
 * surface is flat; the seam is an artifact of how it was cut into triangles.
 *
 * The fix needs a fact the triangle does not carry: whether the edge it was
 * caught on is shared with another triangle. `build_triangle_adjacency` cooks
 * exactly that, and §7.2's "adjacent-face normal correction" is then one rule:
 *
 *   **a contact on an edge that has a neighbour is a contact on a continuous
 *   surface, and the only meaningful normal on a continuous surface is a face
 *   normal.**
 *
 * So the normal snaps to the triangle's own face normal, and the patch is built
 * against the triangle's plane. A crease that is genuinely convex loses nothing
 * by this: the neighbouring triangle produces its own contact with its own face
 * normal, and between them the two faces of the crease are both represented.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/convex_manifold.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/geometry/gjk.hpp>
#include <SushiEngine/physics/geometry/mesh_bvh.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Which edge of a triangle a point is nearest, and how near.
         *
         * Used to decide whether a contact came from the triangle's interior — in
         * which case its normal is already the face normal and nothing needs
         * correcting — or from one of its edges.
         *
         * @param triangle The triangle.
         * @param point    A point, generally the witness point on the triangle.
         * @param edge     Receives the edge index: 0 is (a, b), 1 is (b, c), 2 is (c, a).
         * @return The distance from @p point to that edge.
         */
        template <typename T>
        inline T nearest_triangle_edge(const TriangleCollider<T>& triangle,
                                       const Vector3T<T>& point, int& edge) noexcept
        {
            const Vector3T<T> corners[3] = {triangle.a, triangle.b, triangle.c};
            edge = 0;
            T best = T(1e30);
            for (int e = 0; e < 3; ++e)
            {
                const Vector3T<T>& from = corners[e];
                const Vector3T<T>& to = corners[(e + 1) % 3];
                const Vector3T<T> along = to - from;
                const T length_squared = dot(along, along);
                T t = length_squared > T(1e-24) ? dot(point - from, along) / length_squared : T(0);
                t = t < T(0) ? T(0) : (t > T(1) ? T(1) : t);
                const T distance = length(point - (from + along * t));
                if (distance < best)
                {
                    best = distance;
                    edge = e;
                }
            }
            return best;
        }

        /**
         * @brief The manifold between a convex shape and one triangle of a mesh.
         *
         * The shape is body `a` and the mesh is body `b`, so the normal runs from
         * the shape toward the surface and resolving pushes the shape out of it.
         *
         * The triangle is always the **reference face**: it is a flat, bounded
         * polygon whose plane is exactly the surface being rested on, so clipping
         * the shape's contact face against it produces the patch directly and the
         * separations come out measured against the surface rather than against
         * whatever feature GJK happened to converge on.
         *
         * The triangle arrives as a value and its shared edges as three flags,
         * rather than as an index into a mesh. Every surface built out of triangles
         * needs exactly this routine — a triangle mesh, a height field, and the
         * soft-body surfaces of P6 — and they disagree only about where the
         * triangles come from and which of their edges are interior. Handing those
         * in keeps one implementation of the correction rule instead of one per
         * surface kind, each drifting.
         *
         * @param shape               The convex shape.
         * @param triangle            The triangle, in world space.
         * @param shared_edge         Per edge (a-b, b-c, c-a): does the surface
         *                            continue across it?
         * @param center              The shape's body centre, for the anchor frame.
         * @param orientation         The shape's body orientation.
         * @param surface_center      The surface body's centre.
         * @param surface_orientation The surface body's orientation.
         * @param feature_base        Identifies this triangle within its surface, so
         *                            warm starting does not confuse two of them.
         * @param contact_offset      Contacts are generated out to this separation (§7.6).
         * @param face_tolerance      How flush a feature must be to count as touching.
         */
        template <typename T, typename Shape>
        inline ContactManifold<T> generate_convex_triangle_manifold(
            const Shape& shape, const TriangleCollider<T>& triangle, const bool shared_edge[3],
            const Vector3T<T>& center, const QuaternionT<T>& orientation,
            const Vector3T<T>& surface_center, const QuaternionT<T>& surface_orientation,
            std::uint32_t feature_base, T contact_offset = T(0),
            T face_tolerance = T(1e-3)) noexcept
        {
            const Vector3T<T> face_normal = triangle_normal(triangle);
            if (dot(face_normal, face_normal) <= T(0))
                return ContactManifold<T>{}; // a degenerate triangle is not a surface

            const ConvexContact<T> contact = collide_convex<T>(shape, triangle);
            if (!contact.valid || contact.separation > contact_offset)
                return ContactManifold<T>{};

            // Which side of the surface the shape is on decides which way "out" is.
            // A body that has tunnelled must still be pushed out of the side it is
            // on rather than dragged through the wall.
            //
            // The side is taken from the body's centre against the triangle's plane,
            // not from the contact normal. The contact normal is the wrong witness
            // precisely when it matters: a body resting *on* the surface overlaps it
            // by a hair, so GJK's normal comes out of a shallow, nearly-degenerate
            // configuration and can point either way. Its centre, half a body-width
            // above the plane, is not ambiguous at all.
            const T plane_side = dot(face_normal, center) - dot(face_normal, triangle.a);
            const T facing = plane_side >= T(0) ? T(1) : T(-1);
            Vector3T<T> normal = face_normal * (facing * T(-1));

            // The internal-edge correction. A contact whose normal already *is* the
            // face normal came from the triangle's interior and is fine as it stands;
            // one that differs came from an edge or a corner, and if that edge has a
            // neighbour then the surface continues across it and the edge is not a
            // real feature of the world.
            if (dot(contact.normal, normal) <= T(1) - T(1e-6))
            {
                int edge = 0;
                const T edge_distance = nearest_triangle_edge(triangle, contact.point_b, edge);
                const bool on_edge = edge_distance <= face_tolerance * T(10);
                if (!(on_edge && shared_edge[edge]))
                {
                    // A genuine boundary edge or corner. The shape really is caught
                    // on it, GJK's normal is the truth, and there is no patch to
                    // build: an edge is touched at a point. Returning here rather
                    // than falling through matters, because the patch path measures
                    // separations against the triangle's *plane*, which is the right
                    // question only when the normal is the plane's normal. Asked of
                    // an edge contact it answers zero — the shape is beside the
                    // surface, not above it — and a contact that reports no
                    // penetration resolves to nothing.
                    return make_point_manifold(
                        contact.normal, contact.point_a, contact.point_b, contact.separation,
                        center, orientation, surface_center, surface_orientation,
                        make_feature_id(feature_base & 0xFu, (feature_base >> 4) & 0xFu,
                                        static_cast<std::uint32_t>(edge), true));
                }
            }

            // Clip the shape's contact face against the triangle.
            ContactFace<T> reference;
            reference.count = 3;
            reference.points[0].position = triangle.a;
            reference.points[0].vertex_id = 0;
            reference.points[1].position = triangle.b;
            reference.points[1].vertex_id = 1;
            reference.points[2].position = triangle.c;
            reference.points[2].vertex_id = 2;
            reference.normal = face_normal;

            const ContactFace<T> incident = contact_face(shape, normal, face_tolerance);
            ClippedPoint<T> clipped[max_clipped_points];
            std::size_t clipped_count =
                incident.count >= 2
                    ? clip_against_face(incident, reference, face_tolerance, clipped)
                    : 0;
            if (incident.count == 1)
            {
                clipped[0] = incident.points[0];
                clipped_count = 1;
            }
            if (clipped_count == 0)
                return make_point_manifold(normal, contact.point_a, contact.point_b,
                                           contact.separation, center, orientation,
                                           surface_center, surface_orientation,
                                           make_feature_id(0, 0, 0, false));

            const T plane_offset = dot(face_normal, triangle.a);
            ClippedPoint<T> kept[max_clipped_points];
            T separations[max_clipped_points];
            std::size_t kept_count = 0;
            for (std::size_t i = 0; i < clipped_count; ++i)
            {
                const T separation =
                    (dot(face_normal, clipped[i].position) - plane_offset) * facing;
                if (separation > contact_offset)
                    continue;
                kept[kept_count] = clipped[i];
                separations[kept_count] = separation;
                ++kept_count;
            }
            if (kept_count == 0)
                return ContactManifold<T>{};

            std::size_t chosen[max_manifold_points];
            const std::size_t chosen_count =
                reduce_manifold_points(kept, separations, kept_count, chosen, face_tolerance);

            ContactManifold<T> manifold;
            manifold.normal = normal;
            manifold.point_count = static_cast<std::uint8_t>(chosen_count);
            for (std::size_t i = 0; i < chosen_count; ++i)
            {
                const ClippedPoint<T>& point = kept[chosen[i]];
                const T separation = separations[chosen[i]];
                const Vector3T<T> on_surface = point.position + normal * separation;
                ContactPoint<T>& out = manifold.points[i];
                out.anchor_a_local = to_local_anchor(center, orientation, point.position);
                out.anchor_b_local =
                    to_local_anchor(surface_center, surface_orientation, on_surface);
                out.separation = separation;
                out.normal_lambda = T(0);
                out.tangent_lambda[0] = T(0);
                out.tangent_lambda[1] = T(0);
                // The triangle index is part of the identity: two triangles under the
                // same shape are two contacts, and warm starting must not confuse them.
                out.feature_id = make_feature_id(feature_base & 0xFu, (feature_base >> 4) & 0xFu,
                                                 point.vertex_id, false);
            }
            return manifold;
        }

        /**
         * @brief Every manifold between a convex shape and the triangles it touches.
         *
         * Queries the hierarchy with the shape's world bounds, grown by the contact
         * offset so a speculative contact finds its triangle before it needs it,
         * then emits one manifold per triangle that produces contact. One manifold
         * per triangle rather than one merged manifold, deliberately: a shape lying
         * across a ridge touches two surfaces with two different normals, and
         * flattening them into one normal is how a body sinks into a valley.
         *
         * Triangles arrive in cooked order, which is a function of the mesh alone,
         * so the manifolds are emitted in the same order on every run — the
         * determinism §12.1 asks of a narrowphase input.
         *
         * @param emit Called as `emit(manifold, triangle_index)`.
         */
        template <typename T, typename Shape, typename Emit>
        inline void generate_convex_mesh_manifolds(const Shape& shape,
                                                   const TriangleMeshView<T>& mesh,
                                                   const Vector3T<T>& center,
                                                   const QuaternionT<T>& orientation,
                                                   T contact_offset, T face_tolerance,
                                                   Emit&& emit) noexcept
        {
            const Aabb<T> bounds = aabb_expand(world_bounds(shape), contact_offset);
            query_mesh_bvh(mesh, bounds,
                           [&](std::uint32_t triangle_index) noexcept
                           {
                               const bool shared[3] = {
                                   mesh.adjacency != nullptr &&
                                       mesh.adjacency[3u * triangle_index] != no_adjacent_triangle,
                                   mesh.adjacency != nullptr &&
                                       mesh.adjacency[3u * triangle_index + 1u] !=
                                           no_adjacent_triangle,
                                   mesh.adjacency != nullptr &&
                                       mesh.adjacency[3u * triangle_index + 2u] !=
                                           no_adjacent_triangle};
                               const ContactManifold<T> manifold =
                                   generate_convex_triangle_manifold<T>(
                                       shape, mesh_triangle(mesh, triangle_index), shared, center,
                                       orientation, mesh.center, mesh.orientation, triangle_index,
                                       contact_offset, face_tolerance);
                               if (manifold.point_count > 0)
                                   emit(manifold, triangle_index);
                           });
        }
    } // namespace Physics
} // namespace SushiEngine
