/**************************************************************************/
/* manifold.hpp                                                           */
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
 * @file manifold.hpp
 * @brief Contact manifolds: the several points at which two shapes touch, not one.
 *
 * A single contact point per pair is enough to push two shapes apart and not
 * enough for either of them to *rest*. A box lying on the ground touches it over
 * a square, and a solver handed one point of that square holds the box by one
 * corner at a time: it rocks, it never settles, and a stack of them never
 * converges. That is §1.2 item 3 of `docs/slop/physics_system.md`, and this file
 * is where it is fixed.
 *
 * The manifold is produced by **face clipping** (§7.3): the separating-axis test
 * names a reference face on one shape and the most anti-parallel *incident* face
 * on the other, the incident face polygon is clipped against the reference
 * face's side planes (Sutherland–Hodgman), and the survivors are reduced to at
 * most four points by maximizing the area they enclose. Two properties of the
 * result matter as much as the points themselves:
 *
 * - **Points are stored as a pair of body-local anchors, not as a world point.**
 *   A world point is stale the moment either body moves; two anchors are not.
 *   The separation is *derived* — `dot(anchor_b_world - anchor_a_world, normal)`
 *   — so a manifold generated once per tick (§6.1) can be refreshed for free
 *   every substep instead of being regenerated, which is what makes 32 substeps
 *   affordable.
 * - **Every point carries a `feature_id`**, naming the pair of geometric features
 *   that produced it. Matching this tick's points to last tick's by that id is
 *   what lets the accumulated Lagrange multipliers carry over — warm starting,
 *   without which a ten-crate stack needs iterations it is not going to get.
 *
 * Everything here is pure geometry over shape value types: no bodies, no solver,
 * no runtime. The handles and material indices on `ContactManifold` are filled by
 * whoever owns the bodies; the generation functions never touch them.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/handle.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief The most points a manifold keeps, after reduction (§7.3). */
        inline constexpr std::size_t max_manifold_points = 4;

        /** @brief The most points face clipping can produce before reduction. */
        inline constexpr std::size_t max_clipped_points = 8;

        /**
         * @brief One point of contact, expressed so it survives the bodies moving.
         *
         * `anchor_a_local` and `anchor_b_local` are the touching point on each
         * shape's surface, in that shape's own frame. They coincide in world space
         * when the surfaces just touch, and separate along @ref
         * ContactManifold::normal as the shapes part or overlap — which is why the
         * separation is a derived quantity (@ref contact_point_separation) rather
         * than a stored one, and why refreshing a manifold costs two rotations per
         * point instead of a fresh narrowphase.
         *
         * The three `lambda` accumulators are the solver's, not the narrowphase's:
         * they are zero on a newly generated point and inherited from the previous
         * tick's matching point by @ref warm_start_manifold.
         */
        template <typename T>
        struct ContactPoint
        {
            Vector3T<T> anchor_a_local{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> anchor_b_local{Vector3T<T>{T(0), T(0), T(0)}};
            /** @brief Separation at generation time; negative means penetrating. */
            T separation = 0;
            /** @brief Accumulated normal impulse, carried across ticks by warm starting. */
            T normal_lambda = 0;
            /** @brief Accumulated friction impulse along the two tangent directions. */
            T tangent_lambda[2] = {0, 0};
            /** @brief Which pair of features produced this point (@ref make_feature_id). */
            std::uint32_t feature_id = 0;
        };

        /**
         * @brief Everything two shapes' contact is: one normal and up to four points.
         *
         * One normal for the whole manifold is the deliberate simplification the
         * clipping model buys: every point of a face-face contact shares the
         * reference face's normal, so per-point normals would be the same number
         * stored four times. A curved contact that genuinely needs several normals
         * is several manifolds.
         *
         * Convention, inherited from `contact.hpp` and unchanged: `normal` is unit
         * length and points from shape @ref a toward shape @ref b, so resolving
         * moves @ref a along `-normal` and @ref b along `+normal`.
         */
        template <typename T>
        struct ContactManifold
        {
            BodyHandle a;               /**< The first body; filled by the caller. */
            BodyHandle b;               /**< The second body; filled by the caller. */
            Vector3T<T> normal{Vector3T<T>{T(0), T(1), T(0)}};
            ContactPoint<T> points[max_manifold_points];
            std::uint8_t point_count = 0;
            std::uint16_t material_a = 0; /**< Index into the scene's material table. */
            std::uint16_t material_b = 0;
        };

        /**
         * @brief Packs the features that produced a contact point into a stable id.
         *
         * Stable is the whole requirement: the same two faces touching at the same
         * corner must hash to the same number this tick as last, and to a different
         * number from any other corner. The three fields are exactly what the
         * clipper knows — which face of each shape met, and which vertex or clip
         * edge of the incident polygon the point came from — and 4/4/8 bits hold
         * them with room to spare for shapes with more faces than a box.
         *
         * @param reference_face Face index on the reference shape (0..15).
         * @param incident_face  Face index on the incident shape (0..15).
         * @param vertex         Which incident vertex (0..3) or clip plane (4..7)
         *                       produced the point.
         * @param flipped        Whether the reference shape was the manifold's `b`
         *                       rather than its `a`; without it, a pair whose
         *                       reference face swaps between ticks would silently
         *                       match points that are not the same contact.
         */
        inline std::uint32_t make_feature_id(std::uint32_t reference_face,
                                             std::uint32_t incident_face, std::uint32_t vertex,
                                             bool flipped) noexcept
        {
            return (reference_face & 0xFu) | ((incident_face & 0xFu) << 4) |
                   ((vertex & 0xFFu) << 8) | (flipped ? 0x10000u : 0u);
        }

        /** @brief Rotates a world-space point into a shape's local frame. */
        template <typename T>
        inline Vector3T<T> to_local_anchor(const Vector3T<T>& center,
                                           const QuaternionT<T>& orientation,
                                           const Vector3T<T>& world_point) noexcept
        {
            return rotate(conjugate(orientation), world_point - center);
        }

        /** @brief Rotates a local anchor back out into world space. */
        template <typename T>
        inline Vector3T<T> to_world_anchor(const Vector3T<T>& center,
                                           const QuaternionT<T>& orientation,
                                           const Vector3T<T>& local_anchor) noexcept
        {
            return center + rotate(orientation, local_anchor);
        }

        /**
         * @brief The live separation of one contact point, from the two bodies' poses.
         *
         * The reason anchors are stored instead of a world point: this is a valid
         * answer at any later time, for any later pose, without asking the
         * narrowphase anything. Negative is penetrating.
         */
        template <typename T>
        inline T contact_point_separation(const ContactPoint<T>& point,
                                          const Vector3T<T>& normal,
                                          const Vector3T<T>& center_a,
                                          const QuaternionT<T>& orientation_a,
                                          const Vector3T<T>& center_b,
                                          const QuaternionT<T>& orientation_b) noexcept
        {
            const Vector3T<T> world_a = to_world_anchor(center_a, orientation_a, point.anchor_a_local);
            const Vector3T<T> world_b = to_world_anchor(center_b, orientation_b, point.anchor_b_local);
            return dot(world_b - world_a, normal);
        }

        /**
         * @brief Re-derives every point's separation from the current poses.
         *
         * The per-substep half of §6.1's "detect once per tick": the features that
         * touch do not change over a substep, only how deeply, so the manifold is
         * refreshed rather than rebuilt. The normal is held fixed for the tick,
         * which is the standard approximation and the reason the tick's substeps
         * stay cheap.
         */
        template <typename T>
        inline void refresh_manifold(ContactManifold<T>& manifold, const Vector3T<T>& center_a,
                                     const QuaternionT<T>& orientation_a,
                                     const Vector3T<T>& center_b,
                                     const QuaternionT<T>& orientation_b) noexcept
        {
            for (std::size_t i = 0; i < manifold.point_count; ++i)
                manifold.points[i].separation =
                    contact_point_separation(manifold.points[i], manifold.normal, center_a,
                                             orientation_a, center_b, orientation_b);
        }

        /**
         * @brief Carries the previous tick's impulses onto this tick's matching points.
         *
         * Warm starting, and the single cheapest convergence win in the phase: a
         * settled stack's contacts already know what force holds them up, so the
         * solver starts from last tick's answer instead of from zero. Points are
         * matched by @ref ContactPoint::feature_id — by *which corner touches
         * which face*, not by array position, because clipping is free to emit the
         * same four corners in a different order as a box rotates.
         *
         * A point with no match keeps its zero accumulators: a genuinely new
         * contact has no history to inherit, and inventing one is how a stack gets
         * a kick on the tick a crate lands on it.
         */
        template <typename T>
        inline void warm_start_manifold(ContactManifold<T>& current,
                                        const ContactManifold<T>& previous) noexcept
        {
            for (std::size_t i = 0; i < current.point_count; ++i)
                for (std::size_t j = 0; j < previous.point_count; ++j)
                {
                    if (current.points[i].feature_id != previous.points[j].feature_id)
                        continue;
                    current.points[i].normal_lambda = previous.points[j].normal_lambda;
                    current.points[i].tangent_lambda[0] = previous.points[j].tangent_lambda[0];
                    current.points[i].tangent_lambda[1] = previous.points[j].tangent_lambda[1];
                    break;
                }
        }

        /**
         * @brief One candidate contact point, mid-clip: a world position and its origin.
         *
         * The clipper works in world space over these rather than over finished
         * `ContactPoint`s, because a point that is about to be discarded should not
         * pay for two frame conversions.
         */
        template <typename T>
        struct ClippedPoint
        {
            Vector3T<T> position{Vector3T<T>{T(0), T(0), T(0)}};
            std::uint32_t vertex_id = 0;
        };

        /**
         * @brief Clips a polygon against one half-space, Sutherland–Hodgman.
         *
         * Keeps the part of the polygon on the inside of the plane
         * (`dot(normal, p) <= offset`) and inserts an intersection vertex wherever
         * an edge crosses it. A vertex that survives keeps its id; a vertex created
         * on the boundary takes @p crossing_id, so a point that is defined by a
         * clip edge is distinguishable from one that is a real corner and both stay
         * stable while the boxes stay in the same relative pose.
         *
         * @param input      The polygon, in order.
         * @param count      How many vertices @p input holds.
         * @param normal     Outward normal of the clipping plane.
         * @param offset     Plane offset along @p normal.
         * @param crossing_id Vertex id to give points created on the plane.
         * @param output     Receives the clipped polygon; must hold @ref
         *                   max_clipped_points vertices.
         * @return The number of vertices written to @p output.
         */
        template <typename T>
        inline std::size_t clip_polygon_against_plane(const ClippedPoint<T>* input,
                                                      std::size_t count,
                                                      const Vector3T<T>& normal, T offset,
                                                      std::uint32_t crossing_id,
                                                      ClippedPoint<T>* output) noexcept
        {
            std::size_t written = 0;
            if (count == 0)
                return 0;

            Vector3T<T> previous = input[count - 1].position;
            T previous_distance = dot(normal, previous) - offset;

            for (std::size_t i = 0; i < count && written < max_clipped_points; ++i)
            {
                const Vector3T<T> current = input[i].position;
                const std::uint32_t current_id = input[i].vertex_id;
                const T current_distance = dot(normal, current) - offset;

                // The edge crosses the plane: emit the crossing point first, so the
                // output polygon stays in order and stays convex.
                if ((previous_distance > T(0)) != (current_distance > T(0)))
                {
                    const T denominator = previous_distance - current_distance;
                    const T t = std::abs(denominator) > T(1e-12)
                                    ? previous_distance / denominator
                                    : T(0);
                    output[written].position = previous + (current - previous) * t;
                    output[written].vertex_id = crossing_id;
                    ++written;
                }
                if (current_distance <= T(0) && written < max_clipped_points)
                {
                    output[written].position = current;
                    output[written].vertex_id = current_id;
                    ++written;
                }

                previous = current;
                previous_distance = current_distance;
            }
            return written;
        }

        /**
         * @brief Picks at most four of the clipped points, maximizing the area they span.
         *
         * Which four is not a detail. Four points clustered along one edge hold a
         * box no better than one point does; four points spread to the corners of
         * the contact patch hold it rigidly. So the reduction is greedy and
         * geometric: the deepest point (it is the one that must be resolved), the
         * point furthest from it (the longest diagonal), the point furthest off
         * that line (the widest triangle), and the point furthest off that triangle
         * (the widest quad).
         *
         * Every tie is broken by the lower index, so the same input always yields
         * the same four points in the same order — §0.5's determinism rule reaching
         * all the way down into the narrowphase.
         *
         * @param candidates  The clipped points.
         * @param separations Each candidate's separation; the deepest is negative-most.
         * @param count       How many candidates there are.
         * @param kept        Receives the indices of the chosen points.
         * @return How many indices were written to @p kept (at most four).
         */
        template <typename T>
        inline std::size_t reduce_manifold_points(const ClippedPoint<T>* candidates,
                                                  const T* separations, std::size_t count,
                                                  std::size_t* kept) noexcept
        {
            if (count <= max_manifold_points)
            {
                for (std::size_t i = 0; i < count; ++i)
                    kept[i] = i;
                return count;
            }

            // 1. The deepest point. It is the one the solver most has to fix, so it
            //    is never the point that gets dropped.
            std::size_t deepest = 0;
            for (std::size_t i = 1; i < count; ++i)
                if (separations[i] < separations[deepest])
                    deepest = i;
            kept[0] = deepest;

            // 2. The point furthest from it: the patch's longest diagonal.
            std::size_t furthest = 0;
            T best = T(-1);
            for (std::size_t i = 0; i < count; ++i)
            {
                if (i == deepest)
                    continue;
                const Vector3T<T> delta = candidates[i].position - candidates[deepest].position;
                const T distance_squared = dot(delta, delta);
                if (distance_squared > best)
                {
                    best = distance_squared;
                    furthest = i;
                }
            }
            kept[1] = furthest;

            // 3. The point furthest off the line through the first two — the widest
            //    triangle, measured by the area of the cross product rather than by a
            //    perpendicular distance, so a degenerate line costs no division.
            const Vector3T<T> edge = candidates[furthest].position - candidates[deepest].position;
            std::size_t widest = 0;
            best = T(-1);
            for (std::size_t i = 0; i < count; ++i)
            {
                if (i == deepest || i == furthest)
                    continue;
                const Vector3T<T> arm = candidates[i].position - candidates[deepest].position;
                const Vector3T<T> area = cross(edge, arm);
                const T magnitude = dot(area, area);
                if (magnitude > best)
                {
                    best = magnitude;
                    widest = i;
                }
            }
            kept[2] = widest;

            // 4. The point that most enlarges the triangle into a quad: the largest
            //    triangle it forms with any one of the three edges already chosen.
            std::size_t fourth = 0;
            best = T(-1);
            for (std::size_t i = 0; i < count; ++i)
            {
                if (i == deepest || i == furthest || i == widest)
                    continue;
                T largest = T(0);
                for (std::size_t e = 0; e < 3; ++e)
                {
                    const Vector3T<T>& from = candidates[kept[e]].position;
                    const Vector3T<T>& to = candidates[kept[(e + 1) % 3]].position;
                    const Vector3T<T> area =
                        cross(to - from, candidates[i].position - from);
                    const T magnitude = dot(area, area);
                    if (magnitude > largest)
                        largest = magnitude;
                }
                if (largest > best)
                {
                    best = largest;
                    fourth = i;
                }
            }
            kept[3] = fourth;
            return max_manifold_points;
        }

        /**
         * @brief The corners of one face of an oriented box, in winding order.
         *
         * @param box    The box.
         * @param axis   Which local axis the face is normal to (0..2).
         * @param sign   +1 for the face on the axis's positive side, -1 for the other.
         * @param corners Receives the four world-space corners.
         */
        template <typename T>
        inline void obb_face_corners(const OrientedBox<T>& box, int axis, T sign,
                                     Vector3T<T> corners[4]) noexcept
        {
            Vector3T<T> axes[3];
            obb_axes(box, axes);
            const T extents[3] = {box.half_extents.x, box.half_extents.y, box.half_extents.z};
            const int u = (axis + 1) % 3;
            const int v = (axis + 2) % 3;
            const Vector3T<T> center = box.center + axes[axis] * (extents[axis] * sign);
            const Vector3T<T> du = axes[u] * extents[u];
            const Vector3T<T> dv = axes[v] * extents[v];
            corners[0] = center - du - dv;
            corners[1] = center + du - dv;
            corners[2] = center + du + dv;
            corners[3] = center - du + dv;
        }

        /** @brief Face index in 0..5 from a local axis and a side. */
        inline std::uint32_t obb_face_index(int axis, bool positive) noexcept
        {
            return static_cast<std::uint32_t>(axis * 2 + (positive ? 0 : 1));
        }

        /**
         * @brief Which face of @p box points most nearly opposite @p direction.
         *
         * The incident face: the face that is actually facing the other shape, and
         * therefore the polygon worth clipping. "Most anti-parallel" is the whole
         * definition, and picking it by the largest negative dot product is both the
         * cheapest and the standard way to find it.
         *
         * @param box       The incident box.
         * @param direction The contact normal, pointing from the reference shape
         *                  toward this one.
         * @param axis      Receives the local axis of the face (0..2).
         * @param sign      Receives +1 or -1 for the side.
         */
        template <typename T>
        inline void obb_incident_face(const OrientedBox<T>& box, const Vector3T<T>& direction,
                                      int& axis, T& sign) noexcept
        {
            Vector3T<T> axes[3];
            obb_axes(box, axes);
            axis = 0;
            sign = T(1);
            T best = T(1e30);
            for (int i = 0; i < 3; ++i)
                for (int s = 0; s < 2; ++s)
                {
                    const T candidate_sign = s == 0 ? T(1) : T(-1);
                    const T projection = dot(axes[i] * candidate_sign, direction);
                    if (projection < best)
                    {
                        best = projection;
                        axis = i;
                        sign = candidate_sign;
                    }
                }
        }

        /**
         * @brief Builds a one-point manifold from a normal, two surface points, and a separation.
         *
         * The shared tail of every analytic narrowphase pair — sphere-sphere,
         * sphere-box, and the edge-edge case of box-box all end here — so the
         * anchor convention is written once rather than four times.
         */
        template <typename T>
        inline ContactManifold<T> make_point_manifold(const Vector3T<T>& normal,
                                                      const Vector3T<T>& point_on_a,
                                                      const Vector3T<T>& point_on_b, T separation,
                                                      const Vector3T<T>& center_a,
                                                      const QuaternionT<T>& orientation_a,
                                                      const Vector3T<T>& center_b,
                                                      const QuaternionT<T>& orientation_b,
                                                      std::uint32_t feature_id) noexcept
        {
            ContactManifold<T> manifold;
            manifold.normal = normal;
            manifold.point_count = 1;
            manifold.points[0].anchor_a_local = to_local_anchor(center_a, orientation_a, point_on_a);
            manifold.points[0].anchor_b_local = to_local_anchor(center_b, orientation_b, point_on_b);
            manifold.points[0].separation = separation;
            manifold.points[0].feature_id = feature_id;
            return manifold;
        }

        /**
         * @brief Manifold between two spheres: one point, and one point is correct.
         *
         * Two spheres touch at a single point however deep they are, so there is no
         * patch to clip and nothing a second point would add. Generated out to
         * @p contact_offset so an approaching pair is constrained before it
         * overlaps (§7.5's speculative contacts), which is why the separation is
         * allowed to be positive.
         */
        template <typename T>
        inline ContactManifold<T> generate_sphere_sphere_manifold(const SphereCollider<T>& a,
                                                                  const SphereCollider<T>& b,
                                                                  T contact_offset = T(0)) noexcept
        {
            const QuaternionT<T> identity{T(0), T(0), T(0), T(1)};
            const Vector3T<T> delta = b.center - a.center;
            const T distance = length(delta);
            const T separation = distance - (a.radius + b.radius);
            if (separation > contact_offset)
                return ContactManifold<T>{};
            const Vector3T<T> normal =
                distance > T(1e-8) ? delta * (T(1) / distance) : Vector3T<T>{T(0), T(1), T(0)};
            return make_point_manifold(normal, a.center + normal * a.radius,
                                       b.center - normal * b.radius, separation, a.center,
                                       identity, b.center, identity, make_feature_id(0, 0, 0, false));
        }

        /**
         * @brief Manifold between an oriented box (a) and a sphere (b): one point.
         *
         * The sphere contributes a single touching point by construction, so this
         * is the closest-point routine of `narrowphase.hpp` re-expressed in anchors
         * — including its inside-the-box case, where no closest-surface direction
         * exists and the nearest face supplies one.
         */
        template <typename T>
        inline ContactManifold<T> generate_obb_sphere_manifold(const OrientedBox<T>& box,
                                                               const SphereCollider<T>& sphere,
                                                               T contact_offset = T(0)) noexcept
        {
            const auto clamp = [](T value, T low, T high) noexcept
            { return value < low ? low : (value > high ? high : value); };
            const QuaternionT<T> identity{T(0), T(0), T(0), T(1)};

            const Vector3T<T> local = rotate(conjugate(box.orientation), sphere.center - box.center);
            const Vector3T<T> closest{clamp(local.x, -box.half_extents.x, box.half_extents.x),
                                      clamp(local.y, -box.half_extents.y, box.half_extents.y),
                                      clamp(local.z, -box.half_extents.z, box.half_extents.z)};
            const Vector3T<T> delta = local - closest;
            const T distance = length(delta);

            Vector3T<T> normal_local;
            T separation;
            if (distance <= T(1e-8))
            {
                normal_local = nearest_face_normal(local, box.half_extents);
                separation = -(sphere.radius + nearest_face_distance(local, box.half_extents));
            }
            else
            {
                normal_local = delta * (T(1) / distance);
                separation = distance - sphere.radius;
                if (separation > contact_offset)
                    return ContactManifold<T>{};
            }

            const Vector3T<T> normal = rotate(box.orientation, normal_local);
            const Vector3T<T> point_on_box =
                distance <= T(1e-8) ? sphere.center : box.center + rotate(box.orientation, closest);
            return make_point_manifold(normal, point_on_box, sphere.center - normal * sphere.radius,
                                       separation, box.center, box.orientation, sphere.center,
                                       identity, make_feature_id(0, 0, 0, false));
        }

        /**
         * @brief Manifold between an oriented box (a) and a static half-space plane (b).
         *
         * The plane is the reference face and it has no side planes, so every corner
         * of the box's incident face survives and the manifold is that face — up to
         * four points, which is exactly what stops a landed box from rocking on one
         * corner. The plane is treated as body `b`, so the normal points from the box
         * toward the plane (`-plane.normal`) and resolving moves the box out along
         * `+plane.normal`, matching the pair convention rather than making the plane
         * a special case with its own sign rule.
         *
         * The plane's anchors are world points: a static half-space has no frame to
         * be local to, and identity is the frame the caller should pass for it.
         */
        template <typename T>
        inline ContactManifold<T> generate_obb_plane_manifold(const OrientedBox<T>& box,
                                                              const PlaneCollider<T>& plane,
                                                              T contact_offset = T(0)) noexcept
        {
            int axis = 0;
            T sign = T(1);
            // The plane is the reference, and it faces the box along +plane.normal;
            // the incident face is the box face pointing most nearly back into it.
            obb_incident_face(box, plane.normal, axis, sign);

            Vector3T<T> corners[4];
            obb_face_corners(box, axis, sign, corners);

            ContactManifold<T> manifold;
            manifold.normal = plane.normal * T(-1);
            const std::uint32_t incident_face = obb_face_index(axis, sign > T(0));

            for (std::uint32_t i = 0; i < 4; ++i)
            {
                const T separation = dot(plane.normal, corners[i]) - plane.offset;
                if (separation > contact_offset)
                    continue;
                ContactPoint<T>& point = manifold.points[manifold.point_count];
                point.anchor_a_local = to_local_anchor(box.center, box.orientation, corners[i]);
                point.anchor_b_local = corners[i] - plane.normal * separation;
                point.separation = separation;
                point.normal_lambda = T(0);
                point.tangent_lambda[0] = T(0);
                point.tangent_lambda[1] = T(0);
                point.feature_id = make_feature_id(0, incident_face, i, false);
                ++manifold.point_count;
            }
            return manifold;
        }

        /**
         * @brief The two closest points of two (infinite) lines, clamped to segments.
         *
         * The edge-edge case of box-box needs this and nothing else needs it, so it
         * lives next to its caller. Parallel edges have no unique answer; the
         * degenerate branch returns the segment midpoints, which is the stable
         * choice rather than a division by something near zero.
         */
        template <typename T>
        inline void closest_points_on_segments(const Vector3T<T>& center_a,
                                               const Vector3T<T>& direction_a, T half_length_a,
                                               const Vector3T<T>& center_b,
                                               const Vector3T<T>& direction_b, T half_length_b,
                                               Vector3T<T>& point_a, Vector3T<T>& point_b) noexcept
        {
            const auto clamp = [](T value, T limit) noexcept
            { return value < -limit ? -limit : (value > limit ? limit : value); };

            const Vector3T<T> delta = center_b - center_a;
            const T dd = dot(direction_a, direction_b);
            const T denominator = T(1) - dd * dd;
            T ta = T(0);
            T tb = T(0);
            if (std::abs(denominator) > T(1e-9))
            {
                const T da = dot(delta, direction_a);
                const T db = dot(delta, direction_b);
                ta = (da - db * dd) / denominator;
                tb = (da * dd - db) / denominator;
            }
            ta = clamp(ta, half_length_a);
            tb = clamp(tb, half_length_b);
            point_a = center_a + direction_a * ta;
            point_b = center_b + direction_b * tb;
        }

        /**
         * @brief Manifold between two oriented boxes, by separating axis and face clipping.
         *
         * The routine §7.3 describes, in three parts:
         *
         * 1. **Separating-axis test** over the 15 candidate axes, exactly as
         *    `collide_obb_obb` does — but remembering *which kind* of axis won, not
         *    only how deep it was. Face axes are given a small bias over edge axes:
         *    when a box rests flat, the face axis and several edge axes are within
         *    floating-point noise of each other, and without the bias the winner
         *    flickers between them, which flickers the feature ids, which throws
         *    away the warm start on the tick it is most needed.
         * 2. **Edge-edge** contacts are genuinely one point — two skew edges cross at
         *    one place — so they take the closest-points path and stop there.
         * 3. **Face** contacts clip the incident face against the reference face's
         *    four side planes and reduce the survivors to four.
         *
         * Contacts are generated out to @p contact_offset, so a pair that is close
         * but not yet touching still produces a manifold with a positive separation.
         * The solver treats that as a constraint not to *become* penetrating, which
         * is the speculative-contact half of §7.5 and costs nothing extra here.
         *
         * @return A manifold with `point_count == 0` when the boxes are further
         *         apart than @p contact_offset.
         */
        template <typename T>
        inline ContactManifold<T> generate_obb_obb_manifold(const OrientedBox<T>& a,
                                                            const OrientedBox<T>& b,
                                                            T contact_offset = T(0)) noexcept
        {
            Vector3T<T> axis_a[3];
            Vector3T<T> axis_b[3];
            obb_axes(a, axis_a);
            obb_axes(b, axis_b);
            const T extent_a[3] = {a.half_extents.x, a.half_extents.y, a.half_extents.z};
            const T extent_b[3] = {b.half_extents.x, b.half_extents.y, b.half_extents.z};
            const Vector3T<T> center_delta = b.center - a.center;

            // Face axes win ties against edge axes by this margin. Small enough not
            // to choose a genuinely worse axis, large enough to swallow the noise in
            // a cross product of two nearly-parallel axes.
            const T face_bias = T(1e-5);

            T best_score = T(-1e30);      // the biased value the axes are ranked by
            T best_separation = T(-1e30); // the winner's true separation
            int best_kind = -1;           // 0 = face of a, 1 = face of b, 2 = edge pair
            int best_index = 0;
            Vector3T<T> best_axis{T(0), T(1), T(0)};

            const auto test_axis = [&](const Vector3T<T>& raw, int kind, int index,
                                       T bias) noexcept -> bool
            {
                const T length_squared = dot(raw, raw);
                if (length_squared < T(1e-12))
                    return true; // parallel edges: not a separating axis
                const Vector3T<T> axis = raw * (T(1) / std::sqrt(length_squared));
                const T radius_a = std::abs(dot(axis_a[0], axis)) * extent_a[0] +
                                   std::abs(dot(axis_a[1], axis)) * extent_a[1] +
                                   std::abs(dot(axis_a[2], axis)) * extent_a[2];
                const T radius_b = std::abs(dot(axis_b[0], axis)) * extent_b[0] +
                                   std::abs(dot(axis_b[1], axis)) * extent_b[1] +
                                   std::abs(dot(axis_b[2], axis)) * extent_b[2];
                const T separation =
                    std::abs(dot(center_delta, axis)) - (radius_a + radius_b);
                if (separation > contact_offset)
                    return false; // a genuine gap: the boxes do not touch
                if (separation + bias > best_score)
                {
                    best_score = separation + bias;
                    best_separation = separation;
                    best_kind = kind;
                    best_index = index;
                    best_axis = axis;
                }
                return true;
            };

            for (int i = 0; i < 3; ++i)
                if (!test_axis(axis_a[i], 0, i, face_bias))
                    return ContactManifold<T>{};
            for (int i = 0; i < 3; ++i)
                if (!test_axis(axis_b[i], 1, i, face_bias))
                    return ContactManifold<T>{};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    if (!test_axis(cross(axis_a[i], axis_b[j]), 2, i * 3 + j, T(0)))
                        return ContactManifold<T>{};

            if (best_kind < 0)
                return ContactManifold<T>{};

            // Orient the winning axis from a toward b, so the manifold obeys the
            // pair convention regardless of which side the axis happened to face.
            const Vector3T<T> normal =
                dot(best_axis, center_delta) < T(0) ? best_axis * T(-1) : best_axis;

            if (best_kind == 2)
            {
                // Edge-edge: the two edges most extreme along the normal, crossing at
                // one point. Which edges is decided by the support direction on the
                // other two axes of each box.
                const int i = best_index / 3;
                const int j = best_index % 3;
                Vector3T<T> center_edge_a = a.center;
                Vector3T<T> center_edge_b = b.center;
                for (int k = 0; k < 3; ++k)
                {
                    if (k == i)
                        continue;
                    const T sign = dot(axis_a[k], normal) < T(0) ? T(-1) : T(1);
                    center_edge_a = center_edge_a + axis_a[k] * (extent_a[k] * sign);
                }
                for (int k = 0; k < 3; ++k)
                {
                    if (k == j)
                        continue;
                    const T sign = dot(axis_b[k], normal) < T(0) ? T(1) : T(-1);
                    center_edge_b = center_edge_b + axis_b[k] * (extent_b[k] * sign);
                }
                Vector3T<T> point_a;
                Vector3T<T> point_b;
                closest_points_on_segments(center_edge_a, axis_a[i], extent_a[i], center_edge_b,
                                           axis_b[j], extent_b[j], point_a, point_b);
                return make_point_manifold(normal, point_a, point_b,
                                           best_separation, a.center, a.orientation,
                                           b.center, b.orientation,
                                           make_feature_id(static_cast<std::uint32_t>(i),
                                                           static_cast<std::uint32_t>(j), 8, false));
            }

            // Face contact. The reference box owns the winning face; the other box
            // supplies the polygon that gets clipped.
            const bool reference_is_b = best_kind == 1;
            const OrientedBox<T>& reference = reference_is_b ? b : a;
            const OrientedBox<T>& incident = reference_is_b ? a : b;
            const Vector3T<T>* reference_axes = reference_is_b ? axis_b : axis_a;
            const T* reference_extents = reference_is_b ? extent_b : extent_a;
            // The reference face normal points from the reference box toward the other.
            const Vector3T<T> reference_normal = reference_is_b ? normal * T(-1) : normal;
            const int reference_axis = best_index;
            const T reference_sign =
                dot(reference_axes[reference_axis], reference_normal) < T(0) ? T(-1) : T(1);

            int incident_axis = 0;
            T incident_sign = T(1);
            obb_incident_face(incident, reference_normal, incident_axis, incident_sign);

            Vector3T<T> incident_corners[4];
            obb_face_corners(incident, incident_axis, incident_sign, incident_corners);

            ClippedPoint<T> buffer_a[max_clipped_points];
            ClippedPoint<T> buffer_b[max_clipped_points];
            for (std::uint32_t i = 0; i < 4; ++i)
            {
                buffer_a[i].position = incident_corners[i];
                buffer_a[i].vertex_id = i;
            }
            std::size_t count = 4;

            // Clip against the four side planes of the reference face: the two axes
            // that are not the face normal, on both sides. The two buffers ping-pong,
            // so no allocation and no copy back.
            ClippedPoint<T>* source = buffer_a;
            ClippedPoint<T>* destination = buffer_b;
            std::uint32_t crossing_id = 4;
            for (int k = 1; k <= 2; ++k)
            {
                const int side_axis = (reference_axis + k) % 3;
                for (int s = 0; s < 2; ++s)
                {
                    const T plane_sign = s == 0 ? T(1) : T(-1);
                    const Vector3T<T> plane_normal = reference_axes[side_axis] * plane_sign;
                    const T plane_offset = dot(plane_normal, reference.center) +
                                           reference_extents[side_axis];
                    count = clip_polygon_against_plane(source, count, plane_normal, plane_offset,
                                                       crossing_id, destination);
                    ClippedPoint<T>* const swapped = source;
                    source = destination;
                    destination = swapped;
                    ++crossing_id;
                    if (count == 0)
                        return ContactManifold<T>{};
                }
            }

            // Keep only what is at or under the reference face, within the offset.
            const Vector3T<T> reference_face_point =
                reference.center +
                reference_axes[reference_axis] * (reference_extents[reference_axis] * reference_sign);
            const T reference_plane_offset = dot(reference_normal, reference_face_point);

            ClippedPoint<T> kept_points[max_clipped_points];
            T separations[max_clipped_points];
            std::size_t kept_count = 0;
            for (std::size_t i = 0; i < count; ++i)
            {
                const T separation = dot(reference_normal, source[i].position) - reference_plane_offset;
                if (separation > contact_offset)
                    continue;
                kept_points[kept_count] = source[i];
                separations[kept_count] = separation;
                ++kept_count;
            }
            if (kept_count == 0)
                return ContactManifold<T>{};

            std::size_t chosen[max_manifold_points];
            const std::size_t chosen_count =
                reduce_manifold_points(kept_points, separations, kept_count, chosen);

            const std::uint32_t reference_face =
                obb_face_index(reference_axis, reference_sign > T(0));
            const std::uint32_t incident_face =
                obb_face_index(incident_axis, incident_sign > T(0));

            ContactManifold<T> manifold;
            manifold.normal = normal;
            manifold.point_count = static_cast<std::uint8_t>(chosen_count);
            for (std::size_t i = 0; i < chosen_count; ++i)
            {
                const ClippedPoint<T>& candidate = kept_points[chosen[i]];
                const T separation = separations[chosen[i]];
                // The clipped point lies on the incident face; its projection onto the
                // reference plane is where the reference box is touched.
                const Vector3T<T> on_reference =
                    candidate.position - reference_normal * separation;
                const Vector3T<T>& on_a = reference_is_b ? candidate.position : on_reference;
                const Vector3T<T>& on_b = reference_is_b ? on_reference : candidate.position;

                ContactPoint<T>& point = manifold.points[i];
                point.anchor_a_local = to_local_anchor(a.center, a.orientation, on_a);
                point.anchor_b_local = to_local_anchor(b.center, b.orientation, on_b);
                point.separation = separation;
                point.normal_lambda = T(0);
                point.tangent_lambda[0] = T(0);
                point.tangent_lambda[1] = T(0);
                point.feature_id = make_feature_id(reference_face, incident_face,
                                                   candidate.vertex_id, reference_is_b);
            }
            return manifold;
        }
    } // namespace Physics
} // namespace SushiEngine
