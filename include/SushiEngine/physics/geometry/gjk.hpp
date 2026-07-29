/**************************************************************************/
/* gjk.hpp                                                                */
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
 * @file gjk.hpp
 * @brief The one routine that collides any convex shape with any other.
 *
 * Gilbert–Johnson–Keerthi for the separated case and Expanding Polytope
 * Algorithm for the penetrating one, over the **Minkowski difference** `A - B`.
 * The whole method rests on one observation: two convex shapes intersect exactly
 * when their Minkowski difference contains the origin, and their separation is
 * exactly the distance from the origin to that difference. So a question about
 * two shapes becomes a question about one set and a point — and that set is
 * never built, only *sampled*, through each shape's support function.
 *
 * This is the Open/Closed payoff §4.2 promises, and it is worth being concrete
 * about the size of it: `shapes.hpp` gains one `support()` overload per new
 * convex shape, and that shape immediately collides correctly against spheres,
 * boxes, capsules, hulls, and every convex shape added afterwards. Nothing here
 * changes, and there is no `switch` anywhere to grow a case. The alternative —
 * an analytic routine per ordered pair — is quadratic in the shape count, which
 * is how a narrowphase becomes the file nobody wants to touch.
 *
 * What this file deliberately does *not* do is produce a contact **patch**. GJK
 * and EPA answer with a single deepest point and a normal, which is the honest
 * answer to the question they were asked and not enough for a hull to rest flat
 * (§7.3 — the same argument that motivated manifolds in the first place). The
 * patch is built on top, in `collision/convex_manifold.hpp`, by taking the
 * normal from here and clipping the two contact faces against each other.
 *
 * Everything is fixed-capacity and free of allocation, so the routine is
 * device-copyable as it stands.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief How many vertices the expanding polytope may reach before giving up. */
        inline constexpr std::size_t max_polytope_vertices = 64;

        /** @brief How many faces the expanding polytope may reach. */
        inline constexpr std::size_t max_polytope_faces = 128;

        /** @brief How many horizon edges one expansion step may open. */
        inline constexpr std::size_t max_horizon_edges = 64;

        /**
         * @brief What a convex-convex query answers with.
         *
         * The same convention every narrowphase in the engine uses: @ref normal is
         * unit and points from shape `a` toward shape `b`, and @ref separation is
         * negative when the shapes overlap. @ref point_a and @ref point_b are the
         * witness points — the closest pair when separated, the deepest pair when
         * penetrating — in world space, which is exactly what a manifold anchor
         * needs.
         */
        template <typename T>
        struct ConvexContact
        {
            bool valid = false;
            Vector3T<T> normal{Vector3T<T>{T(0), T(1), T(0)}};
            T separation = 0;
            Vector3T<T> point_a{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> point_b{Vector3T<T>{T(0), T(0), T(0)}};
        };

        /**
         * @brief One sampled point of the Minkowski difference, with its two origins.
         *
         * Carrying the two supporting points alongside the difference is what makes
         * witness recovery possible at the end: the closest point of the difference
         * is a barycentric combination of a few of these, and the same combination
         * of their `support_a` and `support_b` is the closest pair on the two
         * shapes. Recomputing them afterwards from the normal would be a second,
         * disagreeing answer.
         */
        template <typename T>
        struct MinkowskiVertex
        {
            Vector3T<T> point{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> support_a{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> support_b{Vector3T<T>{T(0), T(0), T(0)}};
        };

        /** @brief Samples the Minkowski difference `A - B` along @p direction. */
        template <typename T, typename ShapeA, typename ShapeB>
        inline MinkowskiVertex<T> minkowski_support(const ShapeA& a, const ShapeB& b,
                                                    const Vector3T<T>& direction) noexcept
        {
            MinkowskiVertex<T> vertex;
            vertex.support_a = support(a, direction);
            vertex.support_b = support(b, direction * T(-1));
            vertex.point = vertex.support_a - vertex.support_b;
            return vertex;
        }

        /**
         * @brief The closest point of a simplex to the origin, and which vertices carry it.
         *
         * GJK's inner half. Given up to four points it reports the nearest point of
         * their convex hull to the origin, the sub-simplex that point actually lies
         * on, and the barycentric weights over it — the reduction is what keeps the
         * simplex from growing and what recovers the witness points at the end.
         */
        template <typename T>
        struct SimplexReduction
        {
            Vector3T<T> closest{Vector3T<T>{T(0), T(0), T(0)}};
            T weights[4] = {0, 0, 0, 0};
            int indices[4] = {0, 0, 0, 0};
            int count = 0;
            bool contains_origin = false;
        };

        /** @brief Closest point of segment (a, b) to the origin, with barycentric weights. */
        template <typename T>
        inline void closest_on_segment(const Vector3T<T>& a, const Vector3T<T>& b, T& weight_a,
                                       T& weight_b) noexcept
        {
            const Vector3T<T> ab = b - a;
            const T denominator = dot(ab, ab);
            if (denominator <= T(1e-24))
            {
                weight_a = T(1);
                weight_b = T(0);
                return;
            }
            T t = dot(a * T(-1), ab) / denominator;
            t = t < T(0) ? T(0) : (t > T(1) ? T(1) : t);
            weight_a = T(1) - t;
            weight_b = t;
        }

        /**
         * @brief Closest point of triangle (a, b, c) to the origin, with barycentric weights.
         *
         * Ericson's region test, specialized to the origin: the plane of the
         * triangle is divided into seven Voronoi regions by the three vertices and
         * three edges, the sign tests say which one the origin falls in, and the
         * answer follows without a projection or a division except in the interior
         * case.
         */
        template <typename T>
        inline void closest_on_triangle(const Vector3T<T>& a, const Vector3T<T>& b,
                                        const Vector3T<T>& c, T weights[3]) noexcept
        {
            const Vector3T<T> ab = b - a;
            const Vector3T<T> ac = c - a;
            const Vector3T<T> ap = a * T(-1);
            const T d1 = dot(ab, ap);
            const T d2 = dot(ac, ap);
            if (d1 <= T(0) && d2 <= T(0))
            {
                weights[0] = T(1);
                weights[1] = T(0);
                weights[2] = T(0);
                return;
            }

            const Vector3T<T> bp = b * T(-1);
            const T d3 = dot(ab, bp);
            const T d4 = dot(ac, bp);
            if (d3 >= T(0) && d4 <= d3)
            {
                weights[0] = T(0);
                weights[1] = T(1);
                weights[2] = T(0);
                return;
            }

            const T vc = d1 * d4 - d3 * d2;
            if (vc <= T(0) && d1 >= T(0) && d3 <= T(0))
            {
                const T denominator = d1 - d3;
                const T v = denominator != T(0) ? d1 / denominator : T(0);
                weights[0] = T(1) - v;
                weights[1] = v;
                weights[2] = T(0);
                return;
            }

            const Vector3T<T> cp = c * T(-1);
            const T d5 = dot(ab, cp);
            const T d6 = dot(ac, cp);
            if (d6 >= T(0) && d5 <= d6)
            {
                weights[0] = T(0);
                weights[1] = T(0);
                weights[2] = T(1);
                return;
            }

            const T vb = d5 * d2 - d1 * d6;
            if (vb <= T(0) && d2 >= T(0) && d6 <= T(0))
            {
                const T denominator = d2 - d6;
                const T w = denominator != T(0) ? d2 / denominator : T(0);
                weights[0] = T(1) - w;
                weights[1] = T(0);
                weights[2] = w;
                return;
            }

            const T va = d3 * d6 - d5 * d4;
            if (va <= T(0) && (d4 - d3) >= T(0) && (d5 - d6) >= T(0))
            {
                const T denominator = (d4 - d3) + (d5 - d6);
                const T w = denominator != T(0) ? (d4 - d3) / denominator : T(0);
                weights[0] = T(0);
                weights[1] = T(1) - w;
                weights[2] = w;
                return;
            }

            const T total = va + vb + vc;
            if (total <= T(1e-24))
            {
                // Degenerate triangle: fall back to the longest edge.
                closest_on_segment(a, b, weights[0], weights[1]);
                weights[2] = T(0);
                return;
            }
            const T denominator = T(1) / total;
            weights[1] = vb * denominator;
            weights[2] = vc * denominator;
            weights[0] = T(1) - weights[1] - weights[2];
        }

        /**
         * @brief Reduces a simplex of 1..4 points to the part nearest the origin.
         *
         * For a tetrahedron the origin is tested against each face's outward plane;
         * if it is inside all four the shapes overlap and GJK is finished. Otherwise
         * only the faces it is outside of can hold the answer, and the nearest of
         * those wins. The outward direction of each face is taken against the
         * opposite vertex rather than assumed from winding, because the simplex GJK
         * builds has no guaranteed orientation.
         */
        /**
         * @brief Drops vertices the closest point does not actually rest on.
         *
         * A reduction that reports "the closest point is on this triangle, with the
         * third vertex weighted zero" is describing an edge. Keeping the vertex
         * anyway is not merely untidy: GJK adds one point per iteration, so a
         * simplex padded with a useless vertex reaches four early, the next
         * reduction can drop the *newly found* point instead of the dead one, and
         * the search stops making progress while the termination test — which only
         * asks whether the support reaches past the current closest point — happily
         * reports convergence. The result is a plausible, wrong distance.
         */
        template <typename T>
        inline void compact_reduction(SimplexReduction<T>& reduction) noexcept
        {
            int written = 0;
            for (int i = 0; i < reduction.count; ++i)
            {
                if (reduction.weights[i] <= T(1e-12) && reduction.count > 1)
                    continue;
                reduction.weights[written] = reduction.weights[i];
                reduction.indices[written] = reduction.indices[i];
                ++written;
            }
            if (written > 0)
                reduction.count = written;
        }

        template <typename T>
        inline SimplexReduction<T> reduce_simplex(const MinkowskiVertex<T>* simplex,
                                                  int count) noexcept
        {
            SimplexReduction<T> reduction;
            if (count <= 0)
                return reduction;

            if (count == 1)
            {
                reduction.count = 1;
                reduction.indices[0] = 0;
                reduction.weights[0] = T(1);
                reduction.closest = simplex[0].point;
                return reduction;
            }

            if (count == 2)
            {
                T weight_a = T(0);
                T weight_b = T(0);
                closest_on_segment(simplex[0].point, simplex[1].point, weight_a, weight_b);
                reduction.count = 2;
                reduction.indices[0] = 0;
                reduction.indices[1] = 1;
                reduction.weights[0] = weight_a;
                reduction.weights[1] = weight_b;
                reduction.closest = simplex[0].point * weight_a + simplex[1].point * weight_b;
                compact_reduction(reduction);
                return reduction;
            }

            if (count == 3)
            {
                T weights[3];
                closest_on_triangle(simplex[0].point, simplex[1].point, simplex[2].point, weights);
                reduction.count = 3;
                for (int i = 0; i < 3; ++i)
                {
                    reduction.indices[i] = i;
                    reduction.weights[i] = weights[i];
                }
                reduction.closest = simplex[0].point * weights[0] +
                                    simplex[1].point * weights[1] + simplex[2].point * weights[2];
                compact_reduction(reduction);
                return reduction;
            }

            // Tetrahedron: the four faces, each named by the three vertices on it.
            static const int faces[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
            static const int opposite[4] = {3, 2, 1, 0};

            bool inside_all = true;
            T best_distance_squared = T(1e30);
            bool have_best = false;

            for (int f = 0; f < 4; ++f)
            {
                const Vector3T<T>& p0 = simplex[faces[f][0]].point;
                const Vector3T<T>& p1 = simplex[faces[f][1]].point;
                const Vector3T<T>& p2 = simplex[faces[f][2]].point;
                Vector3T<T> normal = cross(p1 - p0, p2 - p0);
                const T normal_length_squared = dot(normal, normal);
                if (normal_length_squared <= T(1e-24))
                    continue; // degenerate face: it cannot separate anything
                // Point the normal away from the opposite vertex.
                if (dot(normal, simplex[opposite[f]].point - p0) > T(0))
                    normal = normal * T(-1);
                if (dot(normal, p0 * T(-1)) <= T(0))
                    continue; // origin is on the inner side of this face

                inside_all = false;
                T weights[3];
                closest_on_triangle(p0, p1, p2, weights);
                const Vector3T<T> candidate =
                    p0 * weights[0] + p1 * weights[1] + p2 * weights[2];
                const T distance_squared = dot(candidate, candidate);
                if (!have_best || distance_squared < best_distance_squared)
                {
                    have_best = true;
                    best_distance_squared = distance_squared;
                    reduction.count = 3;
                    reduction.closest = candidate;
                    for (int i = 0; i < 3; ++i)
                    {
                        reduction.indices[i] = faces[f][i];
                        reduction.weights[i] = weights[i];
                    }
                }
            }

            if (inside_all)
            {
                reduction.contains_origin = true;
                reduction.count = 4;
                for (int i = 0; i < 4; ++i)
                {
                    reduction.indices[i] = i;
                    reduction.weights[i] = T(0.25);
                }
                return reduction;
            }
            if (!have_best)
            {
                // Every face was degenerate — the tetrahedron is flat. Reduce to its
                // first triangle rather than reporting nonsense.
                T weights[3];
                closest_on_triangle(simplex[0].point, simplex[1].point, simplex[2].point, weights);
                reduction.count = 3;
                for (int i = 0; i < 3; ++i)
                {
                    reduction.indices[i] = i;
                    reduction.weights[i] = weights[i];
                }
                reduction.closest = simplex[0].point * weights[0] +
                                    simplex[1].point * weights[1] + simplex[2].point * weights[2];
            }
            compact_reduction(reduction);
            return reduction;
        }

        /** @brief One face of the expanding polytope. */
        template <typename T>
        struct PolytopeFace
        {
            int vertex[3] = {0, 0, 0};
            Vector3T<T> normal{Vector3T<T>{T(0), T(1), T(0)}};
            T distance = 0; /**< Distance from the origin to the face's plane. */
            bool alive = false;
        };

        /**
         * @brief Expands a polytope inside the Minkowski difference to find the depth.
         *
         * EPA. GJK has already established that the origin is inside the difference;
         * the penetration depth is then the distance from the origin to the
         * difference's *boundary*, and the direction is the surface normal there.
         * The polytope starts as GJK's tetrahedron and is repeatedly grown toward
         * whichever face is currently nearest the origin, until growing stops moving
         * that face — at which point the face lies on the boundary and its plane is
         * the answer.
         *
         * @param a         The first shape.
         * @param b         The second shape.
         * @param simplex   GJK's terminating simplex (four vertices enclosing the origin).
         * @param tolerance Growth below this ends the expansion.
         * @return The contact, or an invalid result if the polytope could not be built.
         */
        template <typename T, typename ShapeA, typename ShapeB>
        inline ConvexContact<T> expand_polytope(const ShapeA& a, const ShapeB& b,
                                                const MinkowskiVertex<T>* simplex,
                                                T tolerance) noexcept
        {
            MinkowskiVertex<T> vertices[max_polytope_vertices];
            PolytopeFace<T> faces[max_polytope_faces];
            std::size_t vertex_count = 4;
            std::size_t face_count = 0;

            for (std::size_t i = 0; i < 4; ++i)
                vertices[i] = simplex[i];

            // Faces are oriented away from the seed tetrahedron's centroid, not away
            // from the origin — and that distinction is the whole reason this routine
            // is robust. The origin can land exactly *on* a seed face (two spheres
            // whose centres are on the x axis produce a simplex whose first two
            // vertices straddle the origin along that axis, so every face containing
            // both of them passes through it), and a point on a plane has no side of
            // it to be on. Orienting from the origin then flips a coin, and a face
            // pointing inward reports a support that never reaches it, so EPA
            // terminates at distance zero and calls two overlapping shapes touching.
            // The centroid is strictly interior by construction and stays interior as
            // the polytope only ever grows, so it never has to make that choice.
            //
            // A face may therefore legitimately carry a *negative* distance while the
            // origin is still outside the seed. That is not an error state: it simply
            // wins the nearest-face search below and gets expanded first, which is
            // exactly the repair such a seed needs.
            const Vector3T<T> interior =
                (simplex[0].point + simplex[1].point + simplex[2].point + simplex[3].point) *
                T(0.25);

            // Faces are removed as often as they are added — every expansion step
            // kills everything the new vertex can see — so slots are recycled rather
            // than appended. Without that, `face_count` is a high-water mark that
            // only climbs, the capacity check fires after twenty-odd expansions, and
            // EPA stops early on whatever face happened to be nearest. That failure
            // is quiet and looks exactly like convergence.
            bool exhausted = false;
            const auto make_face = [&](int i0, int i1, int i2) noexcept
            {
                std::size_t slot = face_count;
                for (std::size_t f = 0; f < face_count; ++f)
                    if (!faces[f].alive)
                    {
                        slot = f;
                        break;
                    }
                if (slot == face_count)
                {
                    if (face_count >= max_polytope_faces)
                    {
                        exhausted = true;
                        return;
                    }
                    ++face_count;
                }
                PolytopeFace<T>& face = faces[slot];
                const Vector3T<T>& p0 = vertices[i0].point;
                Vector3T<T> normal = cross(vertices[i1].point - p0, vertices[i2].point - p0);
                const T length_squared = dot(normal, normal);
                if (length_squared <= T(1e-24))
                    return; // degenerate triangle: never a boundary face
                normal = normal * (T(1) / std::sqrt(length_squared));
                // Flip the *winding*, not only the normal. The horizon walk below
                // cancels an edge against its reverse, which only identifies interior
                // edges if every face is wound consistently with its outward normal.
                // Storing a flipped normal over the original winding leaves two
                // neighbouring faces listing the shared edge in the same direction,
                // nothing cancels, and the rebuild produces duplicate faces that keep
                // the polytope flat — EPA then never converges and stops on whatever
                // face the iteration cap leaves it holding.
                int v0 = i0;
                int v1 = i1;
                int v2 = i2;
                if (dot(normal, p0 - interior) < T(0))
                {
                    normal = normal * T(-1);
                    v1 = i2;
                    v2 = i1;
                }
                face.vertex[0] = v0;
                face.vertex[1] = v1;
                face.vertex[2] = v2;
                face.normal = normal;
                face.distance = dot(normal, p0);
                face.alive = true;
            };

            make_face(0, 1, 2);
            make_face(0, 1, 3);
            make_face(0, 2, 3);
            make_face(1, 2, 3);
            std::size_t live_faces = 0;
            for (std::size_t f = 0; f < face_count; ++f)
                live_faces += faces[f].alive ? 1u : 0u;
            if (live_faces < 4)
                return ConvexContact<T>{}; // a flat seed has no volume to expand

            for (std::size_t iteration = 0; iteration < 64; ++iteration)
            {
                // The face nearest the origin is the current best guess at the
                // boundary, so it is the one worth pushing on.
                std::size_t nearest = max_polytope_faces;
                T nearest_distance = T(1e30);
                for (std::size_t f = 0; f < face_count; ++f)
                {
                    if (!faces[f].alive)
                        continue;
                    if (faces[f].distance < nearest_distance)
                    {
                        nearest_distance = faces[f].distance;
                        nearest = f;
                    }
                }
                if (nearest == max_polytope_faces)
                    return ConvexContact<T>{};

                const PolytopeFace<T> best = faces[nearest];
                const MinkowskiVertex<T> grown = minkowski_support(a, b, best.normal);
                const T reach = dot(grown.point, best.normal);

                if (reach - best.distance <= tolerance ||
                    vertex_count >= max_polytope_vertices || exhausted)
                {
                    // The boundary has been reached: recover the witness points by
                    // the barycentric coordinates of the origin's projection onto
                    // this face.
                    const Vector3T<T> projection = best.normal * best.distance;
                    T weights[3];
                    closest_on_triangle(vertices[best.vertex[0]].point - projection,
                                        vertices[best.vertex[1]].point - projection,
                                        vertices[best.vertex[2]].point - projection, weights);
                    ConvexContact<T> contact;
                    contact.valid = true;
                    contact.normal = best.normal;
                    contact.separation = -best.distance;
                    contact.point_a = vertices[best.vertex[0]].support_a * weights[0] +
                                      vertices[best.vertex[1]].support_a * weights[1] +
                                      vertices[best.vertex[2]].support_a * weights[2];
                    contact.point_b = vertices[best.vertex[0]].support_b * weights[0] +
                                      vertices[best.vertex[1]].support_b * weights[1] +
                                      vertices[best.vertex[2]].support_b * weights[2];
                    return contact;
                }

                // Everything the new point can see is no longer on the boundary. The
                // edges bounding what it can see are the horizon, and they are the
                // rim the new vertex is stitched to; an edge shared by two visible
                // faces is interior and cancels out, which is what the pairwise
                // removal below implements.
                int horizon[max_horizon_edges][2];
                std::size_t horizon_count = 0;
                for (std::size_t f = 0; f < face_count; ++f)
                {
                    if (!faces[f].alive)
                        continue;
                    if (dot(faces[f].normal, grown.point) - faces[f].distance <= T(0))
                        continue;
                    faces[f].alive = false;
                    for (int e = 0; e < 3; ++e)
                    {
                        const int i0 = faces[f].vertex[e];
                        const int i1 = faces[f].vertex[(e + 1) % 3];
                        bool cancelled = false;
                        for (std::size_t k = 0; k < horizon_count; ++k)
                        {
                            if (horizon[k][0] == i1 && horizon[k][1] == i0)
                            {
                                horizon[k][0] = horizon[horizon_count - 1][0];
                                horizon[k][1] = horizon[horizon_count - 1][1];
                                --horizon_count;
                                cancelled = true;
                                break;
                            }
                        }
                        if (!cancelled && horizon_count < max_horizon_edges)
                        {
                            horizon[horizon_count][0] = i0;
                            horizon[horizon_count][1] = i1;
                            ++horizon_count;
                        }
                    }
                }
                if (horizon_count == 0)
                    return ConvexContact<T>{};

                const int added = static_cast<int>(vertex_count);
                vertices[vertex_count] = grown;
                ++vertex_count;
                for (std::size_t k = 0; k < horizon_count; ++k)
                    make_face(horizon[k][0], horizon[k][1], added);
            }
            return ConvexContact<T>{};
        }

        /**
         * @brief Collides any two convex shapes, separated or overlapping.
         *
         * @param a         The first shape; any type with a `support()` overload.
         * @param b         The second shape.
         * @param tolerance Convergence threshold, in metres. The default suits
         *                  metre-scale geometry in double precision.
         * @return The contact. `separation` is positive when the shapes are apart and
         *         negative when they overlap, so a caller comparing it against a
         *         contact offset needs no case split.
         */
        template <typename T, typename ShapeA, typename ShapeB>
        inline ConvexContact<T> collide_convex(const ShapeA& a, const ShapeB& b,
                                               T tolerance = T(1e-9)) noexcept
        {
            MinkowskiVertex<T> simplex[4];
            int count = 0;

            Vector3T<T> direction{T(1), T(0), T(0)};
            simplex[count++] = minkowski_support(a, b, direction);
            if (dot(simplex[0].point, simplex[0].point) <= T(1e-24))
            {
                // The first sample landed on the origin: the shapes touch exactly
                // there. Nudge, so the simplex has a direction to grow along.
                direction = Vector3T<T>{T(0), T(1), T(0)};
                simplex[0] = minkowski_support(a, b, direction);
            }
            direction = simplex[0].point * T(-1);

            SimplexReduction<T> reduction;
            for (std::size_t iteration = 0; iteration < 64; ++iteration)
            {
                reduction = reduce_simplex(simplex, count);
                if (reduction.contains_origin)
                    break;

                const T distance_squared = dot(reduction.closest, reduction.closest);
                if (distance_squared <= T(1e-20))
                {
                    // The origin lies on the simplex: touching, or overlapping by an
                    // amount below what this precision can resolve. Grow the simplex
                    // to a tetrahedron and let EPA answer.
                    reduction.contains_origin = true;
                    break;
                }

                // Keep only the sub-simplex the reduction landed on.
                MinkowskiVertex<T> kept[4];
                for (int i = 0; i < reduction.count; ++i)
                    kept[i] = simplex[reduction.indices[i]];
                for (int i = 0; i < reduction.count; ++i)
                    simplex[i] = kept[i];
                count = reduction.count;

                direction = reduction.closest * T(-1);
                const MinkowskiVertex<T> grown = minkowski_support(a, b, direction);

                // Termination: `dot(w, v) / |v|` is a lower bound on the true
                // distance, so when it has caught up with `|v|` there is nothing
                // closer to find and the shapes are apart by exactly that.
                const T distance = std::sqrt(distance_squared);
                if (distance - dot(grown.point, reduction.closest) / distance <= tolerance)
                    break;

                if (count >= 4)
                    break; // no progress and no room: treat as converged
                simplex[count++] = grown;
            }

            if (reduction.contains_origin)
            {
                if (count < 4)
                {
                    // EPA needs a volume. Grow whatever GJK ended with into one by
                    // sampling along directions the current simplex does not span.
                    while (count < 4)
                    {
                        Vector3T<T> axis{T(1), T(0), T(0)};
                        if (count == 1)
                        {
                            axis = Vector3T<T>{T(1), T(0), T(0)};
                        }
                        else if (count == 2)
                        {
                            const Vector3T<T> edge = simplex[1].point - simplex[0].point;
                            const Vector3T<T> reference =
                                std::abs(edge.x) < std::abs(edge.y) ? Vector3T<T>{T(1), T(0), T(0)}
                                                                    : Vector3T<T>{T(0), T(1), T(0)};
                            axis = cross(edge, reference);
                        }
                        else
                        {
                            axis = cross(simplex[1].point - simplex[0].point,
                                         simplex[2].point - simplex[0].point);
                        }
                        if (dot(axis, axis) <= T(1e-24))
                            axis = Vector3T<T>{T(0), T(0), T(1)};

                        MinkowskiVertex<T> grown = minkowski_support(a, b, axis);
                        // A direction that adds nothing gets tried the other way
                        // before the simplex is declared unbuildable.
                        bool duplicate = false;
                        for (int i = 0; i < count; ++i)
                        {
                            const Vector3T<T> delta = grown.point - simplex[i].point;
                            if (dot(delta, delta) <= T(1e-20))
                                duplicate = true;
                        }
                        if (duplicate)
                        {
                            grown = minkowski_support(a, b, axis * T(-1));
                            for (int i = 0; i < count; ++i)
                            {
                                const Vector3T<T> delta = grown.point - simplex[i].point;
                                if (dot(delta, delta) <= T(1e-20))
                                    return ConvexContact<T>{};
                            }
                        }
                        simplex[count++] = grown;
                    }
                }
                return expand_polytope(a, b, simplex, tolerance);
            }

            // Separated: the reduction's weights carry the witness pair.
            ConvexContact<T> contact;
            Vector3T<T> point_a{T(0), T(0), T(0)};
            Vector3T<T> point_b{T(0), T(0), T(0)};
            for (int i = 0; i < reduction.count; ++i)
            {
                point_a = point_a + simplex[reduction.indices[i]].support_a * reduction.weights[i];
                point_b = point_b + simplex[reduction.indices[i]].support_b * reduction.weights[i];
            }
            const T distance = length(reduction.closest);
            contact.valid = true;
            contact.separation = distance;
            contact.point_a = point_a;
            contact.point_b = point_b;
            // The closest point of `A - B` points from b to a, so the contact normal
            // — which runs a to b — is its negation.
            contact.normal = distance > T(1e-12) ? reduction.closest * (T(-1) / distance)
                                                 : Vector3T<T>{T(0), T(1), T(0)};
            return contact;
        }
    } // namespace Physics
} // namespace SushiEngine
