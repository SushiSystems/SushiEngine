/**************************************************************************/
/* convex_manifold.hpp                                                    */
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
 * @file convex_manifold.hpp
 * @brief Turning GJK's single deepest point into the patch two shapes rest on.
 *
 * `geometry/gjk.hpp` answers the question it was asked — how far apart are these
 * two convex shapes, and where — and that answer is one point and a normal. A
 * body held by one point rocks and never settles; that is §1.2 item 3, and
 * `collision/manifold.hpp` already solved it for boxes by clipping one face
 * against another. This file generalizes that to every convex shape, using
 * GJK's normal as the axis to clip along.
 *
 * The step that makes it work without cooked face data is **contact-face
 * extraction**: given a direction, a convex shape reports every one of its
 * features that is extreme along it, within a tolerance. A box reports the four
 * corners of a face, a capsule lying on its side reports the two ends of its
 * segment, a sphere reports one point, and a hull reports whichever of its
 * vertices are flush with the supporting plane. That is enough to clip with, and
 * it needs no adjacency — so a hull collides and *rests* correctly today rather
 * than waiting for P4's cooker.
 *
 * Like the support function it sits next to, extraction is an overload set: a
 * new convex shape adds one `contact_face()` overload and gains resting contact
 * along with everything else.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/geometry/gjk.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The features of a shape that are extreme along a direction.
         *
         * One point for a sphere or a corner-on contact, two for an edge, three or
         * more for a face. The count *is* the information: a shape reporting one
         * point cannot be clipped against and does not need to be.
         */
        template <typename T>
        struct ContactFace
        {
            ClippedPoint<T> points[max_clipped_points];
            std::size_t count = 0;
            /** @brief The plane normal of the extracted face, or the query direction. */
            Vector3T<T> normal{Vector3T<T>{T(0), T(1), T(0)}};
        };

        /** @brief A sphere touches along one point, whatever the direction. */
        template <typename T>
        inline ContactFace<T> contact_face(const SphereCollider<T>& sphere,
                                           const Vector3T<T>& direction, T) noexcept
        {
            ContactFace<T> face;
            face.normal = safe_normalize(direction);
            face.points[0].position = sphere.center + face.normal * sphere.radius;
            face.points[0].vertex_id = 0;
            face.count = 1;
            return face;
        }

        /**
         * @brief A capsule touches along one point, or along its whole segment.
         *
         * The distinction is whether the segment is perpendicular to the query
         * direction: a capsule standing on its cap touches at one place, a capsule
         * lying on its side touches along a line, and a line held at one point
         * rolls. The tolerance is on the *sine* of the angle between them, so it
         * scales with the shape rather than with the world.
         */
        template <typename T>
        inline ContactFace<T> contact_face(const CapsuleCollider<T>& capsule,
                                           const Vector3T<T>& direction, T tolerance) noexcept
        {
            ContactFace<T> face;
            face.normal = safe_normalize(direction);

            Vector3T<T> start;
            Vector3T<T> end;
            capsule_segment(capsule, start, end);
            const Vector3T<T> axis = safe_normalize(end - start);
            if (capsule.half_height > T(0) && std::abs(dot(axis, face.normal)) <= tolerance)
            {
                face.points[0].position = start + face.normal * capsule.radius;
                face.points[0].vertex_id = 0;
                face.points[1].position = end + face.normal * capsule.radius;
                face.points[1].vertex_id = 1;
                face.count = 2;
                return face;
            }

            face.points[0].position = support(capsule, direction);
            face.points[0].vertex_id = 2;
            face.count = 1;
            return face;
        }

        /**
         * @brief A box touches along the face most aligned with the direction.
         *
         * When the direction is not close to any face normal the contact is really
         * an edge or a corner, and reporting four coplanar points would invent a
         * patch that does not exist. The alignment test is the same one the
         * separating-axis path already uses in spirit: if no face normal is within
         * the tolerance of the direction, fall back to the support point.
         */
        template <typename T>
        inline ContactFace<T> contact_face(const OrientedBox<T>& box,
                                           const Vector3T<T>& direction, T tolerance) noexcept
        {
            ContactFace<T> face;
            const Vector3T<T> unit = safe_normalize(direction);
            face.normal = unit;

            Vector3T<T> axes[3];
            obb_axes(box, axes);
            int best_axis = 0;
            T best_sign = T(1);
            T best_alignment = T(-2);
            for (int i = 0; i < 3; ++i)
                for (int s = 0; s < 2; ++s)
                {
                    const T sign = s == 0 ? T(1) : T(-1);
                    const T alignment = dot(axes[i] * sign, unit);
                    if (alignment > best_alignment)
                    {
                        best_alignment = alignment;
                        best_axis = i;
                        best_sign = sign;
                    }
                }

            if (best_alignment < T(1) - tolerance)
            {
                face.points[0].position = obb_support_point(box, direction);
                face.points[0].vertex_id = 15;
                face.count = 1;
                return face;
            }

            Vector3T<T> corners[4];
            obb_face_corners(box, best_axis, best_sign, corners);
            const std::uint32_t face_index = obb_face_index(best_axis, best_sign > T(0));
            for (std::uint32_t i = 0; i < 4; ++i)
            {
                face.points[i].position = corners[i];
                face.points[i].vertex_id = face_index * 4u + i;
            }
            face.count = 4;
            face.normal = axes[best_axis] * best_sign;
            return face;
        }

        /**
         * @brief Sorts a coplanar point set into convex order around its centroid.
         *
         * Clipping needs the reference polygon's edges, and an edge is only an edge
         * if the points are in order. Extraction gathers vertices by how extreme
         * they are, which says nothing about their order, so the order is recovered
         * here — by the angle each point makes about the face centroid in the
         * face's own plane. Coplanar and convex by construction, so an angular sort
         * is exactly right and no hull algorithm is needed.
         */
        template <typename T>
        inline void sort_face_points(ContactFace<T>& face) noexcept
        {
            if (face.count < 3)
                return;

            Vector3T<T> centroid{T(0), T(0), T(0)};
            for (std::size_t i = 0; i < face.count; ++i)
                centroid = centroid + face.points[i].position;
            centroid = centroid * (T(1) / static_cast<T>(face.count));

            Vector3T<T> tangent_0;
            Vector3T<T> tangent_1;
            {
                const Vector3T<T>& normal = face.normal;
                const T ax = std::abs(normal.x);
                const T ay = std::abs(normal.y);
                const T az = std::abs(normal.z);
                Vector3T<T> reference{T(1), T(0), T(0)};
                if (ay <= ax && ay <= az)
                    reference = Vector3T<T>{T(0), T(1), T(0)};
                else if (az <= ax && az <= ay)
                    reference = Vector3T<T>{T(0), T(0), T(1)};
                tangent_0 = safe_normalize(cross(normal, reference));
                tangent_1 = cross(normal, tangent_0);
            }

            T angles[max_clipped_points];
            for (std::size_t i = 0; i < face.count; ++i)
            {
                const Vector3T<T> arm = face.points[i].position - centroid;
                angles[i] = std::atan2(dot(arm, tangent_1), dot(arm, tangent_0));
            }
            // Insertion sort: the set is at most eight points and the order must be
            // the same every tick, which a stable sort of a small array gives for
            // free.
            for (std::size_t i = 1; i < face.count; ++i)
            {
                const T angle = angles[i];
                const ClippedPoint<T> point = face.points[i];
                std::size_t j = i;
                while (j > 0 && angles[j - 1] > angle)
                {
                    angles[j] = angles[j - 1];
                    face.points[j] = face.points[j - 1];
                    --j;
                }
                angles[j] = angle;
                face.points[j] = point;
            }
        }

        /**
         * @brief A hull touches along whichever vertices are flush with the supporting plane.
         *
         * No face data, no adjacency: the vertices within @p tolerance of the
         * extreme projection *are* the contact face, because a convex hull's
         * supporting plane in a direction cuts exactly one face, edge, or vertex.
         * The tolerance is a distance, scaled by the caller to the shape.
         */
        template <typename T>
        inline ContactFace<T> contact_face(const ConvexHullView<T>& hull,
                                           const Vector3T<T>& direction, T tolerance) noexcept
        {
            ContactFace<T> face;
            const Vector3T<T> unit = safe_normalize(direction);
            face.normal = unit;
            if (hull.vertices == nullptr || hull.vertex_count == 0)
            {
                face.points[0].position = hull.center;
                face.points[0].vertex_id = 0;
                face.count = 1;
                return face;
            }

            const Vector3T<T> local_direction = rotate(conjugate(hull.orientation), unit);
            T extreme = dot(hull.vertices[0], local_direction);
            for (std::uint32_t i = 1; i < hull.vertex_count; ++i)
            {
                const T projection = dot(hull.vertices[i], local_direction);
                if (projection > extreme)
                    extreme = projection;
            }

            const Vector3T<T> inflation = unit * hull.convex_radius;
            for (std::uint32_t i = 0; i < hull.vertex_count && face.count < max_clipped_points; ++i)
            {
                if (dot(hull.vertices[i], local_direction) < extreme - tolerance)
                    continue;
                face.points[face.count].position =
                    hull.center + rotate(hull.orientation, hull.vertices[i]) + inflation;
                face.points[face.count].vertex_id = i;
                ++face.count;
            }

            if (face.count >= 3)
            {
                // The extracted face's own plane, which is a better clipping axis
                // than the query direction when the two differ slightly.
                const Vector3T<T> edge_0 = face.points[1].position - face.points[0].position;
                const Vector3T<T> edge_1 = face.points[2].position - face.points[0].position;
                const Vector3T<T> plane_normal = cross(edge_0, edge_1);
                if (dot(plane_normal, plane_normal) > T(1e-24))
                {
                    const Vector3T<T> candidate = safe_normalize(plane_normal);
                    face.normal = dot(candidate, unit) < T(0) ? candidate * T(-1) : candidate;
                }
                sort_face_points(face);
            }
            return face;
        }

        /**
         * @brief Clips the incident face against the reference face's side planes.
         *
         * The same Sutherland–Hodgman pass `manifold.hpp` runs for boxes, over an
         * arbitrary reference polygon rather than a rectangle: each edge of the
         * reference face extrudes along its normal into a side plane, and the
         * incident polygon is trimmed by all of them in turn.
         *
         * @return The number of surviving points, written to @p output.
         */
        template <typename T>
        inline std::size_t clip_against_face(const ContactFace<T>& incident,
                                             const ContactFace<T>& reference, T tolerance,
                                             ClippedPoint<T>* output) noexcept
        {
            ClippedPoint<T> buffer_a[max_clipped_points];
            ClippedPoint<T> buffer_b[max_clipped_points];
            std::size_t count = incident.count;
            for (std::size_t i = 0; i < count; ++i)
                buffer_a[i] = incident.points[i];

            ClippedPoint<T>* source = buffer_a;
            ClippedPoint<T>* destination = buffer_b;
            for (std::size_t e = 0; e < reference.count && count > 0; ++e)
            {
                const Vector3T<T>& from = reference.points[e].position;
                const Vector3T<T>& to = reference.points[(e + 1) % reference.count].position;
                const Vector3T<T> plane_normal = cross(to - from, reference.normal);
                if (dot(plane_normal, plane_normal) <= T(1e-24))
                    continue;
                const Vector3T<T> unit = safe_normalize(plane_normal);
                count = clip_polygon_against_plane(source, count, unit, dot(unit, from),
                                                   static_cast<std::uint32_t>(4 + e), tolerance,
                                                   destination);
                ClippedPoint<T>* const swapped = source;
                source = destination;
                destination = swapped;
            }

            for (std::size_t i = 0; i < count; ++i)
                output[i] = source[i];
            return count;
        }

        /**
         * @brief The manifold between any two convex shapes.
         *
         * GJK supplies the normal and the depth; the patch is built by extracting
         * both shapes' contact faces along it and clipping one against the other.
         * When either shape touches at a single point — a sphere always, a box
         * caught on a corner, a capsule on its cap — there is no patch to build and
         * the witness points are the answer.
         *
         * @param a              The first shape; the normal runs from it toward @p b.
         * @param b              The second shape.
         * @param center_a       Body a's centre of mass, for the anchor frame.
         * @param orientation_a  Body a's orientation.
         * @param center_b       Body b's centre of mass.
         * @param orientation_b  Body b's orientation.
         * @param contact_offset Contacts are generated out to this separation (§7.6).
         * @param face_tolerance How flush a feature must be to count as part of the
         *                       contact face, in metres. Scale it to the shape.
         */
        template <typename T, typename ShapeA, typename ShapeB>
        inline ContactManifold<T> generate_convex_manifold(
            const ShapeA& a, const ShapeB& b, const Vector3T<T>& center_a,
            const QuaternionT<T>& orientation_a, const Vector3T<T>& center_b,
            const QuaternionT<T>& orientation_b, T contact_offset = T(0),
            T face_tolerance = T(1e-3)) noexcept
        {
            const ConvexContact<T> contact = collide_convex<T>(a, b);
            if (!contact.valid || contact.separation > contact_offset)
                return ContactManifold<T>{};

            const ContactFace<T> face_a = contact_face(a, contact.normal, face_tolerance);
            const ContactFace<T> face_b =
                contact_face(b, contact.normal * T(-1), face_tolerance);

            if (face_a.count < 2 || face_b.count < 2)
            {
                return make_point_manifold(contact.normal, contact.point_a, contact.point_b,
                                           contact.separation, center_a, orientation_a, center_b,
                                           orientation_b, make_feature_id(0, 0, 0, false));
            }

            // The reference face is the one lying more squarely across the normal;
            // clipping against the flatter of the two is what keeps the patch inside
            // the surface that actually supports it.
            const bool reference_is_b =
                std::abs(dot(face_b.normal, contact.normal)) > std::abs(dot(face_a.normal, contact.normal));
            const ContactFace<T>& reference = reference_is_b ? face_b : face_a;
            const ContactFace<T>& incident = reference_is_b ? face_a : face_b;

            ClippedPoint<T> clipped[max_clipped_points];
            std::size_t clipped_count = clip_against_face(incident, reference, face_tolerance, clipped);
            if (clipped_count == 0)
            {
                return make_point_manifold(contact.normal, contact.point_a, contact.point_b,
                                           contact.separation, center_a, orientation_a, center_b,
                                           orientation_b, make_feature_id(0, 0, 0, false));
            }

            // The reference face's plane, measured along the contact normal so the
            // separations are comparable with the one GJK reported.
            const T reference_offset = dot(contact.normal, reference.points[0].position);
            const T sign = reference_is_b ? T(-1) : T(1);

            ClippedPoint<T> kept[max_clipped_points];
            T separations[max_clipped_points];
            std::size_t kept_count = 0;
            for (std::size_t i = 0; i < clipped_count; ++i)
            {
                const T separation =
                    (dot(contact.normal, clipped[i].position) - reference_offset) * sign;
                if (separation > contact_offset)
                    continue;
                kept[kept_count] = clipped[i];
                separations[kept_count] = separation;
                ++kept_count;
            }
            if (kept_count == 0)
            {
                return make_point_manifold(contact.normal, contact.point_a, contact.point_b,
                                           contact.separation, center_a, orientation_a, center_b,
                                           orientation_b, make_feature_id(0, 0, 0, false));
            }

            std::size_t chosen[max_manifold_points];
            const std::size_t chosen_count =
                reduce_manifold_points(kept, separations, kept_count, chosen, face_tolerance);

            ContactManifold<T> manifold;
            manifold.normal = contact.normal;
            manifold.point_count = static_cast<std::uint8_t>(chosen_count);
            for (std::size_t i = 0; i < chosen_count; ++i)
            {
                const ClippedPoint<T>& point = kept[chosen[i]];
                const T separation = separations[chosen[i]];
                // The clipped point lies on the incident surface; its projection onto
                // the reference plane is where the reference surface is touched.
                const Vector3T<T> on_reference =
                    point.position - contact.normal * (separation * sign);
                const Vector3T<T>& on_a = reference_is_b ? point.position : on_reference;
                const Vector3T<T>& on_b = reference_is_b ? on_reference : point.position;

                ContactPoint<T>& out = manifold.points[i];
                out.anchor_a_local = to_local_anchor(center_a, orientation_a, on_a);
                out.anchor_b_local = to_local_anchor(center_b, orientation_b, on_b);
                out.separation = separation;
                out.normal_lambda = T(0);
                out.tangent_lambda[0] = T(0);
                out.tangent_lambda[1] = T(0);
                out.feature_id = make_feature_id(0, 0, point.vertex_id, reference_is_b);
            }
            return manifold;
        }

        /**
         * @brief The manifold between any convex shape and a static half-space plane.
         *
         * A plane is not a bounded convex shape — its support function runs off to
         * infinity — so it cannot go through GJK, and it does not need to: the
         * normal is known in advance, so the contact face is the only thing to
         * find, and extraction already does that. The result is that a plane needs
         * *one* routine rather than one per shape, and a new convex shape gains
         * ground contact from the same `contact_face()` overload that gave it
         * everything else.
         *
         * The plane is body `b`, so the normal runs from the shape toward the plane
         * and resolving pushes the shape out along `+plane.normal` — the pair
         * convention, rather than a sign rule that only plane contacts obey (§1.3
         * recorded what that costs). Its anchors are world points, because a
         * half-space has no frame to be local to.
         */
        template <typename T, typename Shape>
        inline ContactManifold<T> generate_convex_plane_manifold(
            const Shape& shape, const PlaneCollider<T>& plane, const Vector3T<T>& center,
            const QuaternionT<T>& orientation, T contact_offset = T(0),
            T face_tolerance = T(1e-3)) noexcept
        {
            const ContactFace<T> face = contact_face(shape, plane.normal * T(-1), face_tolerance);

            ClippedPoint<T> kept[max_clipped_points];
            T separations[max_clipped_points];
            std::size_t kept_count = 0;
            for (std::size_t i = 0; i < face.count; ++i)
            {
                const T separation = dot(plane.normal, face.points[i].position) - plane.offset;
                if (separation > contact_offset)
                    continue;
                kept[kept_count] = face.points[i];
                separations[kept_count] = separation;
                ++kept_count;
            }
            if (kept_count == 0)
                return ContactManifold<T>{};

            std::size_t chosen[max_manifold_points];
            const std::size_t chosen_count =
                reduce_manifold_points(kept, separations, kept_count, chosen, face_tolerance);

            ContactManifold<T> manifold;
            manifold.normal = plane.normal * T(-1);
            manifold.point_count = static_cast<std::uint8_t>(chosen_count);
            for (std::size_t i = 0; i < chosen_count; ++i)
            {
                const ClippedPoint<T>& point = kept[chosen[i]];
                const T separation = separations[chosen[i]];
                ContactPoint<T>& out = manifold.points[i];
                out.anchor_a_local = to_local_anchor(center, orientation, point.position);
                out.anchor_b_local = point.position - plane.normal * separation;
                out.separation = separation;
                out.normal_lambda = T(0);
                out.tangent_lambda[0] = T(0);
                out.tangent_lambda[1] = T(0);
                out.feature_id = make_feature_id(0, 0, point.vertex_id, false);
            }
            return manifold;
        }
    } // namespace Physics
} // namespace SushiEngine
