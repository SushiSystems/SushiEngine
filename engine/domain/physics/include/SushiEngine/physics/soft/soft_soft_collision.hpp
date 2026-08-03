/**************************************************************************/
/* soft_soft_collision.hpp                                                */
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
 * @file soft_soft_collision.hpp
 * @brief §9.6.2: two deformable surfaces against each other.
 *
 * Broad phase is each body's refitted hierarchy against the other's
 * (`soft_surface_hierarchy.hpp`), which reduces a quadratic triangle-pair
 * problem to the pairs that could plausibly touch. Narrow phase is the two
 * tests that between them cover every way two triangle meshes can meet:
 *
 * - **Vertex against triangle**, both ways round. This is the contact that
 *   stops a surface passing through another surface's interior.
 * - **Edge against edge**. Not redundant with the above and not an
 *   optimization: two surfaces can cross with no vertex of either inside the
 *   other, in an X, and only an edge pair sees it.
 *
 * Both produce the same `SoftContactConstraint`, so both are solved by one
 * projection.
 *
 * **Discrete or continuous, per body (§9.6.2).** The discrete test asks whether
 * the features are within a combined thickness *now*, once per tick, which is
 * correct and cheap for a body too thick to cross anything in one substep. A
 * body marked continuous instead re-runs its narrow phase **every substep**,
 * over the same tick's candidate pairs, using the coplanarity solve in
 * `geometry/continuous_proximity.hpp` — because the thing it is looking for is
 * a crossing that begins and ends within one substep, and a test that only
 * looks at the substep's endpoints cannot see one. That is the cost the flag
 * buys and the reason it is a flag.
 *
 * **Duplicates are removed rather than tolerated.** A vertex sits on several
 * triangles, so the same vertex-triangle pair is reached from several candidate
 * triangle pairs; left in, each copy would apply the same correction again and
 * the contact would be as many times too stiff as the vertex has neighbours.
 * The set is keyed by the *features* involved, sorted, and reduced — which also
 * fixes the order the contacts are solved in as a function of the surface's
 * topology rather than of the traversal (§12.1).
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/material.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/geometry/closest_point.hpp>
#include <SushiEngine/physics/geometry/continuous_proximity.hpp>
#include <SushiEngine/physics/soft/soft_body_collision.hpp>
#include <SushiEngine/physics/soft/soft_contact.hpp>
#include <SushiEngine/physics/soft/soft_surface_hierarchy.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief What produced a contact, so the same one is never counted twice.
         *
         * @ref kind separates the three ways a pair can be found, because a
         * vertex-triangle pair and an edge-edge pair can name the same four
         * particles and still be different contacts.
         */
        struct SoftContactKey
        {
            std::uint32_t kind = 0;
            std::uint32_t feature[4] = {0, 0, 0, 0};

            bool operator<(const SoftContactKey& other) const noexcept
            {
                if (kind != other.kind)
                    return kind < other.kind;
                for (int i = 0; i < 4; ++i)
                    if (feature[i] != other.feature[i])
                        return feature[i] < other.feature[i];
                return false;
            }

            bool operator==(const SoftContactKey& other) const noexcept
            {
                return kind == other.kind && feature[0] == other.feature[0] &&
                       feature[1] == other.feature[1] && feature[2] == other.feature[2] &&
                       feature[3] == other.feature[3];
            }
        };

        /**
         * @brief A vertex-against-triangle contact, if the two are within @p thickness.
         *
         * The normal keeps the vertex on the side it is already on: the direction
         * from the triangle's closest point out to the vertex. A proximity test has
         * no other way to decide a side — the face normal would push a vertex that
         * has already crossed further through — and a genuine crossing is what the
         * continuous test exists to catch before it becomes one.
         *
         * @tparam T The scalar element type.
         * @param vertex         The vertex's position.
         * @param corner         The triangle's three corner positions.
         * @param thickness      The two surfaces' combined thickness.
         * @param out            Receives the contact's normal, weights and rest distance;
         *                       particle indices and body mask are the caller's to fill.
         * @return False when the two are further apart than @p thickness, or when the
         *         triangle is degenerate enough to have no usable normal.
         */
        template <typename T>
        inline bool make_vertex_triangle_contact(const Vector3T<T>& vertex,
                                                 const Vector3T<T> corner[3], T thickness,
                                                 SoftContactConstraint<T>& out) noexcept
        {
            T weight[3];
            const Vector3T<T> closest =
                closest_point_on_triangle(vertex, corner[0], corner[1], corner[2], weight);
            const Vector3T<T> away = vertex - closest;
            const T distance = length(away);
            if (distance > thickness)
                return false;

            Vector3T<T> normal;
            if (distance > T(1e-12))
            {
                normal = away * (T(-1) / distance);
            }
            else
            {
                // Exactly on the surface: the face's own normal is the only
                // direction left, and either sign of it is equally right there.
                const Vector3T<T> face = cross(corner[1] - corner[0], corner[2] - corner[0]);
                const T face_length = length(face);
                if (face_length <= T(1e-18))
                    return false;
                normal = face * (T(1) / face_length);
            }

            out.normal = normal;
            out.rest_distance = thickness;
            out.weight[0] = T(-1);
            out.weight[1] = weight[0];
            out.weight[2] = weight[1];
            out.weight[3] = weight[2];
            return true;
        }

        /**
         * @brief An edge-against-edge contact, if the two are within @p thickness.
         *
         * @tparam T The scalar element type.
         * @param first     The first edge's two endpoint positions.
         * @param second    The second edge's two endpoint positions.
         * @param thickness The two surfaces' combined thickness.
         * @param out       Receives the contact's normal, weights and rest distance.
         * @return False when the edges are further apart than @p thickness, or when
         *         they are parallel and touching, which has no unique normal.
         */
        template <typename T>
        inline bool make_edge_edge_contact(const Vector3T<T> first[2], const Vector3T<T> second[2],
                                           T thickness, SoftContactConstraint<T>& out) noexcept
        {
            T along_first = 0;
            T along_second = 0;
            closest_points_on_edges(first[0], first[1], second[0], second[1], along_first,
                                    along_second);
            const Vector3T<T> point_first = first[0] + (first[1] - first[0]) * along_first;
            const Vector3T<T> point_second = second[0] + (second[1] - second[0]) * along_second;
            const Vector3T<T> away = point_first - point_second;
            const T distance = length(away);
            if (distance > thickness)
                return false;

            Vector3T<T> normal;
            if (distance > T(1e-12))
            {
                normal = away * (T(-1) / distance);
            }
            else
            {
                const Vector3T<T> crossing =
                    cross(first[1] - first[0], second[1] - second[0]);
                const T crossing_length = length(crossing);
                if (crossing_length <= T(1e-18))
                    return false;
                normal = crossing * (T(1) / crossing_length);
            }

            out.normal = normal;
            out.rest_distance = thickness;
            out.weight[0] = -(T(1) - along_first);
            out.weight[1] = -along_first;
            out.weight[2] = T(1) - along_second;
            out.weight[3] = along_second;
            return true;
        }

        /** @brief A candidate pair of surface triangles, one from each side. */
        struct SoftTrianglePair
        {
            std::uint32_t first = 0;
            std::uint32_t second = 0;
        };

        /** @brief A contact and the features that produced it, before repeats are removed. */
        template <typename T>
        struct SoftKeyedContact
        {
            SoftContactKey key;
            SoftContactConstraint<T> contact;
        };

        /**
         * @brief Sorts the collected contacts by feature and keeps one of each.
         *
         * @param keyed The collected contacts; emptied.
         * @param out   Receives the survivors, in feature order.
         */
        template <typename T>
        inline void reduce_soft_contacts(std::vector<SoftKeyedContact<T>>& keyed,
                                         std::vector<SoftContactConstraint<T>>& out)
        {
            std::sort(keyed.begin(), keyed.end(),
                      [](const SoftKeyedContact<T>& a, const SoftKeyedContact<T>& b)
                      { return a.key < b.key; });
            out.clear();
            for (std::size_t i = 0; i < keyed.size(); ++i)
            {
                if (i > 0 && keyed[i].key == keyed[i - 1].key)
                    continue;
                out.push_back(keyed[i].contact);
            }
            keyed.clear();
        }

        /**
         * @brief Whether two feature index lists name a particle in common.
         *
         * The topological exclusion §9.6.3 asks for, and only meaningful within one
         * body: a vertex is always "touching" the triangles it belongs to, and two
         * edges that meet at a corner are always touching there, so testing either
         * would produce a permanent contact holding the surface away from itself.
         *
         * @param first        The first feature's particle indices.
         * @param first_count  How many.
         * @param second       The second feature's particle indices.
         * @param second_count How many.
         * @return True when any index appears in both.
         */
        inline bool features_share_a_particle(const std::uint32_t* first, std::size_t first_count,
                                              const std::uint32_t* second,
                                              std::size_t second_count) noexcept
        {
            for (std::size_t i = 0; i < first_count; ++i)
                for (std::size_t j = 0; j < second_count; ++j)
                    if (first[i] == second[j])
                        return true;
            return false;
        }

        /**
         * @brief The proximity contacts between two surfaces, over a candidate list.
         *
         * Six vertex-triangle tests and nine edge-edge ones per candidate pair,
         * which between them cover every way two triangles can be near each other.
         * Shared by §9.6.2's two-body collider and §9.6.3's self-collider, which
         * differ in how their candidates are found and in nothing that happens
         * afterward.
         *
         * @tparam T The scalar element type.
         * @param first           The first surface.
         * @param position_first  Its particles' current positions, indexed by particle.
         * @param second          The second surface; the same as @p first for self-collision.
         * @param position_second Its positions.
         * @param candidates      The pairs the broad phase kept.
         * @param thickness       The two surfaces' combined thickness.
         * @param same_body       True when both surfaces are one body, which turns on
         *                        the topological exclusion and keeps every slot in
         *                        the first particle array.
         * @param keyed           Receives the contacts, with duplicates still in.
         */
        template <typename T>
        inline void collect_soft_contacts_discrete(const SoftSurfaceView<T>& first,
                                                   const Vector3T<T>* position_first,
                                                   const SoftSurfaceView<T>& second,
                                                   const Vector3T<T>* position_second,
                                                   const std::vector<SoftTrianglePair>& candidates,
                                                   T thickness, T margin, bool same_body,
                                                   std::vector<SoftKeyedContact<T>>& keyed)
        {
            for (const SoftTrianglePair& candidate : candidates)
            {
                const std::uint32_t* corner_first =
                    first.surface_indices + std::size_t(candidate.first) * 3;
                const std::uint32_t* corner_second =
                    second.surface_indices + std::size_t(candidate.second) * 3;

                Vector3T<T> point_first[3];
                Vector3T<T> point_second[3];
                for (int i = 0; i < 3; ++i)
                {
                    point_first[i] = position_first[corner_first[i]];
                    point_second[i] = position_second[corner_second[i]];
                }

                for (int i = 0; i < 3; ++i)
                {
                    if (same_body &&
                        features_share_a_particle(corner_first + i, 1, corner_second, 3))
                        continue;
                    SoftKeyedContact<T> keyed_contact;
                    if (!make_vertex_triangle_contact(point_first[i], point_second, margin,
                                                      keyed_contact.contact))
                        continue;
                    keyed_contact.contact.rest_distance = thickness;
                    keyed_contact.contact.particle[0] = corner_first[i];
                    for (int c = 0; c < 3; ++c)
                        keyed_contact.contact.particle[1 + c] = corner_second[c];
                    keyed_contact.contact.body_mask = same_body ? 0u : 0b1110u;
                    keyed_contact.key.kind = 0;
                    keyed_contact.key.feature[0] = corner_first[i];
                    keyed_contact.key.feature[1] = candidate.second;
                    keyed.push_back(keyed_contact);
                }

                for (int i = 0; i < 3; ++i)
                {
                    if (same_body &&
                        features_share_a_particle(corner_second + i, 1, corner_first, 3))
                        continue;
                    SoftKeyedContact<T> keyed_contact;
                    if (!make_vertex_triangle_contact(point_second[i], point_first, margin,
                                                      keyed_contact.contact))
                        continue;
                    keyed_contact.contact.rest_distance = thickness;
                    keyed_contact.contact.particle[0] = corner_second[i];
                    for (int c = 0; c < 3; ++c)
                        keyed_contact.contact.particle[1 + c] = corner_first[c];
                    // The vertex is in the second body now, so the mask is the
                    // mirror of the case above — and within one body there is no
                    // second array to name, so it stays zero.
                    keyed_contact.contact.body_mask = same_body ? 0u : 0b0001u;
                    // Within one body the two directions cannot collide, because a
                    // vertex and a triangle are numbered in the same space; across
                    // two bodies they can, so they are kept apart by kind.
                    keyed_contact.key.kind = same_body ? 0u : 1u;
                    keyed_contact.key.feature[0] = corner_second[i];
                    keyed_contact.key.feature[1] = candidate.first;
                    keyed.push_back(keyed_contact);
                }

                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                    {
                        const std::uint32_t edge_first_index[2] = {corner_first[i],
                                                                   corner_first[(i + 1) % 3]};
                        const std::uint32_t edge_second_index[2] = {corner_second[j],
                                                                    corner_second[(j + 1) % 3]};
                        if (same_body && features_share_a_particle(edge_first_index, 2,
                                                                   edge_second_index, 2))
                            continue;

                        const Vector3T<T> edge_first[2] = {point_first[i],
                                                           point_first[(i + 1) % 3]};
                        const Vector3T<T> edge_second[2] = {point_second[j],
                                                            point_second[(j + 1) % 3]};
                        SoftKeyedContact<T> keyed_contact;
                        if (!make_edge_edge_contact(edge_first, edge_second, margin,
                                                    keyed_contact.contact))
                            continue;
                        keyed_contact.contact.rest_distance = thickness;
                        keyed_contact.contact.particle[0] = edge_first_index[0];
                        keyed_contact.contact.particle[1] = edge_first_index[1];
                        keyed_contact.contact.particle[2] = edge_second_index[0];
                        keyed_contact.contact.particle[3] = edge_second_index[1];
                        keyed_contact.contact.body_mask = same_body ? 0u : 0b1100u;
                        keyed_contact.key.kind = 2;
                        keyed_contact.key.feature[0] =
                            edge_first_index[0] < edge_first_index[1] ? edge_first_index[0]
                                                                      : edge_first_index[1];
                        keyed_contact.key.feature[1] =
                            edge_first_index[0] < edge_first_index[1] ? edge_first_index[1]
                                                                      : edge_first_index[0];
                        keyed_contact.key.feature[2] =
                            edge_second_index[0] < edge_second_index[1] ? edge_second_index[0]
                                                                        : edge_second_index[1];
                        keyed_contact.key.feature[3] =
                            edge_second_index[0] < edge_second_index[1] ? edge_second_index[1]
                                                                        : edge_second_index[0];
                        keyed.push_back(keyed_contact);
                    }
            }
        }

        /** @brief The earliest coplanar instant of a moving vertex and triangle that is a real proximity. */
        template <typename T>
        inline bool swept_vertex_triangle_contact(const Vector3T<T> start[4],
                                                  const Vector3T<T> end[4], T thickness,
                                                  SoftContactConstraint<T>& out) noexcept
        {
            T time[3];
            const int count = vertex_triangle_coplanarity_times(start, end, time);
            for (int i = 0; i < count; ++i)
            {
                Vector3T<T> at[4];
                interpolate_positions(start, end, time[i], at);
                const Vector3T<T> corner[3] = {at[1], at[2], at[3]};
                if (make_vertex_triangle_contact(at[0], corner, thickness, out))
                    return true;
            }
            return false;
        }

        /** @brief The same, for two moving edges. */
        template <typename T>
        inline bool swept_edge_edge_contact(const Vector3T<T> start[4], const Vector3T<T> end[4],
                                            T thickness, SoftContactConstraint<T>& out) noexcept
        {
            T time[3];
            const int count = edge_edge_coplanarity_times(start, end, time);
            for (int i = 0; i < count; ++i)
            {
                Vector3T<T> at[4];
                interpolate_positions(start, end, time[i], at);
                const Vector3T<T> edge_first[2] = {at[0], at[1]};
                const Vector3T<T> edge_second[2] = {at[2], at[3]};
                if (make_edge_edge_contact(edge_first, edge_second, thickness, out))
                    return true;
            }
            return false;
        }

        /**
         * @brief The contacts formed by crossings *inside* one substep.
         *
         * The same six-plus-nine tests as the discrete collector, each run against
         * the coplanarity solve across the substep rather than against the poses at
         * one instant. A contact is built at the moment of crossing, which the
         * projection then enforces on the substep's end positions — reaching a
         * crossing at the moment it happens is the whole point, since by the end of
         * the substep the surface is through and no test of that pose alone can
         * tell it from having never touched.
         *
         * @tparam T The scalar element type.
         * @param first      The first surface, read for both its start and end poses.
         * @param second     The second surface; the same as @p first for self-collision.
         * @param candidates The pairs the broad phase kept this tick.
         * @param thickness  The two surfaces' combined thickness.
         * @param same_body  True when both surfaces are one body.
         * @param keyed      Receives the contacts, with duplicates still in.
         */
        template <typename T>
        inline void collect_soft_contacts_continuous(
            const SoftSurfaceView<T>& first, const SoftSurfaceView<T>& second,
            const std::vector<SoftTrianglePair>& candidates, T thickness, bool same_body,
            std::vector<SoftKeyedContact<T>>& keyed)
        {
            const auto span = [](const SoftSurfaceView<T>& view, std::uint32_t particle,
                                 Vector3T<T>& start, Vector3T<T>& end) noexcept
            {
                start = view.particles[particle].previous_position;
                end = view.particles[particle].position;
            };

            for (const SoftTrianglePair& candidate : candidates)
            {
                const std::uint32_t* corner_first =
                    first.surface_indices + std::size_t(candidate.first) * 3;
                const std::uint32_t* corner_second =
                    second.surface_indices + std::size_t(candidate.second) * 3;

                for (int direction = 0; direction < 2; ++direction)
                {
                    const SoftSurfaceView<T>& vertex_side = direction == 0 ? first : second;
                    const SoftSurfaceView<T>& triangle_side = direction == 0 ? second : first;
                    const std::uint32_t* vertex_corner =
                        direction == 0 ? corner_first : corner_second;
                    const std::uint32_t* triangle_corner =
                        direction == 0 ? corner_second : corner_first;

                    for (int i = 0; i < 3; ++i)
                    {
                        if (same_body &&
                            features_share_a_particle(vertex_corner + i, 1, triangle_corner, 3))
                            continue;

                        Vector3T<T> start[4];
                        Vector3T<T> end[4];
                        span(vertex_side, vertex_corner[i], start[0], end[0]);
                        for (int c = 0; c < 3; ++c)
                            span(triangle_side, triangle_corner[c], start[1 + c], end[1 + c]);

                        SoftKeyedContact<T> keyed_contact;
                        if (!swept_vertex_triangle_contact(start, end, thickness,
                                                           keyed_contact.contact))
                            continue;
                        keyed_contact.contact.particle[0] = vertex_corner[i];
                        for (int c = 0; c < 3; ++c)
                            keyed_contact.contact.particle[1 + c] = triangle_corner[c];
                        keyed_contact.contact.body_mask =
                            same_body ? 0u : (direction == 0 ? 0b1110u : 0b0001u);
                        keyed_contact.key.kind = same_body ? 0u : std::uint32_t(direction);
                        keyed_contact.key.feature[0] = vertex_corner[i];
                        keyed_contact.key.feature[1] =
                            direction == 0 ? candidate.second : candidate.first;
                        keyed.push_back(keyed_contact);
                    }
                }

                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                    {
                        const std::uint32_t edge_first_index[2] = {corner_first[i],
                                                                   corner_first[(i + 1) % 3]};
                        const std::uint32_t edge_second_index[2] = {corner_second[j],
                                                                    corner_second[(j + 1) % 3]};
                        if (same_body && features_share_a_particle(edge_first_index, 2,
                                                                   edge_second_index, 2))
                            continue;

                        Vector3T<T> start[4];
                        Vector3T<T> end[4];
                        span(first, edge_first_index[0], start[0], end[0]);
                        span(first, edge_first_index[1], start[1], end[1]);
                        span(second, edge_second_index[0], start[2], end[2]);
                        span(second, edge_second_index[1], start[3], end[3]);

                        SoftKeyedContact<T> keyed_contact;
                        if (!swept_edge_edge_contact(start, end, thickness, keyed_contact.contact))
                            continue;
                        keyed_contact.contact.particle[0] = edge_first_index[0];
                        keyed_contact.contact.particle[1] = edge_first_index[1];
                        keyed_contact.contact.particle[2] = edge_second_index[0];
                        keyed_contact.contact.particle[3] = edge_second_index[1];
                        keyed_contact.contact.body_mask = same_body ? 0u : 0b1100u;
                        keyed_contact.key.kind = 2;
                        keyed_contact.key.feature[0] =
                            edge_first_index[0] < edge_first_index[1] ? edge_first_index[0]
                                                                      : edge_first_index[1];
                        keyed_contact.key.feature[1] =
                            edge_first_index[0] < edge_first_index[1] ? edge_first_index[1]
                                                                      : edge_first_index[0];
                        keyed_contact.key.feature[2] =
                            edge_second_index[0] < edge_second_index[1] ? edge_second_index[0]
                                                                        : edge_second_index[1];
                        keyed_contact.key.feature[3] =
                            edge_second_index[0] < edge_second_index[1] ? edge_second_index[1]
                                                                        : edge_second_index[0];
                        keyed.push_back(keyed_contact);
                    }
            }
        }

        /**
         * @brief §9.6.2's collider: two soft surfaces, broad phase to contacts.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SoftSoftCollider final : public ISoftBodyPairCollider<T>
        {
            public:
                /** @brief The first body's surface; assign before @ref build. */
                SoftSurfaceView<T> first{};

                /** @brief The second body's surface; assign before @ref build. */
                SoftSurfaceView<T> second{};

                /** @brief The anti-jitter floor restitution is suppressed below; usually `2 * g * h`. */
                T restitution_threshold = 0;

                /**
                 * @brief Builds both hierarchies over the two surfaces' topology.
                 *
                 * Call once per pair, and again only if either surface's topology
                 * changes — which fracture (§9.5) is the one thing that does.
                 */
                void build()
                {
                    hierarchy_first_.build(first.surface_indices, first.index_count,
                                           first.particles, first.particle_count);
                    hierarchy_second_.build(second.surface_indices, second.index_count,
                                            second.particles, second.particle_count);
                    combine_friction(first.collision.surface, second.collision.surface,
                                     static_friction_, dynamic_friction_);
                    restitution_ =
                        combine_restitution(first.collision.surface, second.collision.surface);
                }

                /**
                 * @brief Refits both hierarchies, finds the candidate pairs, and — for a
                 *        discrete pair — the contacts.
                 *
                 * The bounds are widened by the combined thickness *and* by how far the
                 * fastest particle of either body can travel this tick, so a pair that
                 * will meet during the tick is already a candidate at the start of it.
                 * A continuous pair keeps only the candidates here; its contacts are
                 * found per substep, where the crossing it is looking for happens.
                 *
                 * @param dt The tick's duration, in seconds.
                 */
                void generate_contacts(T dt) override
                {
                    contacts_.clear();
                    candidates_.clear();
                    if (first.particles == nullptr || second.particles == nullptr)
                        return;

                    const T thickness =
                        first.collision.thickness + second.collision.thickness;
                    const T travel = fastest_travel(dt);
                    hierarchy_first_.refit(first.particles, thickness + travel);
                    hierarchy_second_.refit(second.particles, thickness + travel);

                    for_each_overlapping_triangle_pair(
                        hierarchy_first_, hierarchy_second_,
                        [&](std::uint32_t triangle_first, std::uint32_t triangle_second)
                        {
                            candidates_.push_back(
                                SoftTrianglePair{triangle_first, triangle_second});
                        });

                    // Built for *both* modes, not only the discrete one. The same
                    // widening the broad phase above already applies; leaving the
                    // narrow phase at the bare thickness was the bug that let a cube
                    // dropped on a cube fall straight through it, because the broad
                    // phase correctly offered every pair that could meet during the
                    // tick and the narrow phase then rejected all of them for not
                    // touching *yet*.
                    //
                    // The continuous path keeps this set and adds to it rather than
                    // replacing it, so turning the flag on can only ever find more.
                    // Replacing it would make `continuous` strictly worse than
                    // `discrete` for anything the speculative margin already caught —
                    // a flag that costs more and detects less is the one shape of bug
                    // nobody goes looking for.
                    collect_soft_contacts_discrete(
                        first, hierarchy_first_.positions().data(), second,
                        hierarchy_second_.positions().data(), candidates_, thickness,
                        thickness + travel, false, keyed_);
                    reduce_soft_contacts(keyed_, speculative_);
                    contacts_ = speculative_;
                }

                /** @brief Records every contact's closing speed before the substep integrates. */
                void capture_velocities() noexcept override
                {
                    const SoftParticlePair<T> source = particle_source();
                    for (SoftContactConstraint<T>& contact : contacts_)
                        capture_soft_contact_velocity(source, contact);
                }

                /**
                 * @brief Projects every contact; for a continuous pair, finds them first.
                 *
                 * @param substep_index Which substep this is; a discrete pair's
                 *                      accumulators are cleared on every one but the
                 *                      first, exactly as a rigid manifold's are.
                 * @param h             The substep duration, in seconds.
                 */
                void project_positions(std::size_t substep_index, T h) override
                {
                    (void)h;
                    const SoftParticlePair<T> source = particle_source();
                    if (continuous())
                    {
                        // The crossing this pair watches for is bounded by one
                        // substep, so the swept set is rebuilt from the substep's own
                        // start and end poses rather than carried from the tick's —
                        // and then *added* to the tick's speculative set, which is
                        // still the thing catching everything that merely approaches.
                        collect_soft_contacts_continuous(
                            first, second, candidates_,
                            first.collision.thickness + second.collision.thickness, false, keyed_);
                        reduce_soft_contacts(keyed_, swept_);
                        contacts_ = speculative_;
                        contacts_.insert(contacts_.end(), swept_.begin(), swept_.end());
                        for (SoftContactConstraint<T>& contact : contacts_)
                            capture_soft_contact_velocity(source, contact);
                    }
                    else if (substep_index > 0)
                    {
                        for (SoftContactConstraint<T>& contact : contacts_)
                        {
                            contact.normal_lambda = T(0);
                            contact.tangent_lambda[0] = T(0);
                            contact.tangent_lambda[1] = T(0);
                        }
                    }

                    for (SoftContactConstraint<T>& contact : contacts_)
                        project_soft_contact_position(source, contact, static_friction_);
                }

                /**
                 * @brief Applies dynamic friction and restitution at every contact.
                 * @param h The substep duration, in seconds (> 0).
                 */
                void solve_velocities(T h) override
                {
                    const SoftParticlePair<T> source = particle_source();
                    for (const SoftContactConstraint<T>& contact : contacts_)
                        solve_soft_contact_velocity(source, contact, dynamic_friction_,
                                                    restitution_, restitution_threshold, h);
                }

                /** @brief This tick's contacts, in feature order. */
                const std::vector<SoftContactConstraint<T>>& contacts() const noexcept
                {
                    return contacts_;
                }

                /** @brief How many triangle pairs the broad phase kept this tick. */
                std::size_t candidate_count() const noexcept
                {
                    return candidates_.size();
                }

            private:
                bool continuous() const noexcept
                {
                    return first.collision.continuous || second.collision.continuous;
                }

                SoftParticlePair<T> particle_source() const noexcept
                {
                    SoftParticlePair<T> source;
                    source.first = first.particles;
                    source.second = second.particles;
                    return source;
                }

                /** @brief How far the fastest particle of either body travels in @p dt. */
                T fastest_travel(T dt) const noexcept
                {
                    T fastest = 0;
                    const auto scan = [&fastest](const SoftSurfaceView<T>& view) noexcept
                    {
                        for (std::size_t i = 0; i < view.particle_count; ++i)
                        {
                            const T speed = length(view.particles[i].velocity);
                            if (speed > fastest)
                                fastest = speed;
                        }
                    };
                    scan(first);
                    scan(second);
                    return fastest * (dt > T(0) ? dt : T(0));
                }

                SoftSurfaceHierarchy<T> hierarchy_first_;
                SoftSurfaceHierarchy<T> hierarchy_second_;
                /** @brief The tick's speculative set, kept so the swept pass can add to it. */
                std::vector<SoftContactConstraint<T>> speculative_;
                /** @brief This substep's swept set, rebuilt every substep. */
                std::vector<SoftContactConstraint<T>> swept_;
                std::vector<SoftTrianglePair> candidates_;
                std::vector<SoftKeyedContact<T>> keyed_;
                std::vector<SoftContactConstraint<T>> contacts_;
                T static_friction_ = T(0.6);
                T dynamic_friction_ = T(0.5);
                T restitution_ = 0;
        };
    } // namespace Physics
} // namespace SushiEngine
