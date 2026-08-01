/**************************************************************************/
/* soft_self_collision.hpp                                                */
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
 * @file soft_self_collision.hpp
 * @brief §9.6.3: a surface against itself, off unless a body asks for it.
 *
 * The broad phase is a **spatial hash at the collision thickness**, not the
 * hierarchy §9.6.2 uses, and the difference is not an implementation
 * preference. A hierarchy is good at proving that two *separate* things are far
 * apart; a body's surface against itself has one root box containing both sides
 * of every query, so the descent has nothing to prune with until it is deep in
 * the tree — the very case a hierarchy is worst at. A uniform grid at the
 * thickness is the opposite: every cell is exactly as large as the distance a
 * contact can span, so a triangle's own cells are the complete list of places a
 * partner can be, and it costs a hash per triangle rather than a traversal.
 *
 * **Off by default**, per §9.6: it is the expensive test, and it is the only
 * one whose cost a body pays without anything else in the scene being nearby.
 *
 * **Topological neighbours are excluded** (`features_share_a_particle`). A
 * vertex is permanently touching the triangles it belongs to, and two edges
 * that meet at a corner are permanently touching there; without the exclusion
 * every surface would spend its whole existence pushing itself apart at every
 * corner it has.
 *
 * The hash is a **sorted array**, not a hash map. Both find a cell's occupants,
 * but only one of them enumerates them in an order that is a function of the
 * simulation rather than of insertion history and bucket layout — and §0.5 is
 * not satisfied by an answer that is merely usually the same.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/material.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/soft/soft_body_collision.hpp>
#include <SushiEngine/physics/soft/soft_contact.hpp>
#include <SushiEngine/physics/soft/soft_soft_collision.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A uniform grid over a surface's triangles, rebuilt every tick.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SoftSurfaceGrid
        {
            public:
                /** @brief One triangle's membership of one cell. */
                struct Entry
                {
                    std::uint64_t cell = 0;
                    std::uint32_t triangle = 0;

                    bool operator<(const Entry& other) const noexcept
                    {
                        if (cell != other.cell)
                            return cell < other.cell;
                        return triangle < other.triangle;
                    }
                };

                /**
                 * @brief Rebuilds the grid from the surface's current positions.
                 *
                 * A triangle joins every cell its bounds touch, widened by the
                 * thickness, so a query needs to look at one cell rather than at a
                 * cell and all twenty-six of its neighbours.
                 *
                 * @param positions       The surface's particle positions.
                 * @param surface_indices Three particle indices per triangle.
                 * @param triangle_count  How many triangles.
                 * @param cell_size       The grid's cell edge; the collision thickness.
                 */
                void build(const Vector3T<T>* positions, const std::uint32_t* surface_indices,
                           std::uint32_t triangle_count, T cell_size)
                {
                    entries_.clear();
                    cell_size_ = cell_size > T(0) ? cell_size : T(1);
                    if (positions == nullptr || surface_indices == nullptr)
                        return;

                    for (std::uint32_t triangle = 0; triangle < triangle_count; ++triangle)
                    {
                        const std::uint32_t* corner = surface_indices + std::size_t(triangle) * 3;
                        Vector3T<T> low = positions[corner[0]];
                        Vector3T<T> high = low;
                        for (int i = 1; i < 3; ++i)
                        {
                            const Vector3T<T>& point = positions[corner[i]];
                            low = Vector3T<T>{low.x < point.x ? low.x : point.x,
                                              low.y < point.y ? low.y : point.y,
                                              low.z < point.z ? low.z : point.z};
                            high = Vector3T<T>{high.x > point.x ? high.x : point.x,
                                               high.y > point.y ? high.y : point.y,
                                               high.z > point.z ? high.z : point.z};
                        }

                        const std::int64_t first[3] = {cell_of(low.x - cell_size_),
                                                       cell_of(low.y - cell_size_),
                                                       cell_of(low.z - cell_size_)};
                        const std::int64_t last[3] = {cell_of(high.x + cell_size_),
                                                      cell_of(high.y + cell_size_),
                                                      cell_of(high.z + cell_size_)};
                        for (std::int64_t x = first[0]; x <= last[0]; ++x)
                            for (std::int64_t y = first[1]; y <= last[1]; ++y)
                                for (std::int64_t z = first[2]; z <= last[2]; ++z)
                                    entries_.push_back(Entry{pack(x, y, z), triangle});
                    }
                    std::sort(entries_.begin(), entries_.end());
                }

                /**
                 * @brief Every pair of triangles sharing a cell, each pair once.
                 *
                 * A triangle spanning several cells meets the same partner in each of
                 * them, so the pairs are collected and then reduced rather than
                 * emitted as found.
                 *
                 * @param out Receives the pairs, ascending, with `first < second`.
                 */
                void collect_pairs(std::vector<SoftTrianglePair>& out) const
                {
                    out.clear();
                    std::size_t begin = 0;
                    while (begin < entries_.size())
                    {
                        std::size_t end = begin + 1;
                        while (end < entries_.size() && entries_[end].cell == entries_[begin].cell)
                            ++end;

                        for (std::size_t i = begin; i < end; ++i)
                            for (std::size_t j = i + 1; j < end; ++j)
                                out.push_back(SoftTrianglePair{entries_[i].triangle,
                                                               entries_[j].triangle});
                        begin = end;
                    }

                    std::sort(out.begin(), out.end(),
                              [](const SoftTrianglePair& a, const SoftTrianglePair& b)
                              {
                                  if (a.first != b.first)
                                      return a.first < b.first;
                                  return a.second < b.second;
                              });
                    out.erase(std::unique(out.begin(), out.end(),
                                          [](const SoftTrianglePair& a, const SoftTrianglePair& b)
                                          {
                                              return a.first == b.first && a.second == b.second;
                                          }),
                              out.end());
                }

                /** @brief How many triangle-cell memberships the grid holds. */
                std::size_t entry_count() const noexcept
                {
                    return entries_.size();
                }

            private:
                std::int64_t cell_of(T coordinate) const noexcept
                {
                    return static_cast<std::int64_t>(std::floor(coordinate / cell_size_));
                }

                /**
                 * @brief One cell's three indices as a single key.
                 *
                 * Twenty-one bits each, biased to unsigned so the ordering is by
                 * cell rather than by sign. A body spanning more than a million
                 * cells on an axis would wrap, which at any usable thickness is a
                 * body the size of a continent.
                 */
                static std::uint64_t pack(std::int64_t x, std::int64_t y, std::int64_t z) noexcept
                {
                    const std::uint64_t bias = 1u << 20;
                    const std::uint64_t mask = (std::uint64_t(1) << 21) - 1;
                    const std::uint64_t ux = (std::uint64_t(x) + bias) & mask;
                    const std::uint64_t uy = (std::uint64_t(y) + bias) & mask;
                    const std::uint64_t uz = (std::uint64_t(z) + bias) & mask;
                    return ux | (uy << 21) | (uz << 42);
                }

                std::vector<Entry> entries_;
                T cell_size_ = 1;
        };

        /**
         * @brief §9.6.3's collider: one soft body's surface against itself.
         *
         * A single-body collider, so it plugs into the same seam a distance-field
         * collider does and a body can carry both through `SoftBodyColliderSet`.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SoftSelfCollider final : public ISoftBodyCollider<T>
        {
            public:
                /** @brief The body's own surface; assign before stepping. */
                SoftSurfaceView<T> surface{};

                /** @brief The anti-jitter floor restitution is suppressed below; usually `2 * g * h`. */
                T restitution_threshold = 0;

                /**
                 * @brief Rebuilds the grid and, for a discrete body, this tick's contacts.
                 *
                 * Does nothing at all when the body has not asked for self-collision,
                 * which is what makes the feature free for the bodies that do not
                 * want it rather than merely cheap.
                 *
                 * @param particles      The body's particles at the tick's start.
                 * @param particle_count How many.
                 */
                void generate_contacts(const RigidBodyT<T>* particles,
                                       std::size_t particle_count, T dt) override
                {
                    contacts_.clear();
                    candidates_.clear();
                    if (!surface.collision.self_collision || particles == nullptr ||
                        surface.surface_indices == nullptr)
                        return;

                    // The view tracks whatever array the body just handed over, so a
                    // model whose particles moved in memory — which fracture (§9.5)
                    // does — cannot leave the continuous path reading a freed one.
                    surface.particles = const_cast<RigidBodyT<T>*>(particles);
                    surface.particle_count = particle_count;

                    positions_.assign(particle_count, Vector3T<T>{T(0), T(0), T(0)});
                    for (std::size_t i = 0; i < particle_count; ++i)
                        positions_[i] = particles[i].position;

                    const T thickness = surface.collision.thickness * T(2);
                    // A body's own halves close on each other as fast as anything
                    // else does, so the grid and the narrow phase are both widened by
                    // how far a particle can travel this tick. Without it a falling
                    // half passes through the half it was going to land on, having
                    // been offered no contact at any point where it was close enough
                    // to be given one.
                    T fastest = 0;
                    for (std::size_t i = 0; i < particle_count; ++i)
                    {
                        const T speed = length(particles[i].velocity);
                        if (speed > fastest)
                            fastest = speed;
                    }
                    const T margin = thickness + (dt > T(0) ? fastest * dt : T(0));

                    grid_.build(positions_.data(), surface.surface_indices,
                                std::uint32_t(surface.index_count / 3), margin);
                    grid_.collect_pairs(candidates_);

                    combine_friction(surface.collision.surface, surface.collision.surface,
                                     static_friction_, dynamic_friction_);
                    restitution_ = combine_restitution(surface.collision.surface,
                                                       surface.collision.surface);

                    if (!surface.collision.continuous)
                    {
                        collect_soft_contacts_discrete(surface, positions_.data(), surface,
                                                       positions_.data(), candidates_, thickness,
                                                       margin, true, keyed_);
                        reduce_soft_contacts(keyed_, contacts_);
                    }
                }

                /** @brief Records every contact's closing speed before the substep integrates. */
                void capture_velocities(const RigidBodyT<T>* particles) noexcept override
                {
                    const SoftParticleArray<T> source = source_of(particles);
                    for (SoftContactConstraint<T>& contact : contacts_)
                        capture_soft_contact_velocity(source, contact);
                }

                /**
                 * @brief Projects the body's self-contacts; for a continuous body, finds them first.
                 *
                 * @param particles     The body's particles; positions updated in place.
                 * @param substep_index Which substep this is.
                 * @param h             The substep duration, in seconds.
                 */
                void project_positions(RigidBodyT<T>* particles, std::size_t substep_index,
                                       T h) override
                {
                    (void)h;
                    if (!surface.collision.self_collision)
                        return;

                    const SoftParticleArray<T> source = source_of(particles);
                    if (surface.collision.continuous)
                    {
                        collect_soft_contacts_continuous(surface, surface, candidates_,
                                                         surface.collision.thickness * T(2), true,
                                                         keyed_);
                        reduce_soft_contacts(keyed_, contacts_);
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
                 * @brief Applies dynamic friction and restitution at every self-contact.
                 * @param particles The body's particles; velocities updated in place.
                 * @param h         The substep duration, in seconds (> 0).
                 */
                void solve_velocities(RigidBodyT<T>* particles, T h) override
                {
                    const SoftParticleArray<T> source = source_of(particles);
                    for (const SoftContactConstraint<T>& contact : contacts_)
                        solve_soft_contact_velocity(source, contact, dynamic_friction_,
                                                    restitution_, restitution_threshold, h);
                }

                /** @brief This tick's self-contacts, in feature order. */
                const std::vector<SoftContactConstraint<T>>& contacts() const noexcept
                {
                    return contacts_;
                }

                /** @brief How many triangle pairs shared a cell this tick. */
                std::size_t candidate_count() const noexcept
                {
                    return candidates_.size();
                }

            private:
                static SoftParticleArray<T> source_of(RigidBodyT<T>* particles) noexcept
                {
                    SoftParticleArray<T> source;
                    source.particles = particles;
                    return source;
                }

                static SoftParticleArray<T> source_of(const RigidBodyT<T>* particles) noexcept
                {
                    SoftParticleArray<T> source;
                    source.particles = const_cast<RigidBodyT<T>*>(particles);
                    return source;
                }

                SoftSurfaceGrid<T> grid_;
                std::vector<Vector3T<T>> positions_;
                std::vector<SoftTrianglePair> candidates_;
                std::vector<SoftKeyedContact<T>> keyed_;
                std::vector<SoftContactConstraint<T>> contacts_;
                T static_friction_ = T(0.6);
                T dynamic_friction_ = T(0.5);
                T restitution_ = 0;
        };
    } // namespace Physics
} // namespace SushiEngine
