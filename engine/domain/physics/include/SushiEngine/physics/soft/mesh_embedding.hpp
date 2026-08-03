/**************************************************************************/
/* mesh_embedding.hpp                                                     */
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
 * @file mesh_embedding.hpp
 * @brief §8.6's per-tick kernel: where a render vertex is, given where the
 *        simulation vertices are.
 *
 * The whole binding is one line — `sum(weight[i] * tetrahedron_vertex[i])` — and
 * everything around it in this file exists to make that line correct at speed:
 * the cooked indices are rebased once at build time so the per-tick loop is four
 * indexed reads and three multiply-adds with no bounds test, and the surviving
 * bounds test happens where a bad binding is *found* rather than where it is
 * used.
 *
 * Barycentric embedding is chosen over the alternatives for a reason worth
 * writing down: it is **continuous across element boundaries**. A render vertex
 * on a face between two tetrahedra gets the same answer from either, because on
 * that face the weight of the two off-face vertices is zero. Nearest-vertex or
 * per-element rigid schemes are not continuous there, and the seam shows as a
 * crack in the render mesh that no amount of simulation accuracy removes.
 *
 * **Normals are recomputed, not carried.** A dented panel that keeps its rest
 * normals looks undented under any lighting model, so the deformed positions are
 * differentiated back into normals in the same pass. Per face, then summed per
 * vertex, then normalized — and the sum is *already* area-weighted, because the
 * unnormalized cross product of two triangle edges has twice the triangle's area
 * as its length. Normalizing each face normal before summing would be both extra
 * work and worse: it would let a sliver triangle shout as loudly as the large
 * face it sits against.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/cooking/soft_body_asset.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/soft/fem_fracture.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief One render vertex's four driving particles and their weights.
         *
         * The cooked `Cooking::SoftBodyBinding` names an *element*, which costs an
         * extra indirection per vertex per tick to resolve. This resolves it once,
         * at build time, into the four particle indices themselves — the same
         * trade `FEMTetrahedronT` makes by carrying `Dm^-1` rather than the rest
         * positions it was derived from.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct EmbeddedVertex
        {
            /**
             * @brief Four particle indices into the model's own array.
             *
             * An unbound vertex leaves them at `UNBOUND` rather than at zero. Zero is
             * a perfectly good particle index, so a default-constructed entry would
             * otherwise read as "bound to particle zero with all weights zero" and
             * quietly place the vertex at the world origin — a value that looks like
             * geometry. The sentinel makes the same entry fail `deform`'s bounds test
             * instead, which is where the rest-position fallback lives.
             */
            static constexpr std::uint32_t UNBOUND = 0xFFFFFFFFu;

            std::uint32_t particle[4] = {UNBOUND, UNBOUND, UNBOUND, UNBOUND};

            /** @brief Barycentric weights summing to one; negative where extrapolated. */
            T weight[4] = {T(0), T(0), T(0), T(0)};

            /**
             * @brief Which element this vertex was bound to, in the model's numbering.
             *
             * Carried alongside the resolved particles, not instead of them: the
             * per-tick loop wants the particles and never looks at this, and
             * §8.6 invariant 4 — fracture must not tear a hole in the render mesh —
             * wants this and cannot be answered by the particles. Fracture splits a
             * vertex by repointing *elements*, so a render vertex can only learn
             * which copy it now belongs to by asking the element it came from.
             *
             * `UNBOUND` once that element has been fractured away; see
             * @ref MeshEmbedding::follow_fracture for what that means.
             */
            std::uint32_t element = UNBOUND;
        };

        /**
         * @brief A render mesh bound to a simulation lattice, ready to be deformed.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class MeshEmbedding
        {
            public:
                /** @brief One entry per render vertex, in render-vertex order. */
                std::vector<EmbeddedVertex<T>> vertices;

                /**
                 * @brief The render mesh's triangles, three vertex indices each.
                 *
                 * Filled by whoever owns the render mesh, not by
                 * @ref build_mesh_embedding — the topology belongs to the mesh the
                 * asset was cooked *from*, and the physics asset stores the
                 * simulation surface rather than a second copy of it. Carried here
                 * so the normals pass has everything it needs in one object.
                 */
                std::vector<std::uint32_t> indices;

                /**
                 * @brief How many render vertices the cook failed to bind.
                 *
                 * §8.6 invariant 1. A vertex whose binding names an element outside
                 * the level is bound to nothing and is left at its rest position by
                 * @ref deform, which makes it visibly wrong rather than randomly
                 * wrong — a binding that read a neighbouring element instead would
                 * produce geometry, and geometry is the failure mode nobody notices.
                 */
                std::uint32_t unbound_count = 0;

                /** @brief Where every render vertex sits with the body at rest. */
                std::vector<Vector3T<T>> rest_positions;

                /**
                 * @brief Writes the deformed position of every render vertex.
                 *
                 * @param particles      The model's particles.
                 * @param particle_count How many; a binding past it is skipped.
                 * @param out            Receives one position per render vertex; must
                 *                       hold at least @ref vertices `.size()` of them.
                 */
                void deform(const RigidBodyT<T>* particles, std::size_t particle_count,
                            Vector3T<T>* out) const noexcept
                {
                    if (particles == nullptr || out == nullptr)
                        return;
                    for (std::size_t v = 0; v < vertices.size(); ++v)
                    {
                        const EmbeddedVertex<T>& binding = vertices[v];
                        Vector3T<T> position{T(0), T(0), T(0)};
                        bool usable = true;
                        for (int corner = 0; corner < 4; ++corner)
                        {
                            if (binding.particle[corner] >= particle_count)
                            {
                                usable = false;
                                break;
                            }
                            position = position + particles[binding.particle[corner]].position *
                                                      binding.weight[corner];
                        }
                        out[v] = usable ? position
                                        : (v < rest_positions.size() ? rest_positions[v]
                                                                     : Vector3T<T>{T(0), T(0), T(0)});
                    }
                }

                /** @brief How many render vertices this embedding drives. */
                std::size_t vertex_count() const noexcept { return vertices.size(); }

                /**
                 * @brief Re-resolves every binding after a fracture (§8.6 invariant 4).
                 *
                 * A duplicated simulation vertex inherits its parent's binding: the
                 * weights are untouched and only *which* particles they apply to
                 * changes, which is exactly what "inherits" means — the render vertex
                 * sits in the same place inside the same element and follows whichever
                 * side of the crack that element ended up on.
                 *
                 * A render vertex whose element was removed outright keeps the four
                 * particles it already had. They all still exist — fracture removes
                 * elements, never particles — so the vertex goes on being driven by
                 * them, and stretches if they end up on different fragments. That is
                 * what the hole in a broken object looks like, and it is strictly
                 * better than the two alternatives: freezing the vertex leaves a
                 * spike attached to nothing, and dropping it punches a hole through
                 * which the inside of the model is visible.
                 *
                 * @param model The body, after the fracture pass.
                 * @param remap What that pass reported.
                 */
                template <typename Model>
                void follow_fracture(const Model& model, const FEMFractureRemap& remap)
                {
                    for (EmbeddedVertex<T>& binding : vertices)
                    {
                        if (binding.element == EmbeddedVertex<T>::UNBOUND ||
                            binding.element >= remap.element.size())
                            continue;

                        const std::uint32_t moved = remap.element[binding.element];
                        if (moved == FEMFractureRemap::REMOVED)
                        {
                            binding.element = EmbeddedVertex<T>::UNBOUND;
                            continue;
                        }
                        binding.element = moved;
                        if (moved >= model.elements.size())
                            continue;
                        for (int corner = 0; corner < 4; ++corner)
                            binding.particle[corner] = model.elements[moved].vertex[corner];
                    }
                }
        };

        /**
         * @brief Recomputes vertex normals from deformed positions.
         *
         * Area-weighted by construction: each face contributes its unnormalized
         * edge cross product, whose magnitude is twice the face's area.
         *
         * A vertex whose faces cancel — two coincident triangles wound opposite
         * ways, a fully degenerate fan — is left with a zero normal rather than an
         * arbitrary one. Downstream that reads as unlit, which is a visible
         * question; an invented up vector would read as a lit surface facing the
         * wrong way, which is a silent wrong answer.
         *
         * @param positions     The deformed positions, one per vertex.
         * @param vertex_count  How many.
         * @param indices       Three vertex indices per triangle.
         * @param index_count   How many indices; a partial trailing triangle is ignored.
         * @param out           Receives one normal per vertex.
         */
        template <typename T>
        inline void compute_deformed_normals(const Vector3T<T>* positions,
                                             std::size_t vertex_count,
                                             const std::uint32_t* indices, std::size_t index_count,
                                             Vector3T<T>* out) noexcept
        {
            if (positions == nullptr || out == nullptr)
                return;
            for (std::size_t v = 0; v < vertex_count; ++v)
                out[v] = Vector3T<T>{T(0), T(0), T(0)};
            if (indices == nullptr)
                return;

            for (std::size_t i = 0; i + 2 < index_count; i += 3)
            {
                const std::uint32_t a = indices[i];
                const std::uint32_t b = indices[i + 1];
                const std::uint32_t c = indices[i + 2];
                if (a >= vertex_count || b >= vertex_count || c >= vertex_count)
                    continue;
                const Vector3T<T> face =
                    cross(positions[b] - positions[a], positions[c] - positions[a]);
                out[a] = out[a] + face;
                out[b] = out[b] + face;
                out[c] = out[c] + face;
            }

            for (std::size_t v = 0; v < vertex_count; ++v)
            {
                const T magnitude = length(out[v]);
                if (magnitude > T(1e-20))
                    out[v] = out[v] * (T(1) / magnitude);
            }
        }

        /**
         * @brief Builds an embedding from a cooked asset's render-vertex bindings.
         *
         * Resolves each binding's element to its four particles and rebases them to
         * the model's own numbering — `build_finite_element_model` subtracts the
         * level's `first_vertex` from every index it takes, and an embedding that
         * did not do the same would drive the right shape from the wrong particles.
         *
         * @param view       A validated soft-body asset.
         * @param level      Which level the render mesh is embedded in; normally zero.
         * @param origin     The world position the model was built at, added to the
         *                   rest positions so they match the particles.
         * @return The embedding; empty when @p view is invalid or @p level is out of range.
         */
        template <typename T>
        inline MeshEmbedding<T> build_mesh_embedding(const Cooking::SoftBodyAssetView& view,
                                                     std::uint32_t level,
                                                     const Vector3T<T>& origin)
        {
            MeshEmbedding<T> embedding;
            if (!view.valid || level >= view.level_count)
                return embedding;

            const Cooking::SoftBodyLevelRecord& record = view.levels[level];
            embedding.vertices.resize(view.binding_count);
            embedding.rest_positions.resize(view.binding_count);

            for (std::uint32_t v = 0; v < view.binding_count; ++v)
            {
                const Cooking::SoftBodyBinding& binding = view.bindings[v];
                EmbeddedVertex<T>& entry = embedding.vertices[v];

                const bool in_level =
                    binding.tetrahedron >= record.first_tetrahedron &&
                    binding.tetrahedron < record.first_tetrahedron + record.tetrahedron_count;
                if (!in_level)
                {
                    // Bound to nothing: every index left at zero and every weight at
                    // zero, so `deform` sees an unusable entry and falls back to rest.
                    ++embedding.unbound_count;
                    embedding.rest_positions[v] = origin;
                    continue;
                }

                const std::uint32_t* element =
                    view.tetrahedra + std::size_t(binding.tetrahedron) * 4;
                entry.element = binding.tetrahedron - record.first_tetrahedron;
                T weight[4];
                Cooking::read_binding_weights(binding, weight);
                Vector3T<T> rest{T(0), T(0), T(0)};
                bool usable = true;
                for (int corner = 0; corner < 4; ++corner)
                {
                    const std::uint32_t global = element[corner];
                    if (global < record.first_vertex ||
                        global - record.first_vertex >= record.vertex_count)
                    {
                        usable = false;
                        break;
                    }
                    entry.particle[corner] = global - record.first_vertex;
                    entry.weight[corner] = weight[corner];
                    const Vector3& source = view.vertices[global];
                    rest = rest + Vector3T<T>{T(source.x), T(source.y), T(source.z)} *
                                      entry.weight[corner];
                }

                if (!usable)
                {
                    ++embedding.unbound_count;
                    entry = EmbeddedVertex<T>{};
                    embedding.rest_positions[v] = origin;
                    continue;
                }
                embedding.rest_positions[v] = rest + origin;
            }
            return embedding;
        }
    } // namespace Physics
} // namespace SushiEngine
