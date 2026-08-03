/**************************************************************************/
/* fem_fracture.hpp                                                      */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                        */
/**************************************************************************/

#pragma once

/**
 * @file fem_fracture.hpp
 * @brief §9.5: removing an element once its stress exceeds `fracture_stress`.
 *
 * **What this file scopes in.** The three guard rails §9.5 names explicitly
 * — a per-tick fracture budget, a minimum fragment size, and a scene-level
 * cap — all deterministic and state-derived, plus the connectivity check a
 * minimum-fragment guard needs: after a candidate element is (tentatively)
 * removed, a union-find over the *surviving* elements' shared vertices finds
 * how large the resulting pieces are, so a removal that would leave a sliver
 * below the minimum is skipped rather than performed and regretted.
 *
 * **Vertex duplication along the crack (P6-G4).** §9.5's other clause — "its
 * shared vertices are duplicated along the crack surface, splitting the
 * topology" — needed to know which of a vertex's surviving incident elements
 * belong on *which side* of the crack. That looked like it needed cooked
 * face-adjacency data `FemTetrahedronT`'s flat vertex list does not carry, and
 * was left open for exactly that reason. It does not: a vertex's *star* — the
 * elements touching it — is small (a handful, not a mesh), and two elements in
 * it are on the same side precisely when they share a **face through that
 * vertex**, which is to say three vertices one of which is the vertex itself.
 * Counting shared vertices between the members of one small star answers that
 * directly, so the connected components of the star are computable from the
 * vertex lists alone, at a cost proportional to the crack rather than to the
 * body.
 *
 * Splitting is then: one particle per component beyond the first, each a copy
 * of the original, with each component's elements repointed at its own copy.
 * The copies start at the same position, so the split itself moves nothing —
 * the pieces separate because nothing holds them together any more, which is
 * what a crack is.
 *
 * **Only vertices of removed elements are considered.** A pristine mesh can
 * legitimately contain a vertex whose star is disconnected, and splitting those
 * would be this pass rewriting topology it was never asked about. The crack is
 * where elements went away, so that is where it looks.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <SushiEngine/physics/soft/fem_element.hpp>
#include <SushiEngine/physics/soft/finite_element_model.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief The deterministic limits a fracture pass is held to. */
        struct FemFractureBudget
        {
            /** @brief Elements allowed to fracture in a single tick. */
            std::uint32_t max_fractures_per_tick = 4;

            /**
             * @brief The smallest connected piece a fracture may leave behind,
             *        counted in surviving elements.
             *
             * A removal that would leave any resulting piece smaller than this
             * is skipped — the guard against a body dissolving into
             * ever-smaller slivers that §9.5 asks for.
             */
            std::uint32_t minimum_fragment_element_count = 1;

            /**
             * @brief The most elements this model may ever lose to fracture, total.
             *
             * The scene-level cap; independent of the per-tick one, since a
             * scene that fractures a little every tick for a long time is the
             * failure §9.5's own words call "how a physics engine dies," not
             * only a single tick that tries to fracture too much at once.
             */
            std::uint32_t maximum_total_fractures = 64;
        };

        /** @brief What one fracture pass over a model did. */
        struct FemFractureReport
        {
            /** @brief Elements actually removed this call. */
            std::uint32_t elements_removed = 0;

            /**
             * @brief Candidate elements over `fracture_stress` that were skipped.
             *
             * Skipped for one of two reasons: the per-tick or scene-level
             * budget was already spent, or removing it would have left a
             * fragment under `minimum_fragment_element_count`. Non-zero is not
             * an error — a body sitting at its fracture budget every tick for
             * a while is an authored choice about how fast it may come apart —
             * but a caller measuring "did the budget actually bind" reads it
             * from here.
             */
            std::uint32_t elements_skipped = 0;

            /**
             * @brief How many vertex-connected pieces the model's elements form
             *        after this call.
             *
             * One means the model (still) reads as a single connected body to
             * an XPBD solve, whatever it looks like geometrically. Above one
             * means this pass actually separated something — and, per this
             * file's header comment, that separation is only as real as the
             * vertices the two pieces no longer happen to share; a caller that
             * needs the pieces to be independently rigid must still duplicate
             * whatever vertices this report does not know to have split.
             */
            std::uint32_t fragment_count = 1;

            /**
             * @brief Particles added by splitting a vertex along the crack.
             *
             * Zero when a fracture removed elements but severed nothing that was
             * still holding two pieces together — a corner chipped off a block
             * removes elements and splits no vertex.
             */
            std::uint32_t vertices_duplicated = 0;
        };

        /**
         * @brief What a fracture pass did to the indices its consumers hold.
         *
         * Fracture is the one operation in the soft-body system that invalidates
         * *indices* rather than only values: elements go away and the ones after
         * them shift down, and particles are added. Anything holding those indices
         * — the render binding above all (§8.6 invariant 4) — needs to be told, and
         * being told is cheaper and far safer than rediscovering it by comparing
         * before and after.
         */
        struct FemFractureRemap
        {
            /** @brief What @ref element holds for an element that was removed. */
            static constexpr std::uint32_t REMOVED = 0xFFFFFFFFu;

            /**
             * @brief Per original element index: where it is now, or @ref REMOVED.
             *
             * Sized to the element count *before* the pass, so a caller can map an
             * index it captured before calling.
             */
            std::vector<std::uint32_t> element;

            /**
             * @brief Per particle added, the particle it was copied from.
             *
             * In creation order, so the particle added first is at
             * `original_particle_count`, the next at `+ 1`, and so on.
             */
            std::vector<std::uint32_t> duplicated_from;

            /** @brief How many particles the model had before the pass. */
            std::uint32_t original_particle_count = 0;
        };

        namespace detail
        {
            /** @brief Trivial union-find over particle indices, path-compressed. */
            class FemUnionFind
            {
                public:
                    explicit FemUnionFind(std::size_t count) : parent_(count)
                    {
                        for (std::size_t i = 0; i < count; ++i)
                            parent_[i] = std::uint32_t(i);
                    }

                    std::uint32_t find(std::uint32_t x) noexcept
                    {
                        while (parent_[x] != x)
                        {
                            parent_[x] = parent_[parent_[x]];
                            x = parent_[x];
                        }
                        return x;
                    }

                    void merge(std::uint32_t a, std::uint32_t b) noexcept
                    {
                        const std::uint32_t ra = find(a);
                        const std::uint32_t rb = find(b);
                        if (ra != rb)
                            parent_[ra] = rb;
                    }

                private:
                    std::vector<std::uint32_t> parent_;
            };

            /**
             * @brief Groups @p elements into connected pieces by shared vertices.
             *
             * @param elements     The elements to connect (indices into some
             *                     particle array; only the indices matter here).
             * @param particle_count How large the particle array is, so a
             *                     particle no element touches gets its own
             *                     trivial component rather than reading past
             *                     the union-find's storage.
             * @param out_element_component Receives, per element in @p elements
             *                     (same order, same size), which component
             *                     (an arbitrary but stable small integer) it
             *                     belongs to.
             * @return The number of distinct non-empty components among the
             *         elements (not counting untouched particles).
             */
            template <typename T>
            inline std::uint32_t connect_elements_by_shared_vertex(
                const std::vector<FemTetrahedronT<T>>& elements, std::size_t particle_count,
                std::vector<std::uint32_t>& out_element_component)
            {
                FemUnionFind union_find(particle_count);
                for (const FemTetrahedronT<T>& element : elements)
                    for (int i = 1; i < 4; ++i)
                        union_find.merge(element.vertex[0], element.vertex[i]);

                std::vector<std::uint32_t> root_to_component(particle_count, 0xFFFFFFFFu);
                std::uint32_t next_component = 0;
                out_element_component.assign(elements.size(), 0);
                for (std::size_t i = 0; i < elements.size(); ++i)
                {
                    const std::uint32_t root = union_find.find(elements[i].vertex[0]);
                    if (root_to_component[root] == 0xFFFFFFFFu)
                        root_to_component[root] = next_component++;
                    out_element_component[i] = root_to_component[root];
                }
                return next_component;
            }

            /**
             * @brief How many vertices two elements have in common.
             *
             * Sixteen comparisons on four-element arrays, which beats building a
             * set for a problem this size by more than it looks — a star has a
             * handful of elements and this runs inside a pairwise sweep over it.
             */
            template <typename T>
            inline int shared_vertex_count(const FemTetrahedronT<T>& a,
                                           const FemTetrahedronT<T>& b) noexcept
            {
                int shared = 0;
                for (int i = 0; i < 4; ++i)
                    for (int j = 0; j < 4; ++j)
                        if (a.vertex[i] == b.vertex[j])
                        {
                            ++shared;
                            break;
                        }
                return shared;
            }

            /**
             * @brief Whether two elements of one vertex's star lie on the same side of it.
             *
             * They do when they share a face containing the vertex — three common
             * vertices, one of which is the one being asked about. Three common
             * vertices in a tetrahedral mesh *is* a shared face, and requiring the
             * vertex to be among them is what makes this a statement about the star
             * rather than about the mesh: two elements can share a face that does
             * not touch this vertex only by not touching this vertex at all, which
             * the star excludes by construction.
             */
            template <typename T>
            inline bool joined_through_vertex(const FemTetrahedronT<T>& a,
                                              const FemTetrahedronT<T>& b) noexcept
            {
                return shared_vertex_count(a, b) >= 3;
            }
        } // namespace detail

        /**
         * @brief Rebuilds a model's boundary from the elements it has left.
         *
         * A face named by exactly one element is on the boundary; a face named by
         * two is interior. After a fracture the elements that used to cover a face
         * may be gone, so the surface the body presents to §9.6's collision and to
         * §8.6's binding is no longer the one it was cooked with — and a collision
         * surface that still describes the body's shape before it broke is worse
         * than none, because it is confidently wrong.
         *
         * Faces are emitted in the winding a positive-rest-volume tetrahedron
         * implies, so the boundary comes out consistently outward-facing without
         * consulting a single position.
         *
         * @param model The body; its `surface_indices` and `surface_vertices` are replaced.
         */
        template <typename T>
        inline void rebuild_soft_body_surface(FiniteElementModel<T>& model)
        {
            // The four faces of (v0, v1, v2, v3), each wound so its normal points
            // away from the vertex it omits.
            static const int FACE[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};

            struct Face
            {
                std::uint32_t sorted[3];
                std::uint32_t wound[3];

                bool operator<(const Face& other) const noexcept
                {
                    for (int i = 0; i < 3; ++i)
                        if (sorted[i] != other.sorted[i])
                            return sorted[i] < other.sorted[i];
                    return false;
                }
                bool same_face(const Face& other) const noexcept
                {
                    return sorted[0] == other.sorted[0] && sorted[1] == other.sorted[1] &&
                           sorted[2] == other.sorted[2];
                }
            };

            std::vector<Face> faces;
            faces.reserve(model.elements.size() * 4);
            for (const FemTetrahedronT<T>& element : model.elements)
                for (int f = 0; f < 4; ++f)
                {
                    Face face;
                    for (int i = 0; i < 3; ++i)
                        face.wound[i] = element.vertex[FACE[f][i]];
                    face.sorted[0] = face.wound[0];
                    face.sorted[1] = face.wound[1];
                    face.sorted[2] = face.wound[2];
                    // Three values sorted by three comparisons; std::sort on three
                    // elements is all overhead.
                    if (face.sorted[0] > face.sorted[1])
                        std::swap(face.sorted[0], face.sorted[1]);
                    if (face.sorted[1] > face.sorted[2])
                        std::swap(face.sorted[1], face.sorted[2]);
                    if (face.sorted[0] > face.sorted[1])
                        std::swap(face.sorted[0], face.sorted[1]);
                    faces.push_back(face);
                }

            std::sort(faces.begin(), faces.end());

            model.surface_indices.clear();
            for (std::size_t i = 0; i < faces.size();)
            {
                std::size_t j = i + 1;
                while (j < faces.size() && faces[j].same_face(faces[i]))
                    ++j;
                if (j - i == 1)
                    for (int k = 0; k < 3; ++k)
                        model.surface_indices.push_back(faces[i].wound[k]);
                i = j;
            }

            // Ascending and unique by construction, which is the order §9.6's
            // tests are required to walk it in (§12.1).
            std::vector<bool> on_surface(model.particles.size(), false);
            for (const std::uint32_t index : model.surface_indices)
                if (index < model.particles.size())
                    on_surface[index] = true;
            model.surface_vertices.clear();
            for (std::uint32_t i = 0; i < model.particles.size(); ++i)
                if (on_surface[i])
                    model.surface_vertices.push_back(i);
        }

        /**
         * @brief Duplicates every vertex a removal left holding two pieces together.
         *
         * For each vertex of a removed element, the surviving elements that still
         * touch it are grouped into face-connected components; every component past
         * the first gets its own copy of the particle and has its elements repointed
         * at it.
         *
         * **The copies share the original's position and velocity**, so the split
         * changes nothing about where the body is on the tick it happens — the
         * pieces drift apart over the following ticks because nothing constrains
         * them to each other any more, which is the only way a crack can open
         * without a visible jump.
         *
         * **Mass is divided, not copied.** Each component takes the share of the
         * original's mass that its element count represents, so a vertex split three
         * ways does not triple the body's weight. A pinned vertex (`inv_mass` zero)
         * stays pinned in every copy: it is held by the world, and the world does
         * not divide.
         *
         * @param model    The body; `particles` grows.
         * @param removed  The elements that were removed, whose vertices are the crack.
         * @param remap    Receives one entry per particle added.
         * @return How many particles were added.
         */
        template <typename T>
        inline std::uint32_t split_fractured_vertices(
            FiniteElementModel<T>& model, const std::vector<FemTetrahedronT<T>>& removed,
            FemFractureRemap& remap)
        {
            std::vector<std::uint32_t> crack_vertices;
            crack_vertices.reserve(removed.size() * 4);
            for (const FemTetrahedronT<T>& element : removed)
                for (int i = 0; i < 4; ++i)
                    crack_vertices.push_back(element.vertex[i]);
            std::sort(crack_vertices.begin(), crack_vertices.end());
            crack_vertices.erase(std::unique(crack_vertices.begin(), crack_vertices.end()),
                                 crack_vertices.end());

            std::uint32_t added = 0;
            std::vector<std::uint32_t> star;
            std::vector<std::uint32_t> component;
            for (const std::uint32_t vertex : crack_vertices)
            {
                if (vertex >= model.particles.size())
                    continue;

                star.clear();
                for (std::uint32_t e = 0; e < model.elements.size(); ++e)
                    for (int i = 0; i < 4; ++i)
                        if (model.elements[e].vertex[i] == vertex)
                        {
                            star.push_back(e);
                            break;
                        }
                if (star.size() < 2)
                    continue;

                // Connected components of the star under face-sharing, by flooding
                // from each unassigned member. The star is small enough that the
                // quadratic sweep this implies is cheaper than any index built to
                // avoid it.
                component.assign(star.size(), 0xFFFFFFFFu);
                std::uint32_t component_count = 0;
                std::vector<std::uint32_t> frontier;
                for (std::size_t seed = 0; seed < star.size(); ++seed)
                {
                    if (component[seed] != 0xFFFFFFFFu)
                        continue;
                    const std::uint32_t label = component_count++;
                    component[seed] = label;
                    frontier.clear();
                    frontier.push_back(std::uint32_t(seed));
                    while (!frontier.empty())
                    {
                        const std::uint32_t current = frontier.back();
                        frontier.pop_back();
                        for (std::size_t other = 0; other < star.size(); ++other)
                        {
                            if (component[other] != 0xFFFFFFFFu)
                                continue;
                            if (!detail::joined_through_vertex(model.elements[star[current]],
                                                               model.elements[star[other]]))
                                continue;
                            component[other] = label;
                            frontier.push_back(std::uint32_t(other));
                        }
                    }
                }
                if (component_count < 2)
                    continue; // still one piece here: nothing to split

                std::vector<std::uint32_t> component_size(component_count, 0);
                for (const std::uint32_t label : component)
                    ++component_size[label];

                const T inverse_mass = model.particles[vertex].inv_mass;
                const T star_size = T(star.size());
                for (std::uint32_t label = 0; label < component_count; ++label)
                {
                    const T share = T(component_size[label]) / star_size;
                    // Component zero keeps the original particle; the rest get copies.
                    std::uint32_t target = vertex;
                    if (label > 0)
                    {
                        target = std::uint32_t(model.particles.size());
                        model.particles.push_back(model.particles[vertex]);
                        remap.duplicated_from.push_back(vertex);
                        ++added;
                        for (std::size_t s = 0; s < star.size(); ++s)
                            if (component[s] == label)
                                for (int i = 0; i < 4; ++i)
                                    if (model.elements[star[s]].vertex[i] == vertex)
                                        model.elements[star[s]].vertex[i] = target;
                    }
                    if (inverse_mass > T(0))
                        model.particles[target].inv_mass = inverse_mass / share;
                }
            }
            return added;
        }

        /**
         * @brief Removes every element over `fracture_stress` this tick allows,
         *        within budget.
         *
         * Candidates are visited in ascending element-index order — a fixed,
         * state-derived order, per §0.5 — so the same stress field always
         * fractures the same elements first regardless of anything about how
         * the tick happened to be scheduled.
         *
         * @param model              The body; `elements` shrinks by however
         *                           many fracture this call.
         * @param budget             The deterministic limits to hold to.
         * @param total_fractured_so_far Running count of everything this
         *                           model has ever lost to fracture; updated
         *                           in place so the scene-level cap can be
         *                           enforced across many calls.
         * @param remap              Optional; receives the element and particle
         *                           index changes so a render binding can follow
         *                           (§8.6 invariant 4). Null when the caller has
         *                           nothing bound to the body.
         * @return A report of what happened.
         *
         * Not `noexcept`: this builds several vectors, and one that promised
         * otherwise would turn an allocation failure into a termination. (It
         * always did build them — the promise was wrong before the vertex
         * splitting below was added, not because of it.)
         */
        template <typename T>
        inline FemFractureReport apply_fem_fracture(FiniteElementModel<T>& model,
                                                    const FemFractureBudget& budget,
                                                    std::uint32_t& total_fractured_so_far,
                                                    FemFractureRemap* remap = nullptr)
        {
            FemFractureReport report;
            if (remap != nullptr)
            {
                remap->element.assign(model.elements.size(), 0);
                for (std::uint32_t i = 0; i < model.elements.size(); ++i)
                    remap->element[i] = i;
                remap->duplicated_from.clear();
                remap->original_particle_count = std::uint32_t(model.particles.size());
            }
            if (model.material.fracture_stress <= T(0))
                return report; // this material never fractures

            std::vector<std::uint32_t> candidates;
            for (std::uint32_t i = 0; i < model.elements.size(); ++i)
                if (model.elements[i].von_mises_stress > model.material.fracture_stress)
                    candidates.push_back(i);

            // Decided against *original* indices throughout, via this mask,
            // rather than mutating `model.elements` inside the loop: removing
            // one candidate would shift every later index down by one, and a
            // later candidate captured before that shift would then name the
            // wrong element. `model.elements` is rebuilt once, after every
            // decision is made, instead.
            std::vector<bool> marked_for_removal(model.elements.size(), false);

            for (const std::uint32_t candidate : candidates)
            {
                if (report.elements_removed >= budget.max_fractures_per_tick)
                    break; // per-tick budget spent; everything left is skipped, counted below
                if (total_fractured_so_far + report.elements_removed >= budget.maximum_total_fractures)
                    break; // scene-level cap reached, counting this call's own removals so far

                // Tentatively remove every element decided-on so far plus this
                // one candidate, and measure the resulting pieces before
                // committing — the minimum-fragment guard has to see the
                // *after* state to say whether this particular removal is the
                // one that goes too far.
                std::vector<FemTetrahedronT<T>> trial;
                trial.reserve(model.elements.size());
                for (std::size_t i = 0; i < model.elements.size(); ++i)
                    if (!marked_for_removal[i] && i != candidate)
                        trial.push_back(model.elements[i]);

                std::vector<std::uint32_t> component_of;
                const std::uint32_t component_count = detail::connect_elements_by_shared_vertex(
                    trial, model.particles.size(), component_of);

                std::vector<std::uint32_t> component_size(component_count, 0);
                for (const std::uint32_t component : component_of)
                    ++component_size[component];

                bool leaves_a_sliver = false;
                for (const std::uint32_t size : component_size)
                    if (size < budget.minimum_fragment_element_count)
                    {
                        leaves_a_sliver = true;
                        break;
                    }

                if (leaves_a_sliver)
                    continue; // this specific candidate is skipped; later ones still get a turn

                marked_for_removal[candidate] = true;
                ++report.elements_removed;
                report.fragment_count = component_count > 0 ? component_count : 1;
            }

            if (report.elements_removed > 0)
            {
                std::vector<FemTetrahedronT<T>> surviving;
                surviving.reserve(model.elements.size() - report.elements_removed);
                std::vector<FemTetrahedronT<T>> removed;
                removed.reserve(report.elements_removed);
                for (std::size_t i = 0; i < model.elements.size(); ++i)
                {
                    if (marked_for_removal[i])
                    {
                        removed.push_back(model.elements[i]);
                        if (remap != nullptr)
                            remap->element[i] = FemFractureRemap::REMOVED;
                        continue;
                    }
                    if (remap != nullptr)
                        remap->element[i] = std::uint32_t(surviving.size());
                    surviving.push_back(model.elements[i]);
                }
                model.elements = std::move(surviving);
                total_fractured_so_far += report.elements_removed;

                // The order here is the only one that works. Splitting rewrites
                // element vertex indices, so the surface has to be rebuilt after
                // it — a boundary computed first would name particles that the
                // split then moved elements away from, and the body would collide
                // against the shape it had before it broke.
                FemFractureRemap discard;
                report.vertices_duplicated =
                    split_fractured_vertices(model, removed, remap != nullptr ? *remap : discard);
                rebuild_soft_body_surface(model);
            }

            // Every candidate that did not end up removed was skipped, for
            // whatever mix of the budget/cap/sliver-guard reasons above —
            // counted once here rather than at each of the three sites that
            // could have caused it, so the two counts always sum to the
            // candidate count by construction.
            report.elements_skipped = std::uint32_t(candidates.size()) - report.elements_removed;

            return report;
        }
    } // namespace Physics
} // namespace SushiEngine
