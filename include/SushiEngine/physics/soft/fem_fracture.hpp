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
 * **What this file deliberately does not scope in yet.** §9.5's other
 * clause — "its shared vertices are duplicated along the crack surface,
 * splitting the topology" — is a harder problem than element removal: it
 * requires knowing which of a vertex's surviving incident elements belong on
 * *which side* of the crack, which in general needs face-adjacency data
 * (which element is across which of a tetrahedron's four faces) that
 * `FemTetrahedronT`'s flat vertex list does not carry. Until that adjacency
 * exists, a fracture that genuinely severs a body into two independently
 * falling pieces will still show the two pieces held together at whatever
 * vertices they happen to still share — correct removal, incomplete
 * separation. This is recorded here as a real, named gap rather than papered
 * over with an untested guess at the splitting rule, and
 * `FemFractureReport::fragment_count` is reported precisely so a caller can
 * see when a fracture produced more than one piece and still finds them
 * connected.
 */

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
        } // namespace detail

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
         * @return A report of what happened.
         */
        template <typename T>
        inline FemFractureReport apply_fem_fracture(FiniteElementModel<T>& model,
                                                     const FemFractureBudget& budget,
                                                     std::uint32_t& total_fractured_so_far) noexcept
        {
            FemFractureReport report;
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
                for (std::size_t i = 0; i < model.elements.size(); ++i)
                    if (!marked_for_removal[i])
                        surviving.push_back(model.elements[i]);
                model.elements = std::move(surviving);
                total_fractured_so_far += report.elements_removed;
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
