/**************************************************************************/
/* graph_coloring.hpp                                                    */
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

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/physics/solver/constraint_bodies.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A partition of constraints into independent colour batches.
         *
         * Each colour is a list of constraint indices in which no two constraints
         * share a body, so every constraint in a colour can be projected in parallel
         * without a write conflict. Colours are applied in order; consecutive colours
         * may share bodies and so must run in sequence.
         */
        using ColorBatches = std::vector<std::vector<std::uint32_t>>;

        /**
         * @brief Greedily colours constraints so each colour is conflict-free.
         *
         * Treats the constraints as *hyperedges* over the bodies and colours them:
         * each constraint takes the lowest colour not already used by another
         * constraint on **any** of its bodies. Greedy colouring uses at most one more
         * colour than the busiest body's constraint count, which is the natural
         * sequential depth of a Gauss-Seidel sweep. The result drives the solver, where
         * a colour becomes one parallel task and the runtime orders the colours.
         *
         * Hyperedges rather than edges, because a tetrahedron touches four particles
         * and its projections write to all of them. Colouring one as though it had two
         * endpoints leaves the other two unprotected, and a colour that no longer means
         * "no two constraints here share a body" is a colour whose parallel sweep races.
         * How many bodies a kind has is `constraint_bodies()`'s answer, so a two-body
         * constraint reaches exactly the code it always did.
         *
         * @tparam Constraint Any kind `constraint_bodies()` understands — the default
         *                    two-body shape (`a`, `b`) or one declaring `BODY_COUNT`.
         * @param constraints The constraints to colour.
         * @param body_count  Number of bodies (the index space the constraints address).
         * @return The constraint indices grouped by colour.
         */
        template <typename Constraint>
        ColorBatches color_constraints(const std::vector<Constraint>& constraints,
                                       std::size_t body_count)
        {
            ColorBatches batches;
            std::vector<std::vector<std::uint32_t>> colors_of_body(body_count);

            std::uint32_t bodies[MAXIMUM_CONSTRAINT_BODIES];
            for (std::uint32_t i = 0; i < constraints.size(); ++i)
            {
                const std::size_t count = constraint_bodies(constraints[i], bodies);

                // Mark the colours already taken on any endpoint, then pick the lowest
                // free one (which may be a brand-new colour). A body index past the
                // declared count is skipped rather than trusted: it would index outside
                // `colors_of_body`, and a constraint naming a body that does not exist
                // is a caller error that must not become a buffer overrun here.
                std::vector<bool> forbidden(batches.size() + 1, false);
                for (std::size_t k = 0; k < count; ++k)
                    if (bodies[k] < body_count)
                        for (std::uint32_t used : colors_of_body[bodies[k]])
                            forbidden[used] = true;

                std::uint32_t chosen = 0;
                while (chosen < forbidden.size() && forbidden[chosen]) ++chosen;

                if (chosen == batches.size()) batches.emplace_back();
                batches[chosen].push_back(i);
                for (std::size_t k = 0; k < count; ++k)
                    if (bodies[k] < body_count)
                        colors_of_body[bodies[k]].push_back(chosen);
            }

            return batches;
        }
    } // namespace Physics
} // namespace SushiEngine
