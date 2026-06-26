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
         * Treats the constraints as edges of a graph over the bodies and edge-colours
         * them: each constraint takes the lowest colour not already used by another
         * constraint on either of its two bodies. Greedy colouring uses at most one
         * more colour than the busiest body's constraint count, which is the natural
         * sequential depth of a Gauss-Seidel sweep. The result drives the solver,
         * where a colour becomes one parallel task and the runtime orders the colours.
         *
         * @tparam Constraint A constraint type exposing body indices `a` and `b`.
         * @param constraints The constraints to colour.
         * @param body_count  Number of bodies (the index space of `a` / `b`).
         * @return The constraint indices grouped by colour.
         */
        template <typename Constraint>
        ColorBatches color_constraints(const std::vector<Constraint>& constraints,
                                       std::size_t body_count)
        {
            ColorBatches batches;
            std::vector<std::vector<std::uint32_t>> colors_of_body(body_count);

            for (std::uint32_t i = 0; i < constraints.size(); ++i)
            {
                const Constraint& c = constraints[i];

                // Mark the colours already taken on either endpoint, then pick the
                // lowest free one (which may be a brand-new colour).
                std::vector<bool> forbidden(batches.size() + 1, false);
                for (std::uint32_t used : colors_of_body[c.a]) forbidden[used] = true;
                for (std::uint32_t used : colors_of_body[c.b]) forbidden[used] = true;

                std::uint32_t chosen = 0;
                while (chosen < forbidden.size() && forbidden[chosen]) ++chosen;

                if (chosen == batches.size()) batches.emplace_back();
                batches[chosen].push_back(i);
                colors_of_body[c.a].push_back(chosen);
                colors_of_body[c.b].push_back(chosen);
            }

            return batches;
        }
    } // namespace Physics
} // namespace SushiEngine
