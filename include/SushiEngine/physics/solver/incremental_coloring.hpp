/**************************************************************************/
/* incremental_coloring.hpp                                               */
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
 * @file incremental_coloring.hpp
 * @brief Colouring a constraint set that changes, one constraint at a time.
 *
 * `color_constraints()` colours a whole set from scratch. That is the right
 * primitive for a world that is built once, and the wrong one for a world where a
 * joint breaks, an element fractures, or a chunk streams in — recolouring
 * everything to add one constraint makes the constraint set's *structure* change
 * every tick, and structure changes are what force the solve graph to recompose.
 *
 * The incremental rule is the same greedy rule applied to a single constraint: take
 * the lowest colour not already used by either of its bodies. Doing it one
 * constraint at a time gives a colouring no worse than greedy over an arbitrary
 * insertion order, which is the guarantee greedy offers anyway.
 *
 * The per-body "which colours are taken" set is a 64-bit mask, so the lowest free
 * colour is one bitwise-or and one scan. This is why the colour count is bounded at
 * 64: a body's used-colour set has to fit a word for the query to stay O(1), and a
 * bound that is honest and checked beats a bound that is implicit and discovered.
 *
 * A colour bit is exactly "this body has a constraint of this colour", never a
 * count, because two constraints on one body can never share a colour — that is the
 * definition of the colouring. So removal clears a bit rather than decrementing.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Assigns conflict-free colours to constraints as they come and go.
         *
         * Holds only the colouring, not the constraints: the caller keeps the
         * descriptors, and this answers "which colour may this body pair use" and
         * "release the colour this pair held".
         */
        class IncrementalColoring
        {
            public:
                /** @brief The most colours this can ever track, set by the mask width. */
                static constexpr std::size_t MAXIMUM_COLORS = 64;

                /** @brief Returned by @ref assign when no colour is available. */
                static constexpr std::uint32_t NO_COLOR = 0xFFFFFFFFu;

                /**
                 * @brief Creates a colourer for a fixed body capacity and colour bound.
                 *
                 * @param body_capacity How many body slots may be referenced.
                 * @param color_limit   The colour ceiling; clamped to @ref MAXIMUM_COLORS.
                 */
                IncrementalColoring(std::size_t body_capacity, std::size_t color_limit)
                    : used_colors_(body_capacity, 0),
                      color_limit_(color_limit < MAXIMUM_COLORS ? color_limit
                                                                : MAXIMUM_COLORS)
                {
                }

                /**
                 * @brief Picks the lowest colour free on both @p a and @p b and takes it.
                 *
                 * @param a The first body's slot index.
                 * @param b The second body's slot index.
                 * @return The colour taken, or @ref NO_COLOR when both bodies together
                 *         already occupy every colour up to the limit.
                 */
                std::uint32_t assign(std::uint32_t a, std::uint32_t b) noexcept
                {
                    if (a >= used_colors_.size() || b >= used_colors_.size())
                        return NO_COLOR;

                    const std::uint64_t taken = used_colors_[a] | used_colors_[b];
                    for (std::uint32_t color = 0; color < color_limit_; ++color)
                    {
                        const std::uint64_t bit = std::uint64_t(1) << color;
                        if ((taken & bit) == 0)
                        {
                            used_colors_[a] |= bit;
                            used_colors_[b] |= bit;
                            if (color >= highest_used_)
                                highest_used_ = color + 1;
                            return color;
                        }
                    }
                    return NO_COLOR;
                }

                /**
                 * @brief Gives back the colour a constraint on @p a and @p b held.
                 *
                 * @ref highest_used deliberately does not shrink here. Recomputing it
                 * means scanning every body, and the number exists to size the solve
                 * graph — a value that fell and rose again would make the graph
                 * recompose for no gain. It is reset only by @ref recolor.
                 *
                 * @param a     The first body's slot index.
                 * @param b     The second body's slot index.
                 * @param color The colour to release.
                 */
                void release(std::uint32_t a, std::uint32_t b, std::uint32_t color) noexcept
                {
                    if (color >= MAXIMUM_COLORS)
                        return;
                    const std::uint64_t bit = ~(std::uint64_t(1) << color);
                    if (a < used_colors_.size())
                        used_colors_[a] &= bit;
                    if (b < used_colors_.size())
                        used_colors_[b] &= bit;
                }

                /**
                 * @brief Forgets every assignment, so a caller can colour from scratch.
                 *
                 * The scheduled full recolour of §6.4: incremental assignment leaves
                 * gaps as constraints come and go, and eventually a set that would
                 * fit in four colours is spread over twelve. Rebuilding is the fix,
                 * and it is the caller's to schedule because only the caller knows
                 * when a recomposition is affordable.
                 */
                void recolor() noexcept
                {
                    for (std::uint64_t& mask : used_colors_)
                        mask = 0;
                    highest_used_ = 0;
                }

                /**
                 * @brief How many colours have been used at least once since the last recolour.
                 *
                 * The number that sizes the solve graph. Monotone within a colouring,
                 * for the reason @ref release explains.
                 */
                std::size_t highest_used() const noexcept { return highest_used_; }

                /** @brief The colour ceiling this was constructed with. */
                std::size_t color_limit() const noexcept { return color_limit_; }

                /** @brief Whether body @p index currently holds a constraint of @p color. */
                bool holds(std::uint32_t index, std::uint32_t color) const noexcept
                {
                    if (index >= used_colors_.size() || color >= MAXIMUM_COLORS)
                        return false;
                    return (used_colors_[index] & (std::uint64_t(1) << color)) != 0;
                }

            private:
                std::vector<std::uint64_t> used_colors_;
                std::size_t color_limit_;
                std::size_t highest_used_ = 0;
        };
    } // namespace Physics
} // namespace SushiEngine
