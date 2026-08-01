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
 * the lowest colour not already used by *any* of its bodies. Doing it one constraint
 * at a time gives a colouring no worse than greedy over an arbitrary insertion
 * order, which is the guarantee greedy offers anyway.
 *
 * "Any of its bodies" rather than "either", because a constraint is not necessarily
 * an edge. A tetrahedron touches four particles and its projections write to all of
 * them, so the two-body form here is a convenience overload over the N-body one and
 * not the other way round — there is one rule, and the short spelling forwards to it.
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
                 * @brief Picks the lowest colour free on every one of @p bodies and takes it.
                 *
                 * The N-body form, and the one the two-body @ref assign forwards to. A
                 * constraint touching four particles must be free on all four: taking a
                 * colour that only two of them agree on would leave the other two sharing
                 * a colour with something else, which is exactly the race the colouring
                 * exists to prevent.
                 *
                 * Named apart from @ref assign rather than overloaded on it, because
                 * `assign(0, 1)` would be ambiguous: a literal `0` is both a body index
                 * and a null pointer constant, so the two candidates tie. A name is
                 * cheaper than every call site having to cast.
                 *
                 * @param bodies The constraint's body slot indices.
                 * @param count  How many there are.
                 * @return The colour taken, or @ref NO_COLOR when the bodies together
                 *         already occupy every colour up to the limit, when @p count is
                 *         zero, or when any index is out of range.
                 */
                std::uint32_t assign_bodies(const std::uint32_t* bodies,
                                            std::size_t count) noexcept
                {
                    // Zero bodies is refused rather than trivially satisfied: a colour
                    // handed to a constraint that constrains nothing would be a colour
                    // permanently taken on no body, and the caller has a bug either way.
                    if (bodies == nullptr || count == 0)
                        return NO_COLOR;

                    std::uint64_t taken = 0;
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        if (!tracks(bodies[i]))
                            return NO_COLOR;
                        taken |= used_colors_[bodies[i]];
                    }

                    for (std::uint32_t color = 0; color < color_limit_; ++color)
                    {
                        const std::uint64_t bit = std::uint64_t(1) << color;
                        if ((taken & bit) != 0)
                            continue;
                        for (std::size_t i = 0; i < count; ++i)
                            used_colors_[bodies[i]] |= bit;
                        if (color >= highest_used_)
                            highest_used_ = color + 1;
                        return color;
                    }
                    return NO_COLOR;
                }

                /**
                 * @brief Picks the lowest colour free on both @p a and @p b and takes it.
                 *
                 * The two-body shape, kept because most constraints have it and writing
                 * an array at every such call site would be noise. It forwards, so there
                 * is one rule rather than two that have to agree.
                 *
                 * @param a The first body's slot index.
                 * @param b The second body's slot index.
                 * @return The colour taken, or @ref NO_COLOR when both bodies together
                 *         already occupy every colour up to the limit.
                 */
                std::uint32_t assign(std::uint32_t a, std::uint32_t b) noexcept
                {
                    const std::uint32_t bodies[2] = {a, b};
                    return assign_bodies(bodies, 2);
                }

                /**
                 * @brief Takes @p color for a constraint on @p bodies, without choosing it.
                 *
                 * @ref assign_bodies picks the lowest free colour and takes it in one step,
                 * which is right when *any* free colour will do. It is not right when
                 * the caller has a second constraint on the choice — a colour whose
                 * storage band is full is free to the colourer and useless to the
                 * store — so the caller reads @ref mask_of, applies its own rule, and
                 * takes the colour it settled on through this.
                 *
                 * @param bodies The constraint's body slot indices.
                 * @param count  How many there are.
                 * @param color  The colour to take; must be free on every body.
                 * @return False when @p count is zero, any index is out of range, or
                 *         @p color is past the limit — in which case nothing was taken.
                 */
                bool take_bodies(const std::uint32_t* bodies, std::size_t count,
                                 std::uint32_t color) noexcept
                {
                    if (bodies == nullptr || count == 0 || color >= color_limit_)
                        return false;
                    // Checked in full before anything is written, so a rejected call
                    // leaves no colour half-taken — a colour marked on some of a
                    // constraint's bodies and not the others is worse than either
                    // outcome, because every later assignment would route around it.
                    for (std::size_t i = 0; i < count; ++i)
                        if (!tracks(bodies[i]))
                            return false;

                    const std::uint64_t bit = std::uint64_t(1) << color;
                    for (std::size_t i = 0; i < count; ++i)
                        used_colors_[bodies[i]] |= bit;
                    if (color >= highest_used_)
                        highest_used_ = color + 1;
                    return true;
                }

                /** @copydoc take_bodies */
                bool take(std::uint32_t a, std::uint32_t b, std::uint32_t color) noexcept
                {
                    const std::uint32_t bodies[2] = {a, b};
                    return take_bodies(bodies, 2, color);
                }

                /** @brief Whether @p index names a body slot this colourer tracks. */
                bool tracks(std::uint32_t index) const noexcept
                {
                    return index < used_colors_.size();
                }

                /**
                 * @brief Gives back the colour a constraint on @p bodies held.
                 *
                 * @ref highest_used deliberately does not shrink here. Recomputing it
                 * means scanning every body, and the number exists to size the solve
                 * graph — a value that fell and rose again would make the graph
                 * recompose for no gain. It is reset only by @ref recolor.
                 *
                 * @param bodies The constraint's body slot indices.
                 * @param count  How many there are.
                 * @param color  The colour to release.
                 */
                void release_bodies(const std::uint32_t* bodies, std::size_t count,
                                    std::uint32_t color) noexcept
                {
                    if (bodies == nullptr || color >= MAXIMUM_COLORS)
                        return;
                    const std::uint64_t bit = ~(std::uint64_t(1) << color);
                    // Out-of-range indices are skipped rather than refusing the whole
                    // call, the opposite of `take`: releasing as much as possible is
                    // always safe, while leaving a colour held on a body that no
                    // constraint uses would block that colour for ever.
                    for (std::size_t i = 0; i < count; ++i)
                        if (bodies[i] < used_colors_.size())
                            used_colors_[bodies[i]] &= bit;
                }

                /** @copydoc release_bodies */
                void release(std::uint32_t a, std::uint32_t b, std::uint32_t color) noexcept
                {
                    const std::uint32_t bodies[2] = {a, b};
                    release_bodies(bodies, 2, color);
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

                /**
                 * @brief The whole set of colours body @p index currently holds.
                 *
                 * Exposed so a *second*, shorter-lived colouring can be layered on
                 * top of this one rather than beside it. Contacts are recoloured
                 * every tick while joints and distance constraints are not (§6.3),
                 * and a contact that took a colour one of its bodies already holds
                 * would share a node with a constraint it shares a body with — which
                 * is precisely the race colouring exists to prevent. Reading the mask
                 * is how the contact colouring starts from what this one has already
                 * taken; one word, not a per-colour interrogation.
                 *
                 * @param index The body slot.
                 * @return Its used-colour mask, or zero when @p index is out of range.
                 */
                std::uint64_t mask_of(std::uint32_t index) const noexcept
                {
                    return index < used_colors_.size() ? used_colors_[index] : 0;
                }

                /**
                 * @brief The union of the colours every one of @p bodies holds.
                 *
                 * What a caller layering its own rule on top needs: a colour is free to a
                 * constraint only if it is free on *all* of the constraint's bodies, so
                 * the union is the question rather than any single mask. Out-of-range
                 * indices contribute nothing, which is the conservative direction — an
                 * untracked body constrains no colour.
                 *
                 * @param bodies The constraint's body slot indices.
                 * @param count  How many there are.
                 * @return The combined used-colour mask; zero for an empty list.
                 */
                std::uint64_t mask_of_all(const std::uint32_t* bodies,
                                          std::size_t count) const noexcept
                {
                    std::uint64_t taken = 0;
                    if (bodies == nullptr)
                        return taken;
                    for (std::size_t i = 0; i < count; ++i)
                        taken |= mask_of(bodies[i]);
                    return taken;
                }

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
