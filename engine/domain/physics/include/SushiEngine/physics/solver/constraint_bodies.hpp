/**************************************************************************/
/* constraint_bodies.hpp                                                  */
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
 * @file constraint_bodies.hpp
 * @brief How many bodies a constraint touches, and which — for any constraint kind.
 *
 * The colouring and the constraint store were written when every constraint had
 * exactly two endpoints, and they said so: `a` and `b`, everywhere. That was true of
 * distance constraints, joints and contacts, and it is not true of a tetrahedron,
 * which touches four particles and whose two projections write to all of them. A
 * four-body constraint coloured as if it were two-body is not slightly wrong — the
 * two ignored particles are unprotected, so a colour that is supposed to guarantee
 * no two constraints in it share a body no longer does, and the parallel sweep the
 * colouring exists to license races.
 *
 * So the machinery takes a *list* of bodies. This header is the one place that knows
 * how to get that list out of a constraint, and it is a customization point rather
 * than a switch: a two-body constraint is the default shape and needs no
 * cooperation, while a kind with more endpoints opts in by declaring
 * `BODY_COUNT` and a `vertex` array. Adding a fifth constraint kind touches this
 * file not at all.
 *
 * `BODY_COUNT` is required rather than inferred from the array's extent because an
 * inference would silently accept any type with a `vertex` member — including one
 * where `vertex` means something else entirely — and the failure would be a race,
 * which is the failure mode least likely to be found by looking.
 */

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief The most bodies one constraint may name; sizes every caller's stack buffer. */
        constexpr std::size_t MAXIMUM_CONSTRAINT_BODIES = 8;

        namespace Detail
        {
            /** @brief Whether @p T opted in to the multi-body shape by declaring `BODY_COUNT`. */
            template <typename T, typename = void>
            struct HasBodyCount : std::false_type
            {
            };

            template <typename T>
            struct HasBodyCount<T, std::void_t<decltype(T::BODY_COUNT)>> : std::true_type
            {
            };
        } // namespace Detail

        /**
         * @brief Writes the bodies @p constraint touches into @p out.
         *
         * @tparam Constraint Either the default two-body shape (members `a` and `b`) or a
         *                    kind declaring `BODY_COUNT` and a `vertex` array.
         * @param constraint The constraint to read.
         * @param out        Receives the body slot indices; must hold at least
         *                   @ref MAXIMUM_CONSTRAINT_BODIES entries.
         * @return How many were written.
         */
        template <typename Constraint>
        std::size_t constraint_bodies(const Constraint& constraint,
                                      std::uint32_t out[MAXIMUM_CONSTRAINT_BODIES]) noexcept
        {
            if constexpr (Detail::HasBodyCount<Constraint>::value)
            {
                constexpr std::size_t count =
                    Constraint::BODY_COUNT < MAXIMUM_CONSTRAINT_BODIES
                        ? std::size_t(Constraint::BODY_COUNT)
                        : MAXIMUM_CONSTRAINT_BODIES;
                for (std::size_t i = 0; i < count; ++i)
                    out[i] = constraint.vertex[i];
                return count;
            }
            else
            {
                out[0] = constraint.a;
                out[1] = constraint.b;
                return 2;
            }
        }
    } // namespace Physics
} // namespace SushiEngine
