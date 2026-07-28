/**************************************************************************/
/* material.hpp                                                           */
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
 * @file material.hpp
 * @brief Surface response: friction, restitution, density, and how two of them combine.
 *
 * A material is data, not behaviour. That is the point: the differences between a
 * tyre, an ice sheet, and a crate are parameters in a table indexed by a body's
 * `material_index`, so adding a surface kind never adds a branch anywhere in the
 * solver (§4.4 — the same reasoning that deletes the `is_cloth` type tag).
 *
 * Contacts are between *two* materials, and no single rule is right for every pair,
 * so the combine mode is itself a property of the material. Where the two materials
 * in a contact disagree about how to combine, the stricter mode wins, which is
 * stated once in @ref combine_friction rather than at every call site.
 */

#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief How the two materials in a contact produce one coefficient.
         *
         * Ordered by increasing strictness, which is what lets @ref combine_friction
         * resolve a disagreement by taking the larger enumerator.
         */
        enum class MaterialCombineMode : std::uint8_t
        {
            average = 0,  /**< The arithmetic mean; the neutral default. */
            minimum,      /**< The smaller of the two; the slipperier surface wins. */
            multiply,     /**< The product; two coefficients below one compound. */
            maximum       /**< The larger of the two; the grippier surface wins. */
        };

        /**
         * @brief The surface parameters of one body, referenced by index.
         *
         * Defaults describe an ordinary solid object: it does not bounce, it grips
         * about as well as wood on wood, and it neither resists rolling nor
         * spinning. A body that never had a material authored therefore behaves
         * plausibly rather than behaving like frictionless glass.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct PhysicsMaterialT
        {
            /** @brief Resistance to a contact that is not yet sliding. */
            T static_friction = T(0.6);

            /** @brief Resistance to a contact that is already sliding. */
            T dynamic_friction = T(0.5);

            /** @brief Bounce: 0 keeps no closing speed, 1 returns all of it. */
            T restitution = T(0);

            /** @brief Mass per unit volume, in kg/m^3; used to derive mass from a shape. */
            T density = T(1000);

            /**
             * @brief Resistance to rolling.
             *
             * Without it a sphere on a plane rolls for ever, because a perfectly
             * round contact has no lever to oppose it. Needed for wheels and for
             * spheres to settle at all.
             */
            T rolling_friction = T(0);

            /** @brief Resistance to spinning about the contact normal. */
            T spinning_friction = T(0);

            /** @brief How this material's friction combines with another's. */
            MaterialCombineMode friction_combine = MaterialCombineMode::average;

            /** @brief How this material's restitution combines with another's. */
            MaterialCombineMode restitution_combine = MaterialCombineMode::maximum;
        };

        /** @brief The boundary material: `PhysicsMaterialT` fixed to `Scalar`. */
        using PhysicsMaterial = PhysicsMaterialT<Scalar>;

        /**
         * @brief Combines two coefficients under @p mode.
         *
         * @tparam T The scalar element type.
         * @param a    The first material's coefficient.
         * @param b    The second material's coefficient.
         * @param mode The rule to apply.
         * @return The combined coefficient.
         */
        template <typename T>
        inline T combine_coefficient(T a, T b, MaterialCombineMode mode) noexcept
        {
            switch (mode)
            {
                case MaterialCombineMode::minimum:
                    return a < b ? a : b;
                case MaterialCombineMode::multiply:
                    return a * b;
                case MaterialCombineMode::maximum:
                    return a > b ? a : b;
                case MaterialCombineMode::average:
                default:
                    return (a + b) * T(0.5);
            }
        }

        /**
         * @brief The rule to use when two materials ask for different ones.
         *
         * Takes the stricter of the two, which the enumerator order encodes. Stated
         * here once so a contact between a default crate and a deliberately grippy
         * tyre resolves the same way everywhere, instead of depending on which body
         * happened to be first in the pair.
         *
         * @param a The first material's mode.
         * @param b The second material's mode.
         * @return The mode both should be combined under.
         */
        inline MaterialCombineMode stricter_mode(MaterialCombineMode a,
                                                 MaterialCombineMode b) noexcept
        {
            return static_cast<std::uint8_t>(a) >= static_cast<std::uint8_t>(b) ? a : b;
        }

        /**
         * @brief The friction coefficients of a contact between @p a and @p b.
         *
         * @tparam T The scalar element type.
         * @param a                The first body's material.
         * @param b                The second body's material.
         * @param static_friction  Receives the combined static friction.
         * @param dynamic_friction Receives the combined dynamic friction.
         */
        template <typename T>
        inline void combine_friction(const PhysicsMaterialT<T>& a,
                                     const PhysicsMaterialT<T>& b, T& static_friction,
                                     T& dynamic_friction) noexcept
        {
            const MaterialCombineMode mode =
                stricter_mode(a.friction_combine, b.friction_combine);
            static_friction = combine_coefficient(a.static_friction, b.static_friction, mode);
            dynamic_friction =
                combine_coefficient(a.dynamic_friction, b.dynamic_friction, mode);
        }

        /**
         * @brief The restitution of a contact between @p a and @p b.
         *
         * @tparam T The scalar element type.
         * @param a The first body's material.
         * @param b The second body's material.
         * @return The combined restitution.
         */
        template <typename T>
        inline T combine_restitution(const PhysicsMaterialT<T>& a,
                                     const PhysicsMaterialT<T>& b) noexcept
        {
            return combine_coefficient(
                a.restitution, b.restitution,
                stricter_mode(a.restitution_combine, b.restitution_combine));
        }
    } // namespace Physics
} // namespace SushiEngine
