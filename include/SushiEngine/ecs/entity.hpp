/**************************************************************************/
/* entity.hpp                                                             */
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

#include <cstdint>

namespace SushiEngine
{
    /**
     * @brief A stable handle to an entity: a slot index plus a generation.
     *
     * The index addresses a slot in the world's entity directory; the generation
     * distinguishes successive entities that reuse the same slot after a destroy.
     * A handle is valid only while the world's generation for its slot still
     * matches, so a stale handle to a destroyed entity is detected rather than
     * silently aliasing a new one. Trivially copyable and cheap to pass by value.
     */
    struct Entity
    {
        /** @brief Sentinel index marking a null handle. */
        static constexpr std::uint32_t K_INVALID = 0xFFFFFFFFu;

        std::uint32_t index = K_INVALID; /**< Slot in the entity directory. */
        std::uint32_t generation = 0;    /**< Reuse counter guarding staleness. */

        /** @brief True if this handle refers to no entity. */
        constexpr bool is_null() const noexcept { return index == K_INVALID; }

        /**
         * @brief Equality over both the slot and the generation.
         * @param other The handle to compare against.
         * @return True if both handles name the same live entity.
         */
        constexpr bool operator==(const Entity& other) const noexcept
        {
            return index == other.index && generation == other.generation;
        }

        /**
         * @brief Inequality, the negation of operator==.
         * @param other The handle to compare against.
         * @return True if the handles differ in slot or generation.
         */
        constexpr bool operator!=(const Entity& other) const noexcept
        {
            return !(*this == other);
        }
    };
} // namespace SushiEngine
