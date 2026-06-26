/**************************************************************************/
/* component.hpp                                                          */
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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

namespace SushiEngine
{
    /** @brief A process-stable numeric identity for a component type. */
    using ComponentId = std::uint32_t;

    namespace detail
    {
        /**
         * @brief Hands out the next free component id.
         * @return A unique id, increasing by one per distinct component type.
         */
        inline ComponentId next_component_id() noexcept
        {
            static ComponentId counter = 0;
            return counter++;
        }
    } // namespace detail

    /**
     * @brief The stable id assigned to component type @p T on first use.
     *
     * The id is allocated once, the first time this is instantiated for @p T, and
     * is the same for the rest of the process. It keys a component into archetype
     * storage and into a system's read/write set.
     *
     * @tparam T A trivially copyable component type.
     * @return The id for @p T.
     */
    template <typename T>
    ComponentId component_id() noexcept
    {
        static_assert(std::is_trivially_copyable_v<T>,
                      "A component must be trivially copyable for device storage.");
        static const ComponentId id = detail::next_component_id();
        return id;
    }

    /**
     * @brief Type-erased description of one component: its id and byte size.
     *
     * Archetype storage is laid out from a list of these, so it can allocate and
     * copy components without knowing their static types.
     */
    struct ComponentInfo
    {
        ComponentId id = 0;   /**< The component's stable id. */
        std::size_t size = 0; /**< sizeof the component, in bytes. */

        /**
         * @brief Builds the info for component type @p T.
         * @tparam T A trivially copyable component type.
         * @return The id and byte size of @p T.
         */
        template <typename T>
        static ComponentInfo of() noexcept
        {
            return ComponentInfo{component_id<T>(), sizeof(T)};
        }
    };

    /**
     * @brief A sorted, duplicate-free set of component ids identifying an archetype.
     *
     * Sorting makes two signatures comparable for identity and lets a query test
     * "contains all of these components" with a single ordered-subset check.
     */
    using Signature = std::vector<ComponentId>;

    /**
     * @brief Builds the sorted signature for the component types @p Ts.
     * @tparam Ts The component types in the set.
     * @return The ids of @p Ts, sorted ascending.
     */
    template <typename... Ts>
    Signature make_signature()
    {
        Signature s = {component_id<Ts>()...};
        std::sort(s.begin(), s.end());
        return s;
    }

    /**
     * @brief Builds the type-erased component list for the types @p Ts.
     * @tparam Ts The component types of an archetype.
     * @return One ComponentInfo per type, in the order written.
     */
    template <typename... Ts>
    std::vector<ComponentInfo> make_component_infos()
    {
        return {ComponentInfo::of<Ts>()...};
    }

    /**
     * @brief True if the sorted signature @p have contains every id in @p need.
     * @param have   An archetype's full, sorted signature.
     * @param need   The sorted set of ids a query requires.
     * @return True if @p need is a subset of @p have.
     */
    inline bool signature_contains(const Signature& have, const Signature& need) noexcept
    {
        return std::includes(have.begin(), have.end(), need.begin(), need.end());
    }

    /**
     * @brief Declares that a system reads component @p T (a const view).
     *
     * Passed as a template argument to a system; it contributes a read key to the
     * dependency set and hands the kernel a `const T*` column pointer.
     *
     * @tparam T The component type read.
     */
    template <typename T>
    struct Read
    {
        using type = T;                         /**< The component type. */
        static constexpr bool is_write = false; /**< Reads do not write. */
    };

    /**
     * @brief Declares that a system writes component @p T (mutable, read-modify-write).
     *
     * Contributes a write key to the dependency set and hands the kernel a `T*`
     * column pointer. A read-modify-write system (e.g. decaying a value in place)
     * declares Write; the write key is what orders it against other accessors.
     *
     * @tparam T The component type written.
     */
    template <typename T>
    struct Write
    {
        using type = T;                        /**< The component type. */
        static constexpr bool is_write = true; /**< Writes the component. */
    };
} // namespace SushiEngine
