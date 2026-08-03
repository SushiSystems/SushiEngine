/**************************************************************************/
/* hash.hpp                                                               */
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
 * @file hash.hpp
 * @brief The one string-identity hash the animation stack uses everywhere.
 *
 * Joint names, generic-track property names, and animation-event names are all
 * addressed by a 64-bit FNV-1a hash rather than a string at runtime — so masks, IK
 * targets, attachments, and event sinks look up by an integer that is trivially
 * copyable, device-visible, and stable across a build. The hash is computed once at
 * import (or from a string literal at a call site) and stored in the cooked blob;
 * strings never enter the simulation or evaluation domains.
 */

#include <cstddef>
#include <cstdint>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief A 64-bit name identity: the FNV-1a hash of a name string. */
        using NameHash = std::uint64_t;

        /** @brief The FNV-1a 64-bit offset basis. */
        constexpr NameHash FNV_OFFSET_BASIS = 0xcbf29ce484222325ull;

        /** @brief The FNV-1a 64-bit prime. */
        constexpr NameHash FNV_PRIME = 0x100000001b3ull;

        /**
         * @brief FNV-1a 64-bit hash of an explicit byte range.
         * @param data   First byte to hash.
         * @param length Number of bytes.
         * @return The hash of the range (@ref FNV_OFFSET_BASIS for an empty range).
         */
        constexpr NameHash fnv1a_64(const char* data, std::size_t length) noexcept
        {
            NameHash hash = FNV_OFFSET_BASIS;
            for (std::size_t i = 0; i < length; ++i)
            {
                hash ^= static_cast<NameHash>(static_cast<unsigned char>(data[i]));
                hash *= FNV_PRIME;
            }
            return hash;
        }

        /**
         * @brief FNV-1a 64-bit hash of a null-terminated C string.
         * @param text A null-terminated string (may be nullptr, treated as empty).
         * @return The hash of the string up to but excluding the terminator.
         */
        constexpr NameHash hash_name(const char* text) noexcept
        {
            if (text == nullptr)
                return FNV_OFFSET_BASIS;
            NameHash hash = FNV_OFFSET_BASIS;
            for (std::size_t i = 0; text[i] != '\0'; ++i)
            {
                hash ^= static_cast<NameHash>(static_cast<unsigned char>(text[i]));
                hash *= FNV_PRIME;
            }
            return hash;
        }
    } // namespace Animation
} // namespace SushiEngine
