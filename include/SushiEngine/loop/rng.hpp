/**************************************************************************/
/* rng.hpp                                                                */
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
 * @file rng.hpp
 * @brief A determinism guard rail: seeded RNG state that lives in the world.
 *
 * docs/slop/SUSHILOOP.md requires that "the only source of nondeterminism is
 * player input" and that RNG state "lives inside the world like any other
 * component" — never a hidden global or `std::random_device`. `RngState` is a
 * trivially copyable xorshift128+ generator, so it can be stored as an ECS
 * component and snapshotted/rewound with everything else during rollback.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace loop
    {
        /**
         * @brief Deterministic xorshift128+ generator state, storable as a component.
         *
         * Trivially copyable, as `component_id<T>()` requires of every component.
         * Two `RngState`s seeded identically and advanced with the same sequence of
         * `next_u64()` calls always produce the same values — the property that
         * makes rollback-and-replay reproducible.
         */
        struct RngState
        {
            std::uint64_t s0 = 0;
            std::uint64_t s1 = 0;
        };

        /**
         * @brief Seeds an `RngState` from a single 64-bit seed.
         *
         * Runs the seed through SplitMix64 to spread its bits before handing them to
         * xorshift128+, which does not tolerate a low-entropy or all-zero seed.
         *
         * @param seed Any 64-bit value; 0 is valid and produces a well-mixed state.
         * @return An `RngState` ready for `next_u64()`.
         */
        inline RngState seed_rng(std::uint64_t seed) noexcept
        {
            auto split_mix64 = [](std::uint64_t& x) noexcept -> std::uint64_t
            {
                x += 0x9E3779B97F4A7C15ULL;
                std::uint64_t z = x;
                z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
                z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
                return z ^ (z >> 31);
            };
            std::uint64_t x = seed;
            RngState state;
            state.s0 = split_mix64(x);
            state.s1 = split_mix64(x);
            return state;
        }

        /**
         * @brief Advances @p state and returns the next pseudorandom value.
         * @param state The generator state to advance in place.
         * @return A uniformly distributed 64-bit value.
         */
        inline std::uint64_t next_u64(RngState& state) noexcept
        {
            std::uint64_t x = state.s0;
            const std::uint64_t y = state.s1;
            state.s0 = y;
            x ^= x << 23;
            x ^= x >> 17;
            x ^= y ^ (y >> 26);
            state.s1 = x;
            return x + y;
        }

        /**
         * @brief Advances @p state and returns a value uniformly distributed in [0, 1).
         * @param state The generator state to advance in place.
         * @return A double in [0, 1); the 53 mantissa bits of `next_u64()`.
         */
        inline double next_unit(RngState& state) noexcept
        {
            constexpr std::uint64_t MANTISSA_BITS = 53;
            constexpr double SCALE = 1.0 / (std::uint64_t(1) << MANTISSA_BITS);
            return double(next_u64(state) >> (64 - MANTISSA_BITS)) * SCALE;
        }
    } // namespace loop
} // namespace SushiEngine
