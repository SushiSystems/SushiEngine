/**************************************************************************/
/* random.hpp                                                             */
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
 * @file random.hpp
 * @brief PCG32 — the deterministic pseudo-random generator the particle system seeds from.
 *
 * The deterministic CPU backend must be bit-reproducible for rollback/replay, so it may
 * never touch a global RNG or the wall clock. PCG32 (O'Neill 2014) is a tiny, fast,
 * fully-defined integer generator whose entire state is two `std::uint64_t`s, which makes a
 * generator trivially copyable and byte-snapshottable exactly like the state column it lives
 * beside. The GPU cosmetic backend uses the same construction in its shaders (seed derived
 * from emitter seed + particle index), so both backends draw from statistically identical
 * streams and an effect looks the same whichever domain it runs in.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Vfx
    {
        /**
         * @brief A permuted-congruential 32-bit generator with 64-bit state.
         *
         * Trivially copyable: seeding is pure arithmetic and no field is a pointer, so a
         * `Pcg32` may be embedded in a snapshottable state column and compared with memcmp.
         * The default-constructed stream is fixed and reproducible; @ref seed re-bases it.
         */
        struct Pcg32
        {
            std::uint64_t state = 0x853c49e6748fea9bull;
            std::uint64_t increment = 0xda3e39cb94b95bdbull;

            /**
             * @brief Re-bases the stream on a seed and a stream-selector.
             *
             * Different @p sequence values give statistically independent streams from the
             * same @p seed, which is how one emitter's particles each get an uncorrelated
             * sub-stream (sequence = particle index) without extra state.
             *
             * @param seed     The initial state seed.
             * @param sequence The stream selector (any value; distinct sequences are independent).
             */
            void seed(std::uint64_t seed, std::uint64_t sequence) noexcept
            {
                state = 0u;
                increment = (sequence << 1u) | 1u;
                next_uint();
                state += seed;
                next_uint();
            }

            /**
             * @brief Advances the stream and returns the next 32-bit value.
             * @return A uniformly-distributed 32-bit unsigned integer.
             */
            std::uint32_t next_uint() noexcept
            {
                const std::uint64_t old_state = state;
                state = old_state * 6364136223846793005ull + increment;
                const std::uint32_t xorshifted =
                    static_cast<std::uint32_t>(((old_state >> 18u) ^ old_state) >> 27u);
                const std::uint32_t rot = static_cast<std::uint32_t>(old_state >> 59u);
                return (xorshifted >> rot) | (xorshifted << ((0u - rot) & 31u));
            }

            /**
             * @brief The next value as a float in the half-open interval [0, 1).
             * @return A uniformly-distributed float in [0, 1).
             */
            float next_float() noexcept
            {
                // 24 mantissa bits give exact, uniformly-spaced floats in [0, 1).
                return static_cast<float>(next_uint() >> 8u) * (1.0f / 16777216.0f);
            }

            /**
             * @brief The next value mapped uniformly onto [min, max).
             * @param min Lower bound (inclusive).
             * @param max Upper bound (exclusive).
             * @return A uniformly-distributed float in [min, max).
             */
            float next_range(float min, float max) noexcept
            {
                return min + (max - min) * next_float();
            }
        };
    } // namespace Vfx
} // namespace SushiEngine
