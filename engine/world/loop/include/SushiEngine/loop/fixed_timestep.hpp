/**************************************************************************/
/* fixed_timestep.hpp                                                     */
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
 * @file fixed_timestep.hpp
 * @brief SushiLoop's Time layer: the fixed-step accumulator (see docs/slop/SUSHILOOP.md).
 *
 * The host owns wall-clock time; this class never reads it. It only turns a
 * caller-supplied elapsed duration into a whole number of fixed simulation steps
 * plus a leftover interpolation fraction for rendering, which is what keeps the
 * simulation itself free of `chrono::now()` — a hard determinism rule, since a
 * step count derived from the clock would make replay non-deterministic.
 */

#include <cstddef>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Loop
    {
        /**
         * @brief Accumulates real elapsed time into whole fixed-step ticks.
         *
         * Usage mirrors the classic pattern: call `accumulate(real_delta)` once per
         * host frame, then `consume_step()` in a loop while it returns true, ticking
         * the simulation once per call; `interpolation()` gives the leftover fraction
         * for the render thread to blend between the last two simulation states.
         */
        class FixedTimestepClock
        {
            public:
                /**
                 * @brief Creates a clock stepping at a fixed interval.
                 * @param fixed_dt_seconds Duration of one simulation tick, in seconds (> 0).
                 */
                explicit FixedTimestepClock(Scalar fixed_dt_seconds) noexcept
                    : fixed_dt_(fixed_dt_seconds) {}

                /** @brief The fixed duration of one simulation tick, in seconds. */
                Scalar fixed_dt() const noexcept { return fixed_dt_; }

                /**
                 * @brief Adds elapsed real time to the accumulator.
                 * @param real_delta_seconds Wall-clock time since the last call, in seconds.
                 */
                void accumulate(Scalar real_delta_seconds) noexcept
                {
                    accumulator_ += real_delta_seconds;
                }

                /**
                 * @brief Consumes one fixed step from the accumulator, if one is due.
                 *
                 * Call in a loop until it returns false; each true result means the
                 * simulation should tick exactly once. The number of steps this yields
                 * depends only on the accumulated duration, never on how many times the
                 * caller has already looped, so it stays deterministic.
                 *
                 * @return Whether a full fixed step was available and consumed.
                 */
                bool consume_step() noexcept
                {
                    if (accumulator_ < fixed_dt_)
                        return false;
                    accumulator_ -= fixed_dt_;
                    return true;
                }

                /**
                 * @brief The fraction of a step left in the accumulator, for render blending.
                 * @return A value in [0, 1): 0 means the last tick landed exactly on the frame.
                 */
                Scalar interpolation() const noexcept
                {
                    return accumulator_ / fixed_dt_;
                }

            private:
                Scalar fixed_dt_;
                Scalar accumulator_ = 0;
        };
    } // namespace Loop
} // namespace SushiEngine
