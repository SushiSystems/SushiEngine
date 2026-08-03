/**************************************************************************/
/* tick_sample.hpp                                                       */
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
 * @file tick_sample.hpp
 * @brief The determinism boundary — the edge-safe reduction from host frames to ticks.
 *
 * The fixed-step accumulator runs zero, one, or several simulation ticks per host frame,
 * so raw per-frame action state cannot cross into the world directly: a key tapped in a
 * zero-tick frame would vanish, and a single press in a two-tick hitch would fire twice.
 * @ref TickSampleAccumulator folds each host frame's @ref ActionSnapshot into a running
 * accumulator and hands the simulation one @ref TickSample per tick through
 * @ref TickSampleAccumulator::consume, obeying three laws (§2.3):
 *
 *  - **Edges are sticky until consumed.** `pressed`/`released` accumulate across frames and
 *    clear only when a tick samples them, so a zero-tick tap surfaces on the next tick.
 *  - **The first tick of a burst consumes the edges; later ticks see level only.** A two-tick
 *    hitch sees `pressed` once and `held` twice — what a real 60 Hz poll would have observed.
 *  - **Analog values are latest-wins; relative axes (mouse deltas) sum and clear.** A level is
 *    what a poll at the tick boundary reads; a delta is a quantity accrued since the last tick.
 *
 * A game turns a @ref TickSample into its trivially-copyable `Command` inside
 * `sample_command`, quantizing analog values with @ref quantize_axis *before* they enter
 * `InputHistory`. Because quantization happens before recording, rollback replay and server
 * reconciliation operate on bit-identical values — prediction misses from float jitter cannot
 * exist. This header is the only place the two cadences (per-frame, per-tick) meet.
 */

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>

#include <SushiEngine/input/action_map.hpp>
#include <SushiEngine/input/events.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /** @brief The full analog range a quantized axis maps onto (symmetric, exact 0 at rest). */
        constexpr std::int16_t AXIS_QUANTIZED_MAX = 32767;

        /** @brief The "never pressed" sentinel for @ref TickSample::ticks_since_press. */
        constexpr std::uint64_t NEVER_PRESSED = 0xFFFFFFFFFFFFFFFFull;

        /**
         * @brief Quantizes a normalized axis value in [-1, 1] to a symmetric @c std::int16_t.
         *
         * Maps 0 to exactly 0 and ±1 to ±@ref AXIS_QUANTIZED_MAX (leaving -32768 unused so the
         * range stays symmetric). This is the step that makes a game's `Command` bit-reproducible:
         * two machines need never agree on raw float math, only on this quantized result, which is
         * a pure function of the clamped input. Deterministic by construction — same float in, same
         * integer out.
         *
         * @param value The normalized axis value; clamped to [-1, 1] first.
         * @return The quantized value in [-32767, 32767].
         */
        inline std::int16_t quantize_axis(float value) noexcept
        {
            const float clamped = value < -1.0f ? -1.0f : (value > 1.0f ? 1.0f : value);
            const long scaled = std::lround(clamped * static_cast<float>(AXIS_QUANTIZED_MAX));
            return static_cast<std::int16_t>(scaled);
        }

        /**
         * @brief One tick's reduced input — a value snapshot, the only thing crossing into the sim.
         *
         * Queried by action name exactly like @ref ActionSnapshot, but its values are the tick-safe
         * reduction: edges that were sticky until this tick, a level that persists across a burst,
         * and relative axes summed since the last tick. A pure value, so it is the thread-crossing
         * currency the (future) sim thread reads.
         */
        class TickSample
        {
            public:
                /** @brief One action's reduced values for this tick. */
                struct Entry
                {
                    bool held = false;
                    bool pressed = false;
                    bool released = false;
                    float axis1d = 0.0f;
                    Vector2 axis2d;
                    std::uint64_t ticks_since_press = NEVER_PRESSED; /**< Ticks since the last press, for buffering. */
                };

                /** @brief Whether @p action is held at this tick. */
                bool held(const std::string& action) const noexcept
                {
                    const Entry* entry = find(action);
                    return entry != nullptr && entry->held;
                }

                /** @brief Whether @p action was pressed at or before this tick since the last one. */
                bool pressed(const std::string& action) const noexcept
                {
                    const Entry* entry = find(action);
                    return entry != nullptr && entry->pressed;
                }

                /** @brief Whether @p action was released since the last tick. */
                bool released(const std::string& action) const noexcept
                {
                    const Entry* entry = find(action);
                    return entry != nullptr && entry->released;
                }

                /** @brief The `Axis1D` value of @p action at this tick, or 0 if absent. */
                float axis1d(const std::string& action) const noexcept
                {
                    const Entry* entry = find(action);
                    return entry != nullptr ? entry->axis1d : 0.0f;
                }

                /** @brief The `Axis2D` value of @p action at this tick, or (0,0) if absent. */
                Vector2 axis2d(const std::string& action) const noexcept
                {
                    const Entry* entry = find(action);
                    return entry != nullptr ? entry->axis2d : Vector2{};
                }

                /**
                 * @brief Whether @p action was pressed within the last @p window_ticks (this one included).
                 *
                 * The input-buffer query: a jump pressed a few ticks before landing still fires, and
                 * a "coyote time" grace works the same way. `pressed_within(name, 0)` is `pressed(name)`.
                 * A window covering an action never pressed is always false.
                 *
                 * @param action       The action to test.
                 * @param window_ticks How many ticks back still count as a press.
                 * @return True if the last press is within @p window_ticks.
                 */
                bool pressed_within(const std::string& action, std::uint64_t window_ticks) const noexcept
                {
                    const Entry* entry = find(action);
                    return entry != nullptr && entry->ticks_since_press <= window_ticks;
                }

                /** @brief The full entry for @p action, or nullptr if absent. */
                const Entry* find(const std::string& action) const noexcept
                {
                    const auto iterator = entries_.find(action);
                    return iterator != entries_.end() ? &iterator->second : nullptr;
                }

                /** @brief Records @p entry for @p action (called by the accumulator). */
                void set(const std::string& action, const Entry& entry)
                {
                    entries_[action] = entry;
                }

            private:
                std::unordered_map<std::string, Entry> entries_;
        };

        /**
         * @brief Folds host-frame action snapshots into per-tick @ref TickSample values.
         *
         * @ref accumulate is called once per host frame with the frame's resolved snapshot;
         * @ref consume is called once per fixed tick from inside `sample_command`. Between an
         * @ref accumulate and the @ref consume that follows, edges are sticky and relative axes
         * accrue; @ref consume clears the edges and zeroes the relative axes it hands out, while
         * levels and absolute axes persist so the next tick of a burst reads the same level.
         */
        class TickSampleAccumulator
        {
            public:
                /**
                 * @brief Folds one host frame's @p snapshot into the running accumulator.
                 * @param snapshot The mapper's resolved actions for the frame just completed.
                 */
                void accumulate(const ActionSnapshot& snapshot)
                {
                    for (const auto& pair : snapshot.values())
                    {
                        const std::string& name = pair.first;
                        const ActionValue& value = pair.second;
                        Running& running = running_[name];

                        running.held = value.held;
                        running.pressed = running.pressed || value.pressed;
                        running.released = running.released || value.released;
                        running.relative = value.relative;
                        if (value.relative)
                        {
                            running.axis1d += value.axis1d;
                            running.axis2d = running.axis2d + value.axis2d;
                        }
                        else
                        {
                            running.axis1d = value.axis1d;
                            running.axis2d = value.axis2d;
                        }
                    }
                }

                /**
                 * @brief Produces this tick's @ref TickSample and consumes the edges/relative axes.
                 *
                 * The returned sample carries the accumulated edges and the summed relative axes;
                 * on return those are cleared so a subsequent tick in the same burst (with no
                 * intervening @ref accumulate) sees only level state. Levels and absolute axes are
                 * left intact.
                 *
                 * @return The reduced input for one fixed tick.
                 */
                TickSample consume()
                {
                    TickSample sample;
                    for (auto& pair : running_)
                    {
                        Running& running = pair.second;

                        // Buffering: remember the tick a press landed on, so a later tick can ask how
                        // long ago it was (jump-buffer / coyote-time windows).
                        if (running.pressed)
                            running.last_press_tick = tick_index_;

                        TickSample::Entry entry;
                        entry.held = running.held;
                        entry.pressed = running.pressed;
                        entry.released = running.released;
                        entry.axis1d = running.axis1d;
                        entry.axis2d = running.axis2d;
                        entry.ticks_since_press = running.last_press_tick == NEVER_PRESSED
                                                      ? NEVER_PRESSED
                                                      : tick_index_ - running.last_press_tick;
                        sample.set(pair.first, entry);

                        running.pressed = false;
                        running.released = false;
                        if (running.relative)
                        {
                            running.axis1d = 0.0f;
                            running.axis2d = Vector2{};
                        }
                    }
                    ++tick_index_;
                    return sample;
                }

                /** @brief The number of ticks consumed so far (the next tick's index). */
                std::uint64_t tick_index() const noexcept { return tick_index_; }

                /** @brief Discards all accumulated state (a hard reset, e.g. on focus loss). */
                void reset()
                {
                    running_.clear();
                    tick_index_ = 0;
                }

            private:
                struct Running
                {
                    bool held = false;
                    bool pressed = false;
                    bool released = false;
                    bool relative = false;
                    float axis1d = 0.0f;
                    Vector2 axis2d;
                    std::uint64_t last_press_tick = NEVER_PRESSED;
                };

                std::unordered_map<std::string, Running> running_;
                std::uint64_t tick_index_ = 0;
        };
    } // namespace Input
} // namespace SushiEngine
