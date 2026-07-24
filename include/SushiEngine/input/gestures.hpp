/**************************************************************************/
/* gestures.hpp                                                          */
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
 * @file gestures.hpp
 * @brief The optional recognizer stage: pointers → tap, long-press, drag, pinch (§2.4).
 *
 * A @ref GestureRecognizer reads the pointers the @ref VirtualControlSource did not claim and
 * turns them into high-level gestures. It is deliberately separate from the event pipeline: a
 * gesture needs time (a tap is a brief press, a long-press a sustained one), so recognition is
 * driven by an explicit `update(dt)` that returns the frame's gestures. Mapping a gesture to a
 * game action is the consumer's choice — a tap can fire a button, a drag can feed a 2D axis —
 * which keeps the recognizer a pure sensor. Header-only and driven by @ref DeviceRegistry
 * pointer state, so it runs headlessly with scripted pointers and a supplied dt.
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <SushiEngine/input/device_registry.hpp>
#include <SushiEngine/input/events.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /** @brief The kind of a recognized @ref Gesture. */
        enum class GestureType : std::uint8_t
        {
            Tap,       /**< A brief press-and-release with little movement. */
            LongPress, /**< A press held past the long-press time without moving. */
            DragBegin, /**< A held pointer moved past the drag threshold. */
            DragMove,  /**< Continued movement of an in-progress drag. */
            DragEnd,   /**< The dragging pointer lifted. */
            Pinch,     /**< Two pointers changing their separation. */
        };

        /** @brief One recognized gesture for a frame. */
        struct Gesture
        {
            GestureType type = GestureType::Tap;
            int pointer = -1;   /**< The pointer id it came from (the first of the pair for pinch). */
            Vector2 position;   /**< Current position in pixels (midpoint for pinch). */
            Vector2 delta;      /**< Movement since last frame (for drag); `delta.x` = separation change for pinch. */
            float scale = 1.0f; /**< Pinch: current separation / initial separation. 1 otherwise. */
        };

        /** @brief Thresholds tuning the recognizer, in pixels and seconds. */
        struct GestureConfig
        {
            float tap_max_movement = 10.0f; /**< A tap may not move more than this. */
            float tap_max_time = 0.30f;     /**< A tap must release within this. */
            float long_press_time = 0.60f;  /**< A stationary press this long is a long-press. */
            float drag_threshold = 12.0f;   /**< Movement past this starts a drag. */
        };

        /**
         * @brief Turns pointer state into tap/long-press/drag/pinch gestures.
         *
         * Construct it over a @ref DeviceRegistry and call @ref update once per frame with the
         * frame's delta time; it returns the gestures recognized that frame. It holds per-pointer
         * timing and movement state internally, so the caller keeps no bookkeeping.
         */
        class GestureRecognizer
        {
            public:
                /**
                 * @brief Creates a recognizer reading @p registry, tuned by @p config.
                 * @param registry The pointer state to recognize over.
                 * @param config   The thresholds; defaults suit a touchscreen.
                 */
                explicit GestureRecognizer(const DeviceRegistry& registry, GestureConfig config = {}) noexcept
                    : registry_(&registry), config_(config)
                {
                }

                /**
                 * @brief Advances recognition by @p dt and returns this frame's gestures.
                 * @param dt Seconds since the last call.
                 * @return The gestures recognized this frame (empty if none).
                 */
                std::vector<Gesture> update(float dt)
                {
                    std::vector<Gesture> gestures;
                    recognize_single_pointers(dt, gestures);
                    recognize_pinch(gestures);
                    return gestures;
                }

            private:
                struct PointerState
                {
                    bool active = false;
                    bool dragging = false;
                    bool long_press_fired = false;
                    float held_time = 0.0f;
                    Vector2 start;
                    Vector2 last;
                };

                static float distance(const Vector2& a, const Vector2& b) noexcept
                {
                    const float dx = a.x - b.x;
                    const float dy = a.y - b.y;
                    return std::sqrt(dx * dx + dy * dy);
                }

                void recognize_single_pointers(float dt, std::vector<Gesture>& gestures)
                {
                    for (int index = 0; index < MAX_TOUCH_POINTS; ++index)
                    {
                        const DeviceRegistry::Pointer& pointer = registry_->pointer(index);
                        PointerState& state = states_[static_cast<std::size_t>(index)];

                        if (pointer.active && !state.active)
                        {
                            // Down: begin tracking.
                            state = PointerState{};
                            state.active = true;
                            state.start = pointer.position;
                            state.last = pointer.position;
                            continue;
                        }

                        if (pointer.active && state.active)
                        {
                            state.held_time += dt;
                            const Vector2 delta{pointer.position.x - state.last.x,
                                                pointer.position.y - state.last.y};
                            state.last = pointer.position;

                            if (state.dragging)
                            {
                                Gesture move;
                                move.type = GestureType::DragMove;
                                move.pointer = index;
                                move.position = pointer.position;
                                move.delta = delta;
                                gestures.push_back(move);
                            }
                            else if (distance(pointer.position, state.start) >= config_.drag_threshold)
                            {
                                state.dragging = true;
                                Gesture begin;
                                begin.type = GestureType::DragBegin;
                                begin.pointer = index;
                                begin.position = pointer.position;
                                begin.delta = delta;
                                gestures.push_back(begin);
                            }
                            else if (!state.long_press_fired && state.held_time >= config_.long_press_time)
                            {
                                state.long_press_fired = true;
                                Gesture press;
                                press.type = GestureType::LongPress;
                                press.pointer = index;
                                press.position = pointer.position;
                                gestures.push_back(press);
                            }
                            continue;
                        }

                        if (!pointer.active && state.active)
                        {
                            // Up: end a drag, or fire a tap if it was brief and still.
                            if (state.dragging)
                            {
                                Gesture end;
                                end.type = GestureType::DragEnd;
                                end.pointer = index;
                                end.position = state.last;
                                gestures.push_back(end);
                            }
                            else if (state.held_time <= config_.tap_max_time &&
                                     distance(state.last, state.start) <= config_.tap_max_movement)
                            {
                                Gesture tap;
                                tap.type = GestureType::Tap;
                                tap.pointer = index;
                                tap.position = state.last;
                                gestures.push_back(tap);
                            }
                            state = PointerState{};
                        }
                    }
                }

                void recognize_pinch(std::vector<Gesture>& gestures)
                {
                    int first = -1;
                    int second = -1;
                    for (int index = 0; index < MAX_TOUCH_POINTS; ++index)
                    {
                        if (!registry_->pointer(index).active)
                            continue;
                        if (first < 0)
                            first = index;
                        else if (second < 0)
                        {
                            second = index;
                            break;
                        }
                    }

                    if (first < 0 || second < 0)
                    {
                        pinch_active_ = false;
                        return;
                    }

                    const Vector2 a = registry_->pointer(first).position;
                    const Vector2 b = registry_->pointer(second).position;
                    const float separation = distance(a, b);

                    if (!pinch_active_)
                    {
                        pinch_active_ = true;
                        pinch_initial_ = separation > 0.0f ? separation : 1.0f;
                        pinch_previous_ = separation;
                        return; // establish the reference before reporting.
                    }

                    Gesture pinch;
                    pinch.type = GestureType::Pinch;
                    pinch.pointer = first;
                    pinch.position = Vector2{(a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f};
                    pinch.delta = Vector2{separation - pinch_previous_, 0.0f};
                    pinch.scale = separation / pinch_initial_;
                    gestures.push_back(pinch);
                    pinch_previous_ = separation;
                }

                const DeviceRegistry* registry_;
                GestureConfig config_;
                std::array<PointerState, MAX_TOUCH_POINTS> states_{};
                bool pinch_active_ = false;
                float pinch_initial_ = 1.0f;
                float pinch_previous_ = 0.0f;
        };
    } // namespace Input
} // namespace SushiEngine
