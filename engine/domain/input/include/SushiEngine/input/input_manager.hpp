/**************************************************************************/
/* input_manager.hpp                                                     */
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
 * @file input_manager.hpp
 * @brief The façade that wires sources → registry → mapper; the one include consumers need.
 *
 * A consumer constructs one @ref InputManager, registers one or more @ref IInputSource
 * (an SDL translator in a windowed build, a @ref ScriptedInputSource in a test), pushes
 * its @ref InputContext stack, and calls @ref begin_frame once per host frame. That call
 * drains every source, folds the events into the @ref DeviceRegistry, and resolves the
 * @ref ActionMapper into a fresh @ref ActionSnapshot the editor and UI read. The
 * simulation reads a reduced per-tick sample (Phase 2) built from the same drained events;
 * the manager exposes them through @ref frame_events for that stage. Nothing here knows a
 * device is SDL-backed, and nothing here touches the `World`.
 */

#include <vector>

#include <SushiEngine/input/action_map.hpp>
#include <SushiEngine/input/device_registry.hpp>
#include <SushiEngine/input/events.hpp>
#include <SushiEngine/input/gamepad.hpp>
#include <SushiEngine/input/gestures.hpp>
#include <SushiEngine/input/haptics.hpp>
#include <SushiEngine/input/player.hpp>
#include <SushiEngine/input/rebinding.hpp>
#include <SushiEngine/input/replay.hpp>
#include <SushiEngine/input/source.hpp>
#include <SushiEngine/input/text_input.hpp>
#include <SushiEngine/input/tick_sample.hpp>
#include <SushiEngine/input/virtual_controls.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /**
         * @brief Owns the device registry, the action mapper, and the source list.
         *
         * The manager holds non-owning pointers to its sources and contexts: their lifetime
         * belongs to the caller (typically both outlive the loop), matching the worked example
         * where a `SdlInputTranslator` and each `InputContext` are locals in `main`. This keeps
         * the manager a pure orchestrator with no ownership policy of its own.
         */
        class InputManager
        {
            public:
                /** @brief Registers @p source as a producer of events drained each frame. */
                void add_source(IInputSource& source) { sources_.push_back(&source); }

                /** @brief Removes @p source from the drained set. */
                void remove_source(const IInputSource& source)
                {
                    for (std::size_t index = sources_.size(); index-- > 0;)
                        if (sources_[index] == &source)
                            sources_.erase(sources_.begin() + static_cast<std::ptrdiff_t>(index));
                }

                /**
                 * @brief Registers @p source as a virtual (pointer-driven) source.
                 *
                 * Virtual sources (on-screen controls, @ref VirtualControlSource) are polled in a
                 * second pass, after the primary events are folded into the registry, so they can
                 * read this frame's pointer state and synthesize events from it. Their output is
                 * then folded too, so the mapper sees a virtual stick exactly like a hardware one.
                 *
                 * @param source The virtual source to poll each frame after the primary fold.
                 */
                void add_virtual_source(IInputSource& source) { virtual_sources_.push_back(&source); }

                /** @brief The device state, mutable — for the mouse-as-pointer flag and pointer feeds. */
                DeviceRegistry& registry_mutable() noexcept { return registry_; }

                /** @brief Pushes @p context as the highest-priority input context. */
                void push_context(InputContext& context) { mapper_.push_context(context); }

                /** @brief Pops the highest-priority context. */
                void pop_context() { mapper_.pop_context(); }

                /** @brief Removes @p context from the stack wherever it sits. */
                void remove_context(const InputContext& context) { mapper_.remove_context(context); }

                /** @brief Sets the immediate-mode UI capture gate for the next frame. */
                void set_gate(const InputGate& gate) noexcept { gate_ = gate; }

                /** @brief The current capture gate. */
                const InputGate& gate() const noexcept { return gate_; }

                /**
                 * @brief Drains all sources, folds their events into state, and resolves actions.
                 *
                 * Call once per host frame, after the window has pumped (so the SDL translator
                 * has received this frame's native events) and before the fixed-step advance.
                 */
                void begin_frame()
                {
                    frame_events_.clear();
                    for (IInputSource* source : sources_)
                        source->poll(frame_events_);

                    registry_.begin_frame();
                    for (const InputEvent& event : frame_events_)
                        registry_.ingest(event);

                    // Second pass: virtual sources read the just-folded pointer state and synthesize
                    // gamepad-shaped events, which are then folded so the mapper cannot tell them
                    // from hardware. Only the newly appended events are ingested.
                    const std::size_t primary_count = frame_events_.size();
                    for (IInputSource* source : virtual_sources_)
                        source->poll(frame_events_);
                    for (std::size_t index = primary_count; index < frame_events_.size(); ++index)
                        registry_.ingest(frame_events_[index]);

                    mapper_.update(registry_, gate_);
                    accumulator_.accumulate(mapper_.snapshot());
                }

                /** @brief The resolved actions for the current frame (editor/UI cadence). */
                const ActionSnapshot& snapshot() const noexcept { return mapper_.snapshot(); }

                /**
                 * @brief Reduces the frames since the last tick into one @ref TickSample.
                 *
                 * Call exactly once per fixed simulation tick, from inside the game's
                 * `Loop::App::sample_command` hook — the sim's only input surface. Edges are
                 * consumed and relative axes zeroed on return, so a burst of ticks in one host
                 * frame sees each edge once (§2.3). Never call it on a render-only frame.
                 *
                 * @return The tick-safe reduced input.
                 */
                TickSample consume_tick_sample() { return accumulator_.consume(); }

                /** @brief The per-tick sampler, for a hard reset on focus loss or level change. */
                TickSampleAccumulator& tick_accumulator() noexcept { return accumulator_; }

                /** @brief The device state, for the UI pointer feed and rebinding capture. */
                const DeviceRegistry& registry() const noexcept { return registry_; }

                /** @brief The mapper, for advanced context management. */
                ActionMapper& mapper() noexcept { return mapper_; }

                /**
                 * @brief The events drained this frame, in arrival order.
                 *
                 * The per-tick sampler (Phase 2) reads these to build its edge-safe reduction;
                 * exposing the raw stream keeps that stage decoupled from the mapper.
                 */
                const std::vector<InputEvent>& frame_events() const noexcept { return frame_events_; }

            private:
                std::vector<IInputSource*> sources_;
                std::vector<IInputSource*> virtual_sources_;
                DeviceRegistry registry_;
                ActionMapper mapper_;
                TickSampleAccumulator accumulator_;
                InputGate gate_;
                std::vector<InputEvent> frame_events_;
        };
    } // namespace Input
} // namespace SushiEngine
