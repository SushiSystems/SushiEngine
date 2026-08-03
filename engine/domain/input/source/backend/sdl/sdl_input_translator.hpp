/**************************************************************************/
/* sdl_input_translator.hpp                                              */
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

#ifndef SUSHIENGINE_INPUT_SDL_INPUT_TRANSLATOR_HPP
#define SUSHIENGINE_INPUT_SDL_INPUT_TRANSLATOR_HPP

/**
 * @file sdl_input_translator.hpp
 * @brief The only SDL-aware input component: SDL_Event → engine @ref InputEvent.
 *
 * The translator does not pump SDL — it receives already-pumped native events through
 * @ref handle_native_event, registered on the window's existing event-handler seam
 * alongside ImGui (there must be exactly one `SDL_PollEvent` loop, and it stays in
 * `SdlWindow`). Translated events are buffered and drained by @ref poll, so the
 * translator is an ordinary @ref IInputSource — behaviourally identical to a
 * @ref ScriptedInputSource from the registry's point of view, which is what keeps the
 * whole action layer free of SDL.
 *
 * The header exposes no SDL type: the native event crosses as a `const void*`, so a
 * consumer includes this and links `sushi_input` without pulling SDL into its own
 * translation unit (the game-controller handles it owns are held as opaque `void*`).
 * Phase 1 translates keyboard and mouse; Phase 3 adds gamepad translation, hot-plug slot
 * assignment, and rumble. Touch joins at its phase without changing this interface.
 */

#include <array>
#include <cstdint>
#include <vector>

#include <SushiEngine/input/device_registry.hpp>
#include <SushiEngine/input/events.hpp>
#include <SushiEngine/input/gamepad.hpp>
#include <SushiEngine/input/haptics.hpp>
#include <SushiEngine/input/input_manager.hpp>
#include <SushiEngine/input/source.hpp>
#include <SushiEngine/input/text_input.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /**
         * @brief Translates pumped SDL events into engine events for one @ref InputManager.
         *
         * Constructing it registers the translator as a source of @p manager, so the only
         * remaining wiring a consumer does is forwarding native events into
         * @ref handle_native_event from the window's handler list. It also implements
         * @ref IHapticsSink, so the same object gameplay never sees as SDL is the one that
         * drives rumble.
         */
        class SdlInputTranslator final : public IInputSource, public IHapticsSink
        {
            public:
                /**
                 * @brief Registers this translator as a source of @p manager.
                 * @param manager The manager whose registry will fold the translated events.
                 */
                explicit SdlInputTranslator(InputManager& manager);

                /** @brief Closes any game controllers this translator opened. */
                ~SdlInputTranslator() override;

                SdlInputTranslator(const SdlInputTranslator&) = delete;
                SdlInputTranslator& operator=(const SdlInputTranslator&) = delete;

                /**
                 * @brief Translates one pumped native event, buffering any engine events it yields.
                 *
                 * Events this translator does not care about (window, quit, text) are ignored;
                 * they remain the business of the other handlers on the window's list. Controller
                 * connect/disconnect events open or close the SDL handle and assign or free the
                 * device slot before emitting the corresponding engine event.
                 *
                 * @param sdl_event A pointer to the SDL_Event to translate (as an opaque handle).
                 */
                void handle_native_event(const void* sdl_event);

                /**
                 * @brief Drains the buffered engine events for this host frame.
                 * @param out Buffer the events are appended to; not cleared by this source.
                 */
                void poll(std::vector<InputEvent>& out) override;

                /** @copydoc IHapticsSink::rumble */
                void rumble(DeviceId device, float low_frequency, float high_frequency,
                            float duration_seconds) override;

                /**
                 * @brief Sets the drawable size used to turn normalized touch coords into pixels.
                 *
                 * SDL reports finger positions in [0, 1]; touch events (and virtual controls) work
                 * in window pixels, so the translator scales by this size. Call it once and on each
                 * resize; until set, finger events are dropped rather than emitted at the wrong scale.
                 *
                 * @param width  Drawable width in pixels.
                 * @param height Drawable height in pixels.
                 */
                void set_display_size(int width, int height) noexcept
                {
                    display_width_ = width;
                    display_height_ = height;
                }

                /**
                 * @brief Routes typed text into @p channel while it is active.
                 *
                 * When set, `SDL_TEXTINPUT` fragments are appended to @p channel and Backspace edits
                 * it, instead of (in addition to) surfacing as key events. Pass nullptr to detach.
                 *
                 * @param channel The text channel to feed, or nullptr for none.
                 */
                void set_text_channel(TextInputChannel* channel) noexcept { text_channel_ = channel; }

                /** @brief Starts OS text input (IME on-screen keyboard, dead keys). */
                void start_text_input();

                /** @brief Stops OS text input. */
                void stop_text_input();

            private:
                void push(const InputEvent& event) { pending_.push_back(event); }
                void open_controller(int device_index);
                void close_controller(GamepadInstanceId instance);
                int finger_slot(std::int64_t finger) const noexcept;
                int claim_finger(std::int64_t finger) noexcept;
                int release_finger(std::int64_t finger) noexcept;

                std::vector<InputEvent> pending_;
                std::uint64_t frame_ = 0;
                GamepadSlotTable slots_;
                std::array<void*, MAX_GAMEPADS> controllers_{}; /**< Opaque SDL_GameController* per slot. */
                std::array<std::int64_t, MAX_TOUCH_POINTS> fingers_ = filled_fingers();
                int display_width_ = 0;
                int display_height_ = 0;
                TextInputChannel* text_channel_ = nullptr;

                static std::array<std::int64_t, MAX_TOUCH_POINTS> filled_fingers() noexcept
                {
                    std::array<std::int64_t, MAX_TOUCH_POINTS> fingers{};
                    for (std::int64_t& finger : fingers)
                        finger = -1;
                    return fingers;
                }
        };
    } // namespace Input
} // namespace SushiEngine

#endif
