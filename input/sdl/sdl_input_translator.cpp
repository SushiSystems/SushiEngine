/**************************************************************************/
/* sdl_input_translator.cpp                                              */
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

#include "sdl/sdl_input_translator.hpp"

#include <SDL.h>

namespace SushiEngine
{
    namespace Input
    {
        namespace
        {
            /**
             * @brief Maps an SDL mouse-button index to a @ref MouseButton ordinal.
             *
             * SDL numbers buttons 1..5 (left, middle, right, X1, X2) — the same order and
             * values @ref MouseButton uses — so the map is the identity within range.
             */
            std::uint16_t mouse_button_ordinal(Uint8 sdl_button) noexcept
            {
                switch (sdl_button)
                {
                    case SDL_BUTTON_LEFT:   return static_cast<std::uint16_t>(MouseButton::Left);
                    case SDL_BUTTON_MIDDLE: return static_cast<std::uint16_t>(MouseButton::Middle);
                    case SDL_BUTTON_RIGHT:  return static_cast<std::uint16_t>(MouseButton::Right);
                    case SDL_BUTTON_X1:     return static_cast<std::uint16_t>(MouseButton::X1);
                    case SDL_BUTTON_X2:     return static_cast<std::uint16_t>(MouseButton::X2);
                    default:                return 0;
                }
            }

            /**
             * @brief Normalizes an SDL axis value to the engine's float range.
             *
             * Sticks span -32768..32767 and map to [-1, 1] (clamped, since -32768/32767 is
             * just past -1); triggers span 0..32767 and fall out as [0, 1] under the same
             * division. @ref GamepadButton / @ref GamepadAxis ordinals match SDL's, so the
             * enum value crosses unchanged.
             */
            float normalize_axis(Sint16 value) noexcept
            {
                const float scaled = static_cast<float>(value) / 32767.0f;
                if (scaled > 1.0f)
                    return 1.0f;
                if (scaled < -1.0f)
                    return -1.0f;
                return scaled;
            }

            /** @brief Clamps @p value to [0, 1] for a rumble intensity. */
            float clamp_unit(float value) noexcept
            {
                if (value < 0.0f)
                    return 0.0f;
                if (value > 1.0f)
                    return 1.0f;
                return value;
            }
        } // namespace

        SdlInputTranslator::SdlInputTranslator(InputManager& manager)
        {
            manager.add_source(*this);
        }

        SdlInputTranslator::~SdlInputTranslator()
        {
            for (void*& handle : controllers_)
            {
                if (handle != nullptr)
                {
                    SDL_GameControllerClose(static_cast<SDL_GameController*>(handle));
                    handle = nullptr;
                }
            }
        }

        void SdlInputTranslator::open_controller(int device_index)
        {
            SDL_GameController* controller = SDL_GameControllerOpen(device_index);
            if (controller == nullptr)
                return;

            const SDL_JoystickID instance =
                SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controller));
            const DeviceId device = slots_.connect(static_cast<GamepadInstanceId>(instance));
            if (device == INVALID_DEVICE)
            {
                // No free slot: close the handle rather than track a controller we cannot address.
                SDL_GameControllerClose(controller);
                return;
            }

            controllers_[static_cast<std::size_t>(device - FIRST_GAMEPAD_DEVICE)] = controller;

            InputEvent connected;
            connected.device = device;
            connected.type = EventType::DeviceConnected;
            push(connected);
        }

        void SdlInputTranslator::close_controller(GamepadInstanceId instance)
        {
            const DeviceId device = slots_.device_for(instance);
            if (device == INVALID_DEVICE)
                return;

            const std::size_t slot = static_cast<std::size_t>(device - FIRST_GAMEPAD_DEVICE);
            if (controllers_[slot] != nullptr)
            {
                SDL_GameControllerClose(static_cast<SDL_GameController*>(controllers_[slot]));
                controllers_[slot] = nullptr;
            }
            slots_.disconnect(instance);

            InputEvent disconnected;
            disconnected.device = device;
            disconnected.type = EventType::DeviceDisconnected;
            push(disconnected);
        }

        int SdlInputTranslator::finger_slot(std::int64_t finger) const noexcept
        {
            for (int slot = 0; slot < MAX_TOUCH_POINTS; ++slot)
                if (fingers_[static_cast<std::size_t>(slot)] == finger)
                    return slot;
            return -1;
        }

        int SdlInputTranslator::claim_finger(std::int64_t finger) noexcept
        {
            const int existing = finger_slot(finger);
            if (existing >= 0)
                return existing;
            for (int slot = 0; slot < MAX_TOUCH_POINTS; ++slot)
            {
                if (fingers_[static_cast<std::size_t>(slot)] == -1)
                {
                    fingers_[static_cast<std::size_t>(slot)] = finger;
                    return slot;
                }
            }
            return -1;
        }

        int SdlInputTranslator::release_finger(std::int64_t finger) noexcept
        {
            const int slot = finger_slot(finger);
            if (slot >= 0)
                fingers_[static_cast<std::size_t>(slot)] = -1;
            return slot;
        }

        void SdlInputTranslator::handle_native_event(const void* sdl_event)
        {
            const SDL_Event& event = *static_cast<const SDL_Event*>(sdl_event);

            switch (event.type)
            {
                case SDL_KEYDOWN:
                {
                    // Backspace edits an active text field rather than acting as a key.
                    if (text_channel_ != nullptr && text_channel_->active() &&
                        event.key.keysym.scancode == SDL_SCANCODE_BACKSPACE)
                    {
                        text_channel_->backspace();
                        break;
                    }
                    // Ignore auto-repeat: an action's edge is a physical press, not a repeat.
                    if (event.key.repeat != 0)
                        break;
                    InputEvent translated;
                    translated.device = KEYBOARD_DEVICE;
                    translated.type = EventType::KeyDown;
                    translated.control = static_cast<std::uint16_t>(event.key.keysym.scancode);
                    translated.value = 1.0f;
                    push(translated);
                    break;
                }
                case SDL_TEXTINPUT:
                {
                    if (text_channel_ != nullptr)
                        text_channel_->append(event.text.text);
                    break;
                }
                case SDL_KEYUP:
                {
                    InputEvent translated;
                    translated.device = KEYBOARD_DEVICE;
                    translated.type = EventType::KeyUp;
                    translated.control = static_cast<std::uint16_t>(event.key.keysym.scancode);
                    translated.value = 0.0f;
                    push(translated);
                    break;
                }
                case SDL_MOUSEBUTTONDOWN:
                case SDL_MOUSEBUTTONUP:
                {
                    InputEvent translated;
                    translated.device = MOUSE_DEVICE;
                    translated.type = event.type == SDL_MOUSEBUTTONDOWN ? EventType::MouseButtonDown
                                                                        : EventType::MouseButtonUp;
                    translated.control = mouse_button_ordinal(event.button.button);
                    translated.value = event.type == SDL_MOUSEBUTTONDOWN ? 1.0f : 0.0f;
                    translated.x = static_cast<float>(event.button.x);
                    translated.y = static_cast<float>(event.button.y);
                    push(translated);
                    break;
                }
                case SDL_MOUSEMOTION:
                {
                    InputEvent translated;
                    translated.device = MOUSE_DEVICE;
                    translated.type = EventType::MouseMove;
                    translated.x = static_cast<float>(event.motion.x);
                    translated.y = static_cast<float>(event.motion.y);
                    push(translated);
                    break;
                }
                case SDL_MOUSEWHEEL:
                {
                    InputEvent translated;
                    translated.device = MOUSE_DEVICE;
                    translated.type = EventType::MouseWheel;
                    float wheel_x = static_cast<float>(event.wheel.x);
                    float wheel_y = static_cast<float>(event.wheel.y);
                    if (event.wheel.direction == SDL_MOUSEWHEEL_FLIPPED)
                    {
                        wheel_x = -wheel_x;
                        wheel_y = -wheel_y;
                    }
                    translated.x = wheel_x;
                    translated.y = wheel_y;
                    push(translated);
                    break;
                }
                case SDL_CONTROLLERDEVICEADDED:
                {
                    // For ADDED, `which` is a device index to open.
                    open_controller(event.cdevice.which);
                    break;
                }
                case SDL_CONTROLLERDEVICEREMOVED:
                {
                    // For REMOVED, `which` is the joystick instance id.
                    close_controller(static_cast<GamepadInstanceId>(event.cdevice.which));
                    break;
                }
                case SDL_CONTROLLERBUTTONDOWN:
                case SDL_CONTROLLERBUTTONUP:
                {
                    const DeviceId device =
                        slots_.device_for(static_cast<GamepadInstanceId>(event.cbutton.which));
                    if (device == INVALID_DEVICE)
                        break;
                    InputEvent translated;
                    translated.device = device;
                    translated.type = event.type == SDL_CONTROLLERBUTTONDOWN ? EventType::GamepadButtonDown
                                                                             : EventType::GamepadButtonUp;
                    translated.control = static_cast<std::uint16_t>(event.cbutton.button);
                    translated.value = event.type == SDL_CONTROLLERBUTTONDOWN ? 1.0f : 0.0f;
                    push(translated);
                    break;
                }
                case SDL_CONTROLLERAXISMOTION:
                {
                    const DeviceId device =
                        slots_.device_for(static_cast<GamepadInstanceId>(event.caxis.which));
                    if (device == INVALID_DEVICE)
                        break;
                    InputEvent translated;
                    translated.device = device;
                    translated.type = EventType::GamepadAxisMotion;
                    translated.control = static_cast<std::uint16_t>(event.caxis.axis);
                    translated.value = normalize_axis(event.caxis.value);
                    push(translated);
                    break;
                }
                case SDL_FINGERDOWN:
                case SDL_FINGERMOTION:
                case SDL_FINGERUP:
                {
                    // Finger positions are normalized [0,1]; scale to pixels for the pointer table
                    // and virtual controls. Without a known display size, drop them rather than
                    // emit at the wrong scale.
                    if (display_width_ <= 0 || display_height_ <= 0)
                        break;
                    const std::int64_t finger = static_cast<std::int64_t>(event.tfinger.fingerId);
                    int slot = -1;
                    EventType type = EventType::TouchMove;
                    if (event.type == SDL_FINGERDOWN)
                    {
                        slot = claim_finger(finger);
                        type = EventType::TouchDown;
                    }
                    else if (event.type == SDL_FINGERUP)
                    {
                        slot = release_finger(finger);
                        type = EventType::TouchUp;
                    }
                    else
                    {
                        slot = finger_slot(finger);
                        type = EventType::TouchMove;
                    }
                    if (slot < 0)
                        break;
                    InputEvent translated;
                    translated.device = MOUSE_DEVICE;
                    translated.type = type;
                    translated.control = static_cast<std::uint16_t>(slot);
                    translated.x = event.tfinger.x * static_cast<float>(display_width_);
                    translated.y = event.tfinger.y * static_cast<float>(display_height_);
                    push(translated);
                    break;
                }
                default:
                    break;
            }
        }

        void SdlInputTranslator::start_text_input()
        {
            SDL_StartTextInput();
        }

        void SdlInputTranslator::stop_text_input()
        {
            SDL_StopTextInput();
        }

        void SdlInputTranslator::poll(std::vector<InputEvent>& out)
        {
            for (InputEvent& event : pending_)
            {
                event.frame = frame_;
                out.push_back(event);
            }
            pending_.clear();
            ++frame_;
        }

        void SdlInputTranslator::rumble(DeviceId device, float low_frequency, float high_frequency,
                                        float duration_seconds)
        {
            const int slot = device - FIRST_GAMEPAD_DEVICE;
            if (slot < 0 || slot >= MAX_GAMEPADS || duration_seconds <= 0.0f)
                return;
            void* handle = controllers_[static_cast<std::size_t>(slot)];
            if (handle == nullptr)
                return;

            const Uint16 low = static_cast<Uint16>(clamp_unit(low_frequency) * 65535.0f);
            const Uint16 high = static_cast<Uint16>(clamp_unit(high_frequency) * 65535.0f);
            const Uint32 milliseconds = static_cast<Uint32>(duration_seconds * 1000.0f);
            SDL_GameControllerRumble(static_cast<SDL_GameController*>(handle), low, high, milliseconds);
        }
    } // namespace Input
} // namespace SushiEngine
