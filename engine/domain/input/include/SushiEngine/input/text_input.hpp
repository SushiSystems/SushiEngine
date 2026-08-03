/**************************************************************************/
/* text_input.hpp                                                        */
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
 * @file text_input.hpp
 * @brief Text entry is not an action — it is a separate channel (§6).
 *
 * IME and character events belong to whoever owns text focus, not to the action mapper: a name
 * being typed must not fire `"Jump"`. A @ref TextInputChannel is that owner — a UTF-8 buffer the
 * SDL translator feeds `SDL_TEXTINPUT` into while it is active. Actions are suppressed for its
 * duration through the mapper's existing capture gate (a consumer sets
 * `InputGate::want_text_input = channel.active()`), so the two never fight over the keyboard.
 * The channel itself is pure engine code with no SDL dependency, so its editing is unit-tested
 * headlessly; the editor keeps using ImGui's own text path, and this serves a future engine-UI
 * text field.
 */

#include <string>

namespace SushiEngine
{
    namespace Input
    {
        /**
         * @brief An active-gated UTF-8 text buffer fed by character events.
         *
         * While @ref active, the translator appends typed text and applies backspaces; the owner
         * (a text field) reads @ref text and clears or commits it. Inactive, it ignores input, so
         * character events outside a focused field are simply dropped.
         */
        class TextInputChannel
        {
            public:
                /** @brief Begins capturing text (a field gained focus). */
                void begin() noexcept { active_ = true; }

                /** @brief Ends capturing text (focus left). */
                void end() noexcept { active_ = false; }

                /** @brief Whether the channel is currently capturing. */
                bool active() const noexcept { return active_; }

                /**
                 * @brief Appends UTF-8 @p utf8 to the buffer if active.
                 * @param utf8 A null-terminated UTF-8 fragment (SDL_TextInputEvent::text).
                 */
                void append(const char* utf8)
                {
                    if (active_ && utf8 != nullptr)
                        buffer_ += utf8;
                }

                /** @brief Removes the last UTF-8 code point from the buffer if active. */
                void backspace()
                {
                    if (!active_)
                        return;
                    while (!buffer_.empty())
                    {
                        const unsigned char byte = static_cast<unsigned char>(buffer_.back());
                        buffer_.pop_back();
                        // Stop once a lead byte is removed; continuation bytes are 10xxxxxx.
                        if ((byte & 0xC0u) != 0x80u)
                            break;
                    }
                }

                /** @brief The captured text so far. */
                const std::string& text() const noexcept { return buffer_; }

                /** @brief Empties the buffer, keeping the active state. */
                void clear() noexcept { buffer_.clear(); }

                /** @brief Empties the buffer and returns its previous contents (a commit). */
                std::string take()
                {
                    std::string committed = std::move(buffer_);
                    buffer_.clear();
                    return committed;
                }

            private:
                std::string buffer_;
                bool active_ = false;
        };
    } // namespace Input
} // namespace SushiEngine
