/**************************************************************************/
/* byte_encoding.cpp                                                      */
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

#include "byte_encoding.hpp"

#include <array>
#include <utility>

namespace SushiEngine
{
    namespace Scene
    {
        namespace Detail
        {
            namespace
            {
                constexpr char ALPHABET[] =
                    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

                /**
                 * @brief Character to sextet, or -1 for anything not in the alphabet.
                 *
                 * A 256-entry table built from @c ALPHABET rather than a chain of range
                 * comparisons: it is one indexed read per character over blobs that run
                 * to megabytes, and it cannot disagree with the encoder's alphabet
                 * because it is derived from it.
                 */
                const std::array<signed char, 256>& decode_table() noexcept
                {
                    static const std::array<signed char, 256> table = []
                    {
                        std::array<signed char, 256> built{};
                        built.fill(-1);
                        for (int index = 0; index < 64; ++index)
                            built[static_cast<unsigned char>(ALPHABET[index])] =
                                static_cast<signed char>(index);
                        return built;
                    }();
                    return table;
                }

                /** @brief One byte of the blob as a plain integer. */
                std::uint32_t octet(std::byte value) noexcept
                {
                    return static_cast<std::uint32_t>(std::to_integer<unsigned char>(value));
                }
            } // namespace

            std::string encode_base64(const std::vector<std::byte>& bytes)
            {
                std::string text;
                text.reserve(((bytes.size() + 2) / 3) * 4);

                const std::size_t whole = bytes.size() - bytes.size() % 3;
                for (std::size_t index = 0; index < whole; index += 3)
                {
                    const std::uint32_t group = (octet(bytes[index]) << 16) |
                                                (octet(bytes[index + 1]) << 8) |
                                                octet(bytes[index + 2]);
                    text.push_back(ALPHABET[(group >> 18) & 0x3Fu]);
                    text.push_back(ALPHABET[(group >> 12) & 0x3Fu]);
                    text.push_back(ALPHABET[(group >> 6) & 0x3Fu]);
                    text.push_back(ALPHABET[group & 0x3Fu]);
                }

                const std::size_t tail = bytes.size() - whole;
                if (tail == 1)
                {
                    const std::uint32_t group = octet(bytes[whole]) << 16;
                    text.push_back(ALPHABET[(group >> 18) & 0x3Fu]);
                    text.push_back(ALPHABET[(group >> 12) & 0x3Fu]);
                    text.push_back('=');
                    text.push_back('=');
                }
                else if (tail == 2)
                {
                    const std::uint32_t group =
                        (octet(bytes[whole]) << 16) | (octet(bytes[whole + 1]) << 8);
                    text.push_back(ALPHABET[(group >> 18) & 0x3Fu]);
                    text.push_back(ALPHABET[(group >> 12) & 0x3Fu]);
                    text.push_back(ALPHABET[(group >> 6) & 0x3Fu]);
                    text.push_back('=');
                }
                return text;
            }

            bool decode_base64(const std::string& text, std::vector<std::byte>& out)
            {
                if (text.size() % 4 != 0)
                    return false;

                // Padding is a property of the whole text, not of a quartet, so it is
                // measured once here; an '=' anywhere else fails the alphabet lookup
                // below, which is the same refusal by a different route.
                std::size_t padding = 0;
                if (!text.empty() && text[text.size() - 1] == '=')
                {
                    ++padding;
                    if (text[text.size() - 2] == '=')
                        ++padding;
                }

                const std::array<signed char, 256>& table = decode_table();
                const auto sextet = [&table](char character) noexcept
                {
                    return static_cast<int>(table[static_cast<unsigned char>(character)]);
                };

                const std::size_t quartets = text.size() / 4;
                const std::size_t whole = padding == 0 ? quartets : quartets - 1;

                // Decoded into a local and moved out only on success, which is what lets
                // a refusal leave the caller's blob exactly as it found it.
                std::vector<std::byte> decoded;
                decoded.reserve(quartets * 3);

                for (std::size_t quartet = 0; quartet < whole; ++quartet)
                {
                    const std::size_t at = quartet * 4;
                    const int first = sextet(text[at]);
                    const int second = sextet(text[at + 1]);
                    const int third = sextet(text[at + 2]);
                    const int fourth = sextet(text[at + 3]);
                    if (first < 0 || second < 0 || third < 0 || fourth < 0)
                        return false;
                    const std::uint32_t group =
                        (static_cast<std::uint32_t>(first) << 18) |
                        (static_cast<std::uint32_t>(second) << 12) |
                        (static_cast<std::uint32_t>(third) << 6) |
                        static_cast<std::uint32_t>(fourth);
                    decoded.push_back(static_cast<std::byte>((group >> 16) & 0xFFu));
                    decoded.push_back(static_cast<std::byte>((group >> 8) & 0xFFu));
                    decoded.push_back(static_cast<std::byte>(group & 0xFFu));
                }

                if (padding != 0)
                {
                    const std::size_t at = whole * 4;
                    const int first = sextet(text[at]);
                    const int second = sextet(text[at + 1]);
                    if (first < 0 || second < 0)
                        return false;
                    if (padding == 2)
                    {
                        // Two characters carry twelve bits for one byte. The four spare
                        // bits must be zero: a non-zero remainder is text no encoder
                        // produced, and accepting it would decode two different strings
                        // to the same blob.
                        if ((second & 0x0F) != 0)
                            return false;
                        decoded.push_back(
                            static_cast<std::byte>(((first << 2) | (second >> 4)) & 0xFF));
                    }
                    else
                    {
                        const int third = sextet(text[at + 2]);
                        if (third < 0 || (third & 0x03) != 0)
                            return false;
                        decoded.push_back(
                            static_cast<std::byte>(((first << 2) | (second >> 4)) & 0xFF));
                        decoded.push_back(
                            static_cast<std::byte>(((second << 4) | (third >> 2)) & 0xFF));
                    }
                }

                out = std::move(decoded);
                return true;
            }

            std::uint64_t content_hash(const std::vector<std::byte>& bytes) noexcept
            {
                std::uint64_t hash = 14695981039346656037ull;
                for (const std::byte value : bytes)
                {
                    hash ^= static_cast<std::uint64_t>(std::to_integer<unsigned char>(value));
                    hash *= 1099511628211ull;
                }
                return hash;
            }
        } // namespace Detail
    } // namespace Scene
} // namespace SushiEngine
