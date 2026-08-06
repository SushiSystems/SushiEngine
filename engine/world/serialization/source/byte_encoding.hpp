/**************************************************************************/
/* byte_encoding.hpp                                                      */
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

#ifndef SUSHIENGINE_SCENE_BYTE_ENCODING_HPP
#define SUSHIENGINE_SCENE_BYTE_ENCODING_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @file byte_encoding.hpp
 * @brief Putting raw bytes into a JSON document, and naming them by their content.
 *
 * Private to the serialization module. JSON has no byte-string type, so a blob a
 * scene file carries by value has to become text; a blob an in-memory snapshot
 * refers to instead needs a stable name derived from the bytes themselves. Both
 * live here because both exist for exactly one reason — the cooked soft-body
 * asset a `SoftBodyParameters` holds by value — and neither is general enough to
 * belong in a shared foundation until a second consumer asks for it.
 */

namespace SushiEngine
{
    namespace Scene
    {
        namespace Detail
        {
            /**
             * @brief Encodes @p bytes as standard base64 (RFC 4648, `+/` alphabet, padded).
             *
             * Three bytes become four characters; a tail of one or two bytes is padded
             * with `=` to a full quartet, so the encoding is self-delimiting and a
             * decoder recovers the exact original length.
             *
             * @param bytes The blob to encode; may be empty.
             * @return The base64 text, empty exactly when @p bytes is empty.
             */
            std::string encode_base64(const std::vector<std::byte>& bytes);

            /**
             * @brief Decodes base64 text produced by @ref encode_base64.
             *
             * Strict by design: a length that is not a multiple of four, a character
             * outside the alphabet, padding anywhere but the tail, or non-zero spare
             * bits in a padded quartet are all refusals. A silently truncated soft-body
             * asset is worse than a missing one — the entity would come back as a body
             * that can never be built, with nothing to say why.
             *
             * @param text The base64 text; an empty string decodes to an empty blob.
             * @param out  Receives the decoded bytes on success; left untouched on
             *     refusal, so a caller can keep whatever it already had.
             * @return False when @p text is not well-formed base64.
             */
            bool decode_base64(const std::string& text, std::vector<std::byte>& out);

            /**
             * @brief FNV-1a 64 over @p bytes, for naming a blob by its content.
             *
             * The same algorithm the cooking pipeline hashes its cache keys with, so a
             * reader comparing the two sees one hash function rather than two that
             * happen to be similar.
             *
             * @param bytes The blob to hash; an empty one hashes to the offset basis.
             * @return The hash, usable as an @ref SushiEngine::Scene::ISceneBlobTable key.
             */
            std::uint64_t content_hash(const std::vector<std::byte>& bytes) noexcept;

            /**
             * @brief FNV-1a 64 over @p text's bytes.
             *
             * The same function as the byte overload, over the other thing this module
             * hashes: a document's serialized text. One hash for both keeps a prefab's
             * revision and a blob's key comparable rather than merely similar.
             *
             * @param text The text to hash; an empty one hashes to the offset basis.
             * @return The hash.
             */
            std::uint64_t content_hash(const std::string& text) noexcept;
        } // namespace Detail
    } // namespace Scene
} // namespace SushiEngine

#endif
