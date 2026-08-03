/**************************************************************************/
/* test_byte_encoding.cpp                                                 */
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

// The serialization module's byte codec, which a cooked soft-body asset rides
// into and out of a `.sushiscene` file. The assertions that earn their keep are
// not the round trip — every codec passes that the day it is written — but the
// refusals: a truncated or mistyped blob that decodes to *something* is a soft
// body that comes back subtly wrong, which is worse than one that comes back
// missing. So every malformed shape is checked, and each is checked to leave the
// caller's own buffer alone rather than half-overwritten.
//
// The expected texts are RFC 4648's own `foobar` vectors and the published
// FNV-1a 64 ones, so the tests are anchored outside this repository rather than
// transcribed from the implementation they guard.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "byte_encoding.hpp"

using SushiEngine::Scene::Detail::content_hash;
using SushiEngine::Scene::Detail::decode_base64;
using SushiEngine::Scene::Detail::encode_base64;

namespace
{
    /** @brief The characters of @p text as a blob. */
    std::vector<std::byte> blob(const std::string& text)
    {
        std::vector<std::byte> bytes;
        bytes.reserve(text.size());
        for (const char character : text)
            bytes.push_back(static_cast<std::byte>(static_cast<unsigned char>(character)));
        return bytes;
    }

    /** @brief Every byte value once, so no bit pattern goes untested. */
    std::vector<std::byte> every_byte_value()
    {
        std::vector<std::byte> bytes;
        bytes.reserve(256);
        for (int value = 0; value < 256; ++value)
            bytes.push_back(static_cast<std::byte>(value));
        return bytes;
    }

    /** @brief Asserts @p text is refused and that the caller's blob survives untouched. */
    void expect_refused(const std::string& text)
    {
        std::vector<std::byte> out = blob("keep me");
        EXPECT_FALSE(decode_base64(text, out)) << "accepted malformed text: " << text;
        // Compared as one boolean rather than element-wise: a blob is opaque, so a
        // per-byte report would be noise, and it keeps std::byte off the printer.
        EXPECT_TRUE(out == blob("keep me"))
            << "a refusal wrote to the caller's blob: " << text;
    }
} // namespace

TEST(Unit_ByteEncoding, EncodesTheReferenceVectors)
{
    // RFC 4648 §10, which covers all three tail lengths and the empty input.
    EXPECT_EQ(encode_base64(blob("")), "");
    EXPECT_EQ(encode_base64(blob("f")), "Zg==");
    EXPECT_EQ(encode_base64(blob("fo")), "Zm8=");
    EXPECT_EQ(encode_base64(blob("foo")), "Zm9v");
    EXPECT_EQ(encode_base64(blob("foob")), "Zm9vYg==");
    EXPECT_EQ(encode_base64(blob("fooba")), "Zm9vYmE=");
    EXPECT_EQ(encode_base64(blob("foobar")), "Zm9vYmFy");
}

TEST(Unit_ByteEncoding, DecodesTheReferenceVectors)
{
    const std::string texts[] = {"", "Zg==", "Zm8=", "Zm9v", "Zm9vYg==", "Zm9vYmE=", "Zm9vYmFy"};
    const std::string expected[] = {"", "f", "fo", "foo", "foob", "fooba", "foobar"};
    for (std::size_t i = 0; i < 7; ++i)
    {
        std::vector<std::byte> out;
        ASSERT_TRUE(decode_base64(texts[i], out)) << texts[i];
        EXPECT_TRUE(out == blob(expected[i])) << texts[i];
    }
}

TEST(Unit_ByteEncoding, RoundTripsEveryTailLength)
{
    // Lengths 0 through 8 cover each of the three tails twice over, so a tail
    // handled correctly only at the smallest size still fails here.
    const std::vector<std::byte> source = every_byte_value();
    for (std::size_t length = 0; length <= 8; ++length)
    {
        const std::vector<std::byte> bytes(source.begin(),
                                           source.begin() + static_cast<std::ptrdiff_t>(length));
        std::vector<std::byte> out;
        ASSERT_TRUE(decode_base64(encode_base64(bytes), out)) << "length " << length;
        EXPECT_TRUE(out == bytes) << "length " << length;
    }
}

TEST(Unit_ByteEncoding, RoundTripsEveryByteValue)
{
    // 256 bytes is not a multiple of three, so this also exercises a one-byte tail
    // on top of every bit pattern a blob can hold.
    const std::vector<std::byte> bytes = every_byte_value();
    std::vector<std::byte> out;
    ASSERT_TRUE(decode_base64(encode_base64(bytes), out));
    EXPECT_TRUE(out == bytes);
}

TEST(Unit_ByteEncoding, AnEmptyTextDecodesToAnEmptyBlob)
{
    // Not a refusal: an empty asset is a legitimate value that must survive the
    // trip as itself, distinct from an asset that failed to decode.
    std::vector<std::byte> out = blob("stale");
    ASSERT_TRUE(decode_base64("", out));
    EXPECT_TRUE(out.empty());
}

TEST(Unit_ByteEncoding, RefusesATruncatedText)
{
    // A length that is not a multiple of four is exactly what a truncated write or
    // a hand-edited file produces.
    expect_refused("Z");
    expect_refused("Zg");
    expect_refused("Zm9");
    expect_refused("Zm9vY");
    expect_refused("Zm9vYg=");
}

TEST(Unit_ByteEncoding, RefusesACharacterOutsideTheAlphabet)
{
    expect_refused("Zm9 ");
    expect_refused("Zm9*");
    expect_refused("Zm9-");
    expect_refused("Zm9\n");
    // The URL-safe alphabet is a different encoding, not a tolerated spelling of
    // this one: accepting `-`/`_` here would decode two texts to one blob.
    expect_refused("Zm9_");
}

TEST(Unit_ByteEncoding, RefusesPaddingAnywhereButTheTail)
{
    expect_refused("Zg=A");
    expect_refused("Z===");
    expect_refused("====");
    expect_refused("Zm9vYg==Zm9v");
    expect_refused("=Zm9");
}

TEST(Unit_ByteEncoding, RefusesNonZeroSpareBitsInAPaddedQuartet)
{
    // "Zg==" is the only spelling of the byte 'f'; "Zh==" carries the same byte
    // with four spare bits set. Accepting it would make the encoding ambiguous,
    // which for a content-hashed blob means two hashes for one asset.
    expect_refused("Zh==");
    expect_refused("Zm9=");
}

TEST(Unit_ByteEncoding, HashesTheReferenceVectors)
{
    // The published FNV-1a 64 values, which also pin the offset basis: an empty
    // blob must hash to the basis rather than to zero.
    EXPECT_EQ(content_hash(blob("")), std::uint64_t(0xcbf29ce484222325ull));
    EXPECT_EQ(content_hash(blob("a")), std::uint64_t(0xaf63dc4c8601ec8cull));
    EXPECT_EQ(content_hash(blob("foobar")), std::uint64_t(0x85944171f73967e8ull));
}

TEST(Unit_ByteEncoding, HashesContentRatherThanIdentity)
{
    // The property the blob table depends on: two independently built copies of
    // one asset collapse onto one entry, and a one-bit difference does not.
    EXPECT_EQ(content_hash(every_byte_value()), content_hash(every_byte_value()));

    std::vector<std::byte> altered = every_byte_value();
    altered[128] = static_cast<std::byte>(0x81);
    EXPECT_NE(content_hash(altered), content_hash(every_byte_value()));

    // Length is part of the content, so a prefix is not the same asset.
    EXPECT_NE(content_hash(blob("foobar")), content_hash(blob("fooba")));
}
