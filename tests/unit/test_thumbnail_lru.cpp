/**************************************************************************/
/* test_thumbnail_lru.cpp                                                 */
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

// Unit_ThumbnailLru: the thumbnail cache's eviction bookkeeping, driven with plain int keys and
// values so the test has no Vulkan/ImGui dependency at all — the property under test is purely
// "which entry goes away," never what a resident thumbnail happens to hold.

#include <gtest/gtest.h>

#include "SushiEngine/imaging/lru_cache.hpp"

using SushiEngine::Imaging::LruCache;

TEST(Unit_ThumbnailLru, InsertingPastCapacityEvictsTheOldestEntry)
{
    LruCache<int, std::string> cache(2);
    EXPECT_EQ(cache.insert(1, "a"), std::nullopt);
    EXPECT_EQ(cache.insert(2, "b"), std::nullopt);

    // Cache holds {1, 2}; inserting 3 must evict 1, the least recently touched.
    const std::optional<std::pair<int, std::string>> evicted = cache.insert(3, "c");
    ASSERT_TRUE(evicted.has_value());
    EXPECT_EQ(evicted->first, 1);
    EXPECT_EQ(evicted->second, "a");
    EXPECT_EQ(cache.size(), std::size_t(2));
    EXPECT_EQ(*cache.touch(2), "b");
    EXPECT_EQ(*cache.touch(3), "c");
    EXPECT_EQ(cache.touch(1), nullptr);
}

TEST(Unit_ThumbnailLru, TouchingAnEntryProtectsItFromTheNextEviction)
{
    LruCache<int, std::string> cache(2);
    cache.insert(1, "a");
    cache.insert(2, "b");

    // Touching 1 makes 2 the least recently used, even though 2 was inserted more recently.
    ASSERT_NE(cache.touch(1), nullptr);

    const std::optional<std::pair<int, std::string>> evicted = cache.insert(3, "c");
    ASSERT_TRUE(evicted.has_value());
    EXPECT_EQ(evicted->first, 2);
    EXPECT_NE(cache.touch(1), nullptr);
    EXPECT_NE(cache.touch(3), nullptr);
}

TEST(Unit_ThumbnailLru, ReinsertingAnExistingKeyOverwritesWithoutEvicting)
{
    LruCache<int, std::string> cache(2);
    cache.insert(1, "a");
    cache.insert(2, "b");

    EXPECT_EQ(cache.insert(1, "a-updated"), std::nullopt);
    EXPECT_EQ(cache.size(), std::size_t(2));
    EXPECT_EQ(*cache.touch(1), "a-updated");
}

TEST(Unit_ThumbnailLru, DrainEmptiesTheCacheAndReturnsEveryEntry)
{
    LruCache<int, std::string> cache(4);
    cache.insert(1, "a");
    cache.insert(2, "b");

    const std::vector<std::pair<int, std::string>> drained = cache.drain();
    EXPECT_EQ(drained.size(), std::size_t(2));
    EXPECT_EQ(cache.size(), std::size_t(0));
    EXPECT_EQ(cache.touch(1), nullptr);
    EXPECT_EQ(cache.touch(2), nullptr);
}
