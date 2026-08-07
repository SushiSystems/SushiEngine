/**************************************************************************/
/* lru_cache.hpp                                                          */
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
 * @file lru_cache.hpp
 * @brief A fixed-capacity least-recently-used cache, generic over key and value.
 *
 * Backed by a doubly linked list ordered most- to least-recently-used, plus a hash index from
 * key to that list's iterator — so both @ref touch and @ref insert are O(1), the property that
 * makes an LRU eviction policy worth choosing over just walking a vector. Not thread-safe: the
 * caller (the thumbnail cache's main-thread upload step) is the only place this is touched.
 */

#include <cstddef>
#include <list>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace SushiEngine
{
    namespace Imaging
    {
        template <typename Key, typename Value>
        class LruCache
        {
            public:
                explicit LruCache(std::size_t capacity) : capacity_(capacity) {}

                /**
                 * @brief Looks up @p key, promoting it to most-recently-used on a hit.
                 * @return A pointer to the value, valid until the next @ref insert that evicts
                 *         it or the cache is destroyed; @c nullptr if @p key is absent.
                 */
                Value* touch(const Key& key)
                {
                    const auto found = index_.find(key);
                    if (found == index_.end())
                        return nullptr;
                    order_.splice(order_.begin(), order_, found->second);
                    return &found->second->second;
                }

                /**
                 * @brief Inserts or overwrites @p key as most-recently-used.
                 * @return The evicted least-recently-used entry if this insert pushed the cache
                 *         past capacity; @c std::nullopt otherwise (including when @p key
                 *         already existed, which never evicts).
                 */
                std::optional<std::pair<Key, Value>> insert(const Key& key, Value value)
                {
                    const auto found = index_.find(key);
                    if (found != index_.end())
                    {
                        found->second->second = std::move(value);
                        order_.splice(order_.begin(), order_, found->second);
                        return std::nullopt;
                    }

                    order_.emplace_front(key, std::move(value));
                    index_[key] = order_.begin();

                    if (order_.size() <= capacity_)
                        return std::nullopt;

                    std::pair<Key, Value> evicted = std::move(order_.back());
                    index_.erase(evicted.first);
                    order_.pop_back();
                    return evicted;
                }

                /** @brief How many entries are resident right now. */
                std::size_t size() const noexcept { return order_.size(); }

                /** @brief Empties the cache, returning every entry it held. */
                std::vector<std::pair<Key, Value>> drain()
                {
                    std::vector<std::pair<Key, Value>> all;
                    all.reserve(order_.size());
                    for (auto& entry : order_)
                        all.push_back(std::move(entry));
                    order_.clear();
                    index_.clear();
                    return all;
                }

            private:
                std::size_t capacity_;
                // Front = most recently used, back = least recently used.
                std::list<std::pair<Key, Value>> order_;
                std::unordered_map<Key, typename std::list<std::pair<Key, Value>>::iterator>
                    index_;
        };
    } // namespace Imaging
} // namespace SushiEngine
