/**************************************************************************/
/* cooked_asset_store.hpp                                                 */
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
 * @file cooked_asset_store.hpp
 * @brief The two places cooked blobs actually live.
 *
 * §8.1's cache, as the two implementations of @ref ICookedAssetStore a project needs:
 * one in memory, one on a disk. Neither is the "real" one — the editor uses the
 * filesystem store so a cook survives a restart, a unit test uses the memory store so
 * it leaves nothing behind, and the cooker cannot tell them apart, which is the whole
 * reason the seam exists.
 *
 * Both are safe to read from concurrently with a cook on another thread, since §8.1
 * puts cooking off the main thread while the editor keeps drawing.
 */

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <SushiEngine/physics/cooking/cooker_interface.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /**
             * @brief A store that keeps blobs in memory and forgets them on destruction.
             *
             * What a test uses, and what an editor session uses for an asset the project
             * has not been asked to keep. Also the reference implementation: it is short
             * enough to be obviously correct, so a disagreement between the two stores
             * is a bug in the other one.
             */
            class MemoryCookedAssetStore final : public ICookedAssetStore
            {
            public:
                bool contains(const CookedAssetKey& key) const override;
                bool load(const CookedAssetKey& key, std::vector<std::byte>& out) const override;
                bool store(const CookedAssetKey& key,
                           const std::vector<std::byte>& bytes) override;
                bool evict(const CookedAssetKey& key) override;

                /** @brief How many assets are held. */
                std::size_t size() const;

                /** @brief Forgets everything. */
                void clear();

            private:
                mutable std::mutex mutex_;
                std::unordered_map<std::uint64_t, std::vector<std::byte>> assets_;
            };

            /**
             * @brief A store that keeps blobs as files under one directory.
             *
             * The filename is the key's folded hash and the extension names the family,
             * so the directory is content-addressed: two projects cooking the same mesh
             * at the same fidelity produce the same filename, and a stale entry is
             * unreachable rather than wrong. The consequence is deliberate — an asset
             * whose key changed leaves its old file behind, which costs disk and never
             * costs correctness. Reclaiming it is a project's housekeeping decision, not
             * something a cooker should do behind an artist's back.
             */
            class FilesystemCookedAssetStore final : public ICookedAssetStore
            {
            public:
                /**
                 * @brief Points the store at a directory, creating it if it is absent.
                 *
                 * @param directory Where blobs are written. A path that cannot be created
                 *                  leaves the store non-functional rather than throwing:
                 *                  a cook against an unwritable cache must still produce
                 *                  its asset (see @ref ICookedAssetStore::store).
                 */
                explicit FilesystemCookedAssetStore(std::string directory);

                bool contains(const CookedAssetKey& key) const override;
                bool load(const CookedAssetKey& key, std::vector<std::byte>& out) const override;
                bool store(const CookedAssetKey& key,
                           const std::vector<std::byte>& bytes) override;
                bool evict(const CookedAssetKey& key) override;

                /** @brief Whether the directory exists and can be written to. */
                bool usable() const noexcept { return usable_; }

                /** @brief The directory blobs are written under. */
                const std::string& directory() const noexcept { return directory_; }

                /**
                 * @brief The path an asset with @p key would occupy.
                 *
                 * Public because the editor shows it and a test asserts on it, and
                 * because a cache whose layout is a secret is a cache nobody can clear.
                 *
                 * @param key The asset's identity.
                 * @return The full path, whether or not the file exists.
                 */
                std::string path_for(const CookedAssetKey& key) const;

            private:
                std::string directory_;
                bool usable_ = false;
                mutable std::mutex mutex_;
            };

            /** @brief The conventional extension for a cooked asset family, without a dot. */
            inline const char* cooked_asset_extension(CookedAssetKind kind) noexcept
            {
                switch (kind)
                {
                    case CookedAssetKind::Collision:
                        return "sushicollision";
                    case CookedAssetKind::SoftBody:
                        return "sushisoft";
                    case CookedAssetKind::NodeBeam:
                        return "sushinodebeam";
                }
                return "sushicooked";
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
