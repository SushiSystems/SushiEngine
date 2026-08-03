/**************************************************************************/
/* animation_database.hpp                                                 */
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
 * @file animation_database.hpp
 * @brief The asset-domain seam: hands out immutable skeleton views by asset id.
 *
 * @ref IAnimationDatabase is the dependency-inversion boundary the simulation and the
 * evaluator sit behind (design §3): they ask for a @ref SkeletonView by id and never
 * see the importer, the file format, or the byte buffers. @ref AnimationDatabase is
 * the concrete owner — it holds each cooked blob in shared, stable storage and returns
 * views that alias it, valid for the database's lifetime. Clip and controller views
 * join this interface in later phases (A1, A3); the id space is shared across all three
 * asset kinds.
 */

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include <SushiEngine/animation/asset_id.hpp>
#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/avatar_mask.hpp>
#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/clip_compress.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief Read-only access to loaded animation assets, keyed by @ref AssetId.
         *
         * The abstraction the runtime depends on. Kept deliberately narrow: an
         * implementer exposes exactly what a consumer reads and nothing about how assets
         * were cooked or stored. Clip and controller accessors extend this interface when
         * those asset kinds land.
         */
        class IAnimationDatabase
        {
            public:
                virtual ~IAnimationDatabase() = default;

                /**
                 * @brief The skeleton named by an id.
                 * @param id A handle from a loader (see @ref AnimationDatabase::add_skeleton).
                 * @return Its view, or an invalid view if @p id names no skeleton.
                 */
                virtual SkeletonView skeleton(AssetId id) const = 0;

                /**
                 * @brief Whether an id names a loaded skeleton.
                 * @param id The handle to test.
                 * @return True if @ref skeleton would return a valid view for @p id.
                 */
                virtual bool has_skeleton(AssetId id) const = 0;

                /**
                 * @brief The clip named by an id.
                 * @param id A handle from a loader (see @ref AnimationDatabase::add_clip).
                 * @return Its view, or an invalid view if @p id names no clip.
                 */
                virtual ClipView clip(AssetId id) const = 0;

                /**
                 * @brief Whether an id names a loaded clip.
                 * @param id The handle to test.
                 * @return True if @ref clip would return a valid view for @p id.
                 */
                virtual bool has_clip(AssetId id) const = 0;

                /**
                 * @brief The controller named by an id.
                 * @param id A handle from a loader (see @ref AnimationDatabase::add_controller).
                 * @return Its view, or an invalid view if @p id names no controller.
                 */
                virtual ControllerView controller(AssetId id) const = 0;

                /**
                 * @brief Whether an id names a loaded controller.
                 * @param id The handle to test.
                 * @return True if @ref controller would return a valid view for @p id.
                 */
                virtual bool has_controller(AssetId id) const = 0;

                /**
                 * @brief The avatar mask named by an id.
                 * @param id A handle from a loader (see @ref AnimationDatabase::add_mask).
                 * @return Its view, or an invalid view if @p id names no mask.
                 */
                virtual MaskView mask(AssetId id) const = 0;

                /**
                 * @brief Whether an id names a loaded avatar mask.
                 * @param id The handle to test.
                 * @return True if @ref mask would return a valid view for @p id.
                 */
                virtual bool has_mask(AssetId id) const = 0;
        };

        /**
         * @brief In-memory owner of cooked animation blobs behind @ref IAnimationDatabase.
         *
         * Each blob lives in its own heap buffer, so the buffers do not move when the
         * database grows and the views handed out stay valid for the database's lifetime.
         * Non-copyable: it is the single owner of the asset bytes.
         */
        class AnimationDatabase : public IAnimationDatabase
        {
            public:
                AnimationDatabase() = default;

                AnimationDatabase(const AnimationDatabase&) = delete;
                AnimationDatabase& operator=(const AnimationDatabase&) = delete;
                AnimationDatabase(AnimationDatabase&&) = default;
                AnimationDatabase& operator=(AnimationDatabase&&) = default;

                /**
                 * @brief Takes ownership of a cooked skeleton blob and registers it.
                 * @param blob A `.sushiskel` byte buffer (moved in).
                 * @return The id naming it, or @ref INVALID_ASSET if the blob is malformed.
                 */
                AssetId add_skeleton(std::vector<std::byte> blob)
                {
                    Entry entry;
                    entry.kind = AssetKind::Skeleton;
                    entry.skeleton = load_skeleton_blob(blob.data(), blob.size());
                    if (!entry.skeleton.valid())
                        return INVALID_ASSET;
                    entry.blob = std::move(blob);
                    const AssetId id = static_cast<AssetId>(assets_.size());
                    assets_.push_back(std::move(entry));
                    return id;
                }

                /**
                 * @brief Takes ownership of a cooked clip blob and registers it.
                 * @param blob A `.sushianim` byte buffer (moved in).
                 * @return The id naming it, or @ref INVALID_ASSET if the blob is malformed.
                 */
                AssetId add_clip(std::vector<std::byte> blob)
                {
                    Entry entry;
                    entry.kind = AssetKind::Clip;
                    entry.clip = load_any_clip_blob(blob.data(), blob.size());
                    if (!entry.clip.valid())
                        return INVALID_ASSET;
                    entry.blob = std::move(blob);
                    const AssetId id = static_cast<AssetId>(assets_.size());
                    assets_.push_back(std::move(entry));
                    return id;
                }

                /**
                 * @brief Reads a `.sushiskel` file and registers the skeleton it holds.
                 * @param path Filesystem path to the cooked blob.
                 * @return The id naming it, or @ref INVALID_ASSET if the file is missing or malformed.
                 */
                AssetId load_skeleton_file(const std::string& path)
                {
                    std::vector<std::byte> blob;
                    return read_file(path, blob) ? add_skeleton(std::move(blob)) : INVALID_ASSET;
                }

                /**
                 * @brief Takes ownership of a cooked controller blob and registers it.
                 * @param blob A `.sushictrl` byte buffer (moved in).
                 * @return The id naming it, or @ref INVALID_ASSET if the blob is malformed.
                 */
                AssetId add_controller(std::vector<std::byte> blob)
                {
                    Entry entry;
                    entry.kind = AssetKind::Controller;
                    entry.controller = load_controller_blob(blob.data(), blob.size());
                    if (!entry.controller.valid())
                        return INVALID_ASSET;
                    entry.blob = std::move(blob);
                    const AssetId id = static_cast<AssetId>(assets_.size());
                    assets_.push_back(std::move(entry));
                    return id;
                }

                /**
                 * @brief Reads a `.sushianim` file and registers the clip it holds.
                 * @param path Filesystem path to the cooked blob.
                 * @return The id naming it, or @ref INVALID_ASSET if the file is missing or malformed.
                 */
                AssetId load_clip_file(const std::string& path)
                {
                    std::vector<std::byte> blob;
                    return read_file(path, blob) ? add_clip(std::move(blob)) : INVALID_ASSET;
                }

                /**
                 * @brief Reads a `.sushictrl` file and registers the controller it holds.
                 * @param path Filesystem path to the cooked blob.
                 * @return The id naming it, or @ref INVALID_ASSET if the file is missing or malformed.
                 */
                AssetId load_controller_file(const std::string& path)
                {
                    std::vector<std::byte> blob;
                    return read_file(path, blob) ? add_controller(std::move(blob)) : INVALID_ASSET;
                }

                /**
                 * @brief Takes ownership of a cooked avatar-mask blob and registers it.
                 * @param blob A `.sushimask` byte buffer (moved in).
                 * @return The id naming it, or @ref INVALID_ASSET if the blob is malformed.
                 */
                AssetId add_mask(std::vector<std::byte> blob)
                {
                    Entry entry;
                    entry.kind = AssetKind::Mask;
                    entry.mask = load_mask_blob(blob.data(), blob.size());
                    if (!entry.mask.valid())
                        return INVALID_ASSET;
                    entry.blob = std::move(blob);
                    const AssetId id = static_cast<AssetId>(assets_.size());
                    assets_.push_back(std::move(entry));
                    return id;
                }

                /**
                 * @brief Reads a `.sushimask` file and registers the mask it holds.
                 * @param path Filesystem path to the cooked blob.
                 * @return The id naming it, or @ref INVALID_ASSET if the file is missing or malformed.
                 */
                AssetId load_mask_file(const std::string& path)
                {
                    std::vector<std::byte> blob;
                    return read_file(path, blob) ? add_mask(std::move(blob)) : INVALID_ASSET;
                }

                SkeletonView skeleton(AssetId id) const override
                {
                    if (id >= assets_.size() || assets_[id].kind != AssetKind::Skeleton)
                        return SkeletonView{};
                    return assets_[id].skeleton;
                }

                bool has_skeleton(AssetId id) const override
                {
                    return id < assets_.size() && assets_[id].kind == AssetKind::Skeleton &&
                           assets_[id].skeleton.valid();
                }

                ClipView clip(AssetId id) const override
                {
                    if (id >= assets_.size() || assets_[id].kind != AssetKind::Clip)
                        return ClipView{};
                    return assets_[id].clip;
                }

                bool has_clip(AssetId id) const override
                {
                    return id < assets_.size() && assets_[id].kind == AssetKind::Clip &&
                           assets_[id].clip.valid();
                }

                ControllerView controller(AssetId id) const override
                {
                    if (id >= assets_.size() || assets_[id].kind != AssetKind::Controller)
                        return ControllerView{};
                    return assets_[id].controller;
                }

                bool has_controller(AssetId id) const override
                {
                    return id < assets_.size() && assets_[id].kind == AssetKind::Controller &&
                           assets_[id].controller.valid();
                }

                MaskView mask(AssetId id) const override
                {
                    if (id >= assets_.size() || assets_[id].kind != AssetKind::Mask)
                        return MaskView{};
                    return assets_[id].mask;
                }

                bool has_mask(AssetId id) const override
                {
                    return id < assets_.size() && assets_[id].kind == AssetKind::Mask &&
                           assets_[id].mask.valid();
                }

                /** @brief Number of assets registered, of every kind. */
                std::size_t asset_count() const noexcept { return assets_.size(); }

            private:
                /** @brief The asset kind an @ref Entry holds. */
                enum class AssetKind
                {
                    Skeleton,
                    Clip,
                    Controller,
                    Mask
                };

                /** @brief One owned blob and the view (of its kind) that aliases it. */
                struct Entry
                {
                    AssetKind kind = AssetKind::Skeleton; /**< Which view below is populated. */
                    std::vector<std::byte> blob;          /**< Owns the bytes the view points into. */
                    SkeletonView skeleton{};              /**< Valid when @c kind is Skeleton. */
                    ClipView clip{};                      /**< Valid when @c kind is Clip. */
                    ControllerView controller{};          /**< Valid when @c kind is Controller. */
                    MaskView mask{};                      /**< Valid when @c kind is Mask. */
                };

                /**
                 * @brief Reads a whole file into a byte buffer.
                 * @param path The file to read.
                 * @param out  Receives the bytes on success.
                 * @return True if the file was read; false if missing or empty.
                 */
                static bool read_file(const std::string& path, std::vector<std::byte>& out)
                {
                    std::ifstream file(path, std::ios::binary | std::ios::ate);
                    if (!file)
                        return false;
                    const std::streamsize size = file.tellg();
                    if (size <= 0)
                        return false;
                    file.seekg(0, std::ios::beg);
                    out.resize(static_cast<std::size_t>(size));
                    return static_cast<bool>(file.read(reinterpret_cast<char*>(out.data()), size));
                }

                std::vector<Entry> assets_;
        };
    } // namespace Animation
} // namespace SushiEngine
