/**************************************************************************/
/* cook_bake_state.hpp                                                    */
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
 * @file cook_bake_state.hpp
 * @brief What the Bake panel knows, with no ImGui anywhere near it.
 *
 * §14's collider and soft-body inspectors need a fidelity slider, a Bake button, live
 * progress, and the cook report — and every one of those is a *decision* the panel makes
 * before it draws anything. So the decisions live here, in a translation unit that links no
 * UI, the way the scene serializer and the command history already do.
 *
 * That split is not tidiness. A bake surface's interesting behaviour is "does pressing
 * Re-cook actually get past the cache", "does the overlay follow the asset that is
 * selected", "does a failed cook still leave a report to look at" — and none of that is
 * testable through an ImGui call. The panel that draws this is a hundred lines of widgets
 * over a class that is tested.
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/cooking/collision_asset.hpp>
#include <SushiEngine/physics/cooking/cooked_asset_store.hpp>
#include <SushiEngine/physics/cooking/cooking_service.hpp>
#include <SushiEngine/physics/cooking/import_profile.hpp>
#include <SushiEngine/physics/cooking/mesh_post_processor.hpp>
#include <SushiEngine/physics/cooking/node_beam_asset.hpp>
#include <SushiEngine/physics/cooking/soft_body_asset.hpp>

namespace SushiEngine
{
    namespace Authoring
    {
        /** @brief One asset's most recent cook, as the panel shows it. */
        struct BakedAssetEntry
        {
            /** @brief The asset path, as the project spells it. */
            std::string asset_path;

            /** @brief Whether the mesh could be read at all. */
            bool loaded = false;

            /** @brief Triangles the source mesh held. */
            std::uint32_t source_triangle_count = 0;

            /** @brief The collision product, when one was produced. */
            std::vector<std::byte> collision_bytes;

            /** @brief Its report; @c status says whether it shipped. */
            Physics::Cooking::CookingReport collision_report;

            /** @brief The soft-body product, when one was produced. */
            std::vector<std::byte> soft_body_bytes;

            /** @brief Its report. */
            Physics::Cooking::CookingReport soft_body_report;

            /** @brief The node-beam product, when one was produced. */
            std::vector<std::byte> node_beam_bytes;

            /** @brief Its report. */
            Physics::Cooking::CookingReport node_beam_report;

            /** @brief Whether a collision asset exists to inspect or draw. */
            bool has_collision() const noexcept { return !collision_bytes.empty(); }

            /** @brief Whether a soft-body asset exists to inspect. */
            bool has_soft_body() const noexcept { return !soft_body_bytes.empty(); }

            /** @brief Whether a node-beam asset exists to inspect. */
            bool has_node_beam() const noexcept { return !node_beam_bytes.empty(); }
        };

        /**
         * @brief The Bake panel's model: the profile, the queue, and the last results.
         *
         * Owns the store, the chain and the worker service, because the panel is the only
         * thing in the editor that cooks and giving the context a cooking service would put a
         * worker thread in a struct every panel includes.
         */
        class CookBakeState
        {
        public:
            /**
             * @brief Builds the state, its cache directory, and starts the worker.
             *
             * @param loader          Turns an asset path into triangles; the application wires
             *                        `Geometry::import_gltf_mesh` in.
             * @param cache_directory Where cooked blobs live between sessions; an empty path
             *                        keeps them in memory only.
             */
            CookBakeState(Physics::Cooking::MeshLoader loader, const std::string& cache_directory);

            ~CookBakeState();

            CookBakeState(const CookBakeState&) = delete;
            CookBakeState& operator=(const CookBakeState&) = delete;

            /** @brief The project default every asset without an override is cooked at. */
            Physics::Cooking::ImportProfileLibrary& profiles() noexcept { return profiles_; }

            /**
             * @brief Where the project's cooking profile is stored; empty means not persisted.
             *
             * Set once at startup, after the project root is known — which is after this
             * object is constructed, hence a setter rather than a constructor parameter.
             * `profiles()` returning a mutable reference means nothing here can intercept a
             * write to record it automatically, so callers that mutate the profile (the Bake
             * panel, the Cooking Override modal) call @ref save_profiles themselves once they
             * are done, the same way they already call `set_project_default`/`set_override`.
             *
             * @param path The file to load from and save to.
             */
            void set_profile_storage_path(const std::string& path) { profile_storage_path_ = path; }

            /**
             * @brief Resets the profile set to defaults, discarding the project default and
             * every override.
             *
             * `load_profiles` merges into whatever is already here and does nothing when the
             * target file does not exist yet, so a caller switching to a different project must
             * call this first — otherwise the previous project's cooking settings would carry
             * over into a project that has never touched the Bake panel.
             */
            void reset_profiles() { profiles_ = Physics::Cooking::ImportProfileLibrary(); }

            /**
             * @brief Loads the project's cooking profile from its storage path.
             *
             * A no-op, succeeding, when no path is set or the file does not exist yet — a
             * project that has never touched the Bake panel has nothing to load, and that is
             * not a failure.
             *
             * @return False only when the path exists but could not be parsed.
             */
            bool load_profiles();

            /**
             * @brief Saves the project's cooking profile to its storage path.
             * @return False when a path is set and the write failed; true (including a no-op)
             *         otherwise.
             */
            bool save_profiles() const;

            /** @brief The project default, read-only. */
            const Physics::Cooking::ImportProfileLibrary& profiles() const noexcept
            {
                return profiles_;
            }

            /**
             * @brief Queues @p asset_path for cooking at its resolved profile.
             *
             * @param asset_path The asset to cook.
             */
            void bake(const std::string& asset_path);

            /**
             * @brief Queues @p asset_path, dropping any cached asset for it first.
             *
             * §14's "Re-cook" button. It has to evict, because the point of pressing it is to
             * get past a cache entry whose key has *not* changed — which is the case whenever
             * the cooker itself is being worked on, and the one case the content hash cannot
             * detect on the artist's behalf.
             *
             * @param asset_path The asset to re-cook.
             */
            void rebake(const std::string& asset_path);

            /**
             * @brief Takes finished cooks out of the service and files them.
             *
             * Called once per frame by the panel. Separate from drawing so the results land
             * whether or not the panel is open — a bake the artist started and then closed the
             * window on must still finish and still be there.
             *
             * @return How many imports were filed.
             */
            std::size_t poll();

            /** @brief What the service is doing, for the progress readout. */
            Physics::Cooking::CookingServiceStatus status() const { return service_->status(); }

            /** @brief The assets cooked this session, most recent first. */
            const std::vector<BakedAssetEntry>& entries() const noexcept { return entries_; }

            /** @brief The entry for @p asset_path, or null when it has not been cooked. */
            const BakedAssetEntry* entry(const std::string& asset_path) const;

            /** @brief Which entry the panel is showing; empty for none. */
            const std::string& selected() const noexcept { return selected_; }

            /** @brief Selects the entry the panel shows and the overlay draws. */
            void select(const std::string& asset_path);

            /**
             * @brief The selected asset's collision geometry as line segments.
             *
             * Six floats per segment, in the asset's own frame. Rebuilt only when the
             * selection changes rather than per frame: the hull faces are reconstructed from
             * the stored point set (the asset carries none, deliberately), and doing that
             * every frame would be paying a cook-time cost at frame rate.
             */
            const std::vector<float>& collision_wireframe() const noexcept
            {
                return wireframe_;
            }

            /** @brief Whether a bake is queued or running. */
            bool busy() const;

        private:
            void refresh_wireframe();

            Physics::Cooking::ImportProfileLibrary profiles_;
            std::string profile_storage_path_;
            std::unique_ptr<Physics::Cooking::ICookedAssetStore> store_;
            Physics::Cooking::MeshPostProcessorChain chain_;
            std::unique_ptr<Physics::Cooking::CookingService> service_;
            std::vector<BakedAssetEntry> entries_;
            std::string selected_;
            std::vector<float> wireframe_;
        };
    } // namespace Authoring
} // namespace SushiEngine
