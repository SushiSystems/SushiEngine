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
         * @brief What one @ref CookBakeState::load_profiles moved out of the cooking document.
         *
         * A project written before per-asset overrides lived beside their assets carries them
         * in a path-keyed object instead. Reading such a project moves them, and the two lists
         * are how the move says so once rather than happening invisibly — the dropped one in
         * particular, since an override whose asset is gone is settings the artist loses.
         */
        struct CookingOverrideMigration
        {
            /** @brief Asset paths whose `.meta` sidecar now carries what the document held. */
            std::vector<std::string> migrated;

            /** @brief Asset paths the document named that are no longer on disk. */
            std::vector<std::string> dropped;

            /** @brief Whether anything moved, so a caller can stay silent when nothing did. */
            bool empty() const noexcept { return migrated.empty() && dropped.empty(); }
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
             * @brief Where the project's cooking default is stored; empty means not persisted.
             *
             * Set once at startup, after the project root is known — which is after this
             * object is constructed, hence a setter rather than a constructor parameter.
             * `profiles()` returning a mutable reference means nothing here can intercept a
             * write to record it automatically, so the Bake panel calls @ref save_profiles
             * itself once it is done, the same way it already calls `set_project_default`.
             *
             * The document holds the project default alone. What one asset says differs lives
             * in that asset's own `<asset>.meta` sidecar, so renaming or moving the asset
             * carries its settings rather than orphaning them
             * (`docs/design/model_import.md` §4.2).
             *
             * An override recorded through `ImportProfileLibrary::set_override` is therefore a
             * session-only edit: it decides the cooks this session triggers (see @ref bake),
             * and @ref save_profiles does not write it anywhere. Persisting one means writing
             * the asset's sidecar with `Model::save_model_import_settings`, which the Cooking
             * Override modal does not yet do.
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
            void reset_profiles()
            {
                profiles_ = Physics::Cooking::ImportProfileLibrary();
                last_migration_ = CookingOverrideMigration();
            }

            /**
             * @brief Loads the project's cooking default from its storage path.
             *
             * Moves any per-asset overrides an older project document still carries into their
             * assets' `.meta` sidecars first, once, and reads the default out of what is left.
             * @ref last_migration says what moved.
             *
             * A no-op, succeeding, when no path is set or the file does not exist yet — a
             * project that has never touched the Bake panel has nothing to load, and that is
             * not a failure.
             *
             * @return False only when the path exists but could not be parsed, or when the
             *         migration could not complete.
             */
            bool load_profiles();

            /**
             * @brief What the last @ref load_profiles moved out of the cooking document.
             *
             * Empty for a project that had nothing to move, which is every project after the
             * first read. Exposed rather than logged here because this module links no console.
             *
             * @return The migrated and dropped asset paths.
             */
            const CookingOverrideMigration& last_migration() const noexcept
            {
                return last_migration_;
            }

            /**
             * @brief Saves the project's cooking default to its storage path.
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
             * The profile is the project default with the asset's `.meta` folded over it, and
             * then whatever @ref profiles has recorded for it this session — so an edit that
             * has not been written to the sidecar yet still decides the cook it triggers.
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

            // The profile one asset is cooked at: the project default, the asset's own
            // `.meta`, then this session's unsaved edit, each folded over the last.
            Physics::Cooking::ImportProfile resolved_profile(const std::string& asset_path) const;

            Physics::Cooking::ImportProfileLibrary profiles_;
            CookingOverrideMigration last_migration_;
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
