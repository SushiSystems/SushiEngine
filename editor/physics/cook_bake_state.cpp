/**************************************************************************/
/* cook_bake_state.cpp                                                    */
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

#include "cook_bake_state.hpp"

#include <algorithm>
#include <utility>

namespace SushiEngine
{
    namespace Editor
    {
        CookBakeState::CookBakeState(Physics::Cooking::MeshLoader loader,
                                     const std::string& cache_directory)
            : chain_(Physics::Cooking::MeshPostProcessorChain::with_shipped_processors())
        {
            if (cache_directory.empty())
            {
                store_ = std::make_unique<Physics::Cooking::MemoryCookedAssetStore>();
            }
            else
            {
                auto filesystem_store =
                    std::make_unique<Physics::Cooking::FilesystemCookedAssetStore>(
                        cache_directory);
                // A cache directory that cannot be created makes cooking slow, not impossible,
                // so the memory store stands in rather than the panel refusing to work.
                if (filesystem_store->usable())
                    store_ = std::move(filesystem_store);
                else
                    store_ = std::make_unique<Physics::Cooking::MemoryCookedAssetStore>();
            }

            // The loader is moved into the service, which owns the worker that calls it.
            service_ = std::make_unique<Physics::Cooking::CookingService>(std::move(loader),
                                                                         chain_, store_.get());
        }

        CookBakeState::~CookBakeState()
        {
            // Explicit and ordered: the service has a thread that touches the store and the
            // chain, so it has to be gone before either. Declaration order would give this for
            // free, and saying it here is cheaper than finding out it did not.
            service_.reset();
        }

        void CookBakeState::bake(const std::string& asset_path)
        {
            if (asset_path.empty())
                return;
            service_->submit(asset_path, profiles_.resolve(asset_path));
        }

        void CookBakeState::rebake(const std::string& asset_path)
        {
            if (asset_path.empty())
                return;

            // The eviction itself happens inside the processor, which is the first place the
            // mesh and the cooker that owns the key are both in hand — the key needs the
            // source's content hash, and the source is behind the loader on the worker thread.
            Physics::Cooking::ImportProfile profile = profiles_.resolve(asset_path);
            profile.force_recook = true;
            service_->submit(asset_path, std::move(profile));
        }

        std::size_t CookBakeState::poll()
        {
            const std::vector<Physics::Cooking::CookedImport> imports =
                service_->take_completed();
            for (const Physics::Cooking::CookedImport& imported : imports)
            {
                BakedAssetEntry entry_value;
                entry_value.asset_path = imported.asset_path;
                entry_value.loaded = imported.loaded;
                entry_value.source_triangle_count = imported.source_triangle_count;

                const Physics::Cooking::MeshPostProcessResult* collision =
                    imported.product(Physics::Cooking::CookedAssetKind::Collision);
                if (collision != nullptr)
                {
                    entry_value.collision_bytes = collision->bytes;
                    entry_value.collision_report = collision->report;
                }
                const Physics::Cooking::MeshPostProcessResult* soft =
                    imported.product(Physics::Cooking::CookedAssetKind::SoftBody);
                if (soft != nullptr)
                {
                    entry_value.soft_body_bytes = soft->bytes;
                    entry_value.soft_body_report = soft->report;
                }

                // One entry per asset, replaced in place: a re-cook of a crate should update
                // the crate's row rather than add a second one the artist has to tell apart.
                const auto existing =
                    std::find_if(entries_.begin(), entries_.end(),
                                 [&imported](const BakedAssetEntry& candidate)
                                 { return candidate.asset_path == imported.asset_path; });
                if (existing != entries_.end())
                    *existing = std::move(entry_value);
                else
                    entries_.insert(entries_.begin(), std::move(entry_value));

                if (selected_.empty() || selected_ == imported.asset_path)
                {
                    selected_ = imported.asset_path;
                    refresh_wireframe();
                }
            }
            return imports.size();
        }

        const BakedAssetEntry* CookBakeState::entry(const std::string& asset_path) const
        {
            for (const BakedAssetEntry& candidate : entries_)
            {
                if (candidate.asset_path == asset_path)
                    return &candidate;
            }
            return nullptr;
        }

        void CookBakeState::select(const std::string& asset_path)
        {
            if (selected_ == asset_path)
                return;
            selected_ = asset_path;
            refresh_wireframe();
        }

        bool CookBakeState::busy() const
        {
            const Physics::Cooking::CookingServiceStatus current = service_->status();
            return current.busy || current.queued > 0;
        }

        void CookBakeState::refresh_wireframe()
        {
            wireframe_.clear();
            const BakedAssetEntry* current = entry(selected_);
            if (current == nullptr || !current->has_collision())
                return;

            const Physics::Cooking::CollisionAssetView view =
                Physics::Cooking::load_collision_blob(current->collision_bytes.data(),
                                                      current->collision_bytes.size());
            if (!view.valid)
                return;
            Physics::Cooking::collision_asset_wireframe(view, wireframe_);
        }
    } // namespace Editor
} // namespace SushiEngine
