/**************************************************************************/
/* cooking_service.hpp                                                    */
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
 * @file cooking_service.hpp
 * @brief Cooking that happens *while the editor keeps drawing*.
 *
 * §8.1: *"Cooking runs off the main thread, reports progress, and is fully cached."* The
 * first clause is this file. It matters more than it looks: §13.1 budgets three seconds for
 * a fifty-thousand-triangle mesh at half fidelity, and three seconds of frozen editor per
 * dropped asset is a pipeline artists route around by not dropping assets in.
 *
 * One worker thread, not a pool. Two reasons, and the second is the real one. A cook is
 * already allocation-heavy and cache-hostile, so several at once mostly contend; and a
 * single worker makes the *order* of results a function of the submission order, which keeps
 * an import log readable and a test deterministic. Widening it later is a change to this
 * file and to nothing that uses it.
 *
 * The caller polls. No callbacks fire on the worker, because a callback that lands on a
 * background thread is a callback that eventually touches a renderer from the wrong one —
 * @ref CookingService::take_completed hands finished work to whoever asks, on their thread.
 */

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <SushiEngine/physics/cooking/import_profile.hpp>
#include <SushiEngine/physics/cooking/mesh_post_processor.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /** @brief Everything one import produced. */
            struct CookedImport
            {
                /** @brief The asset that was imported. */
                std::string asset_path;

                /** @brief Whether the loader could read geometry at all. */
                bool loaded = false;

                /** @brief Triangles the imported mesh held; zero when @ref loaded is false. */
                std::uint32_t source_triangle_count = 0;

                /** @brief One entry per processor that produced an asset, in chain order. */
                std::vector<MeshPostProcessResult> products;

                /** @brief The product of a given kind, or null when none was produced. */
                const MeshPostProcessResult* product(CookedAssetKind kind) const noexcept
                {
                    for (const MeshPostProcessResult& candidate : products)
                    {
                        if (candidate.kind == kind)
                            return &candidate;
                    }
                    return nullptr;
                }
            };

            /** @brief What the service is doing, for a progress display. */
            struct CookingServiceStatus
            {
                /** @brief The asset being cooked, or empty when idle. */
                std::string asset_path;

                /**
                 * @brief The running cooking stage's name, or empty.
                 *
                 * The stage and not the processor. There is no processor field because the
                 * chain does not report which of its members is running, and a field that is
                 * structurally always empty is the same failure as a made-up one — the stage
                 * name is also the more useful of the two, since "Tetrahedralize" says both
                 * what is happening and which cooker is doing it.
                 */
                std::string stage;

                /** @brief Stages completed within the current cook. */
                std::uint32_t completed_stages = 0;

                /** @brief Stages in the current cook. */
                std::uint32_t total_stages = 0;

                /** @brief Imports queued but not yet started. */
                std::size_t queued = 0;

                /** @brief Imports finished and not yet taken. */
                std::size_t completed = 0;

                /** @brief Whether a cook is in flight. */
                bool busy = false;
            };

            /**
             * @brief Runs the import chain on a worker thread, one asset at a time.
             *
             * The store and the chain are borrowed for the service's whole lifetime. The
             * service must be destroyed before either — it is the thing with a thread in it,
             * so it is the thing that has to go first, and its destructor joins.
             */
            class CookingService
            {
            public:
                /**
                 * @brief Starts the worker.
                 *
                 * @param loader Turns an asset path into triangles; must be safe to call on
                 *               the worker thread and must outlive the service.
                 * @param chain  The processors to run; borrowed, must outlive the service.
                 * @param store  The content-hash cache; may be null, which cooks every time.
                 */
                CookingService(MeshLoader loader, const MeshPostProcessorChain& chain,
                               ICookedAssetStore* store);

                /** @brief Stops the worker and joins it; queued work is abandoned. */
                ~CookingService();

                CookingService(const CookingService&) = delete;
                CookingService& operator=(const CookingService&) = delete;

                /**
                 * @brief Queues an asset for cooking.
                 *
                 * Returns immediately. Submitting the same path twice queues it twice — the
                 * service does not deduplicate, because the second submission usually means
                 * the file changed and the content hash is what decides whether that is real
                 * work (§8.1).
                 *
                 * @param asset_path The asset to import.
                 * @param profile    The resolved import profile.
                 */
                void submit(std::string asset_path, ImportProfile profile);

                /**
                 * @brief Takes every finished import.
                 *
                 * Non-blocking, and it empties the completed queue: results are handed over
                 * once, so a caller polling every frame sees each import exactly once and does
                 * not have to track what it has already seen.
                 *
                 * @return The finished imports, in submission order.
                 */
                std::vector<CookedImport> take_completed();

                /**
                 * @brief Blocks until nothing is queued and nothing is in flight.
                 *
                 * For a build machine, which has no frame to keep drawing, and for a test,
                 * which needs the work to have happened before it asserts. Not for the editor.
                 */
                void wait_until_idle();

                /** @brief A snapshot of what the service is doing. */
                CookingServiceStatus status() const;

            private:
                struct Job
                {
                    std::string asset_path;
                    ImportProfile profile;
                };

                void worker_loop();

                MeshLoader loader_;
                const MeshPostProcessorChain* chain_;
                ICookedAssetStore* store_;

                mutable std::mutex mutex_;
                std::condition_variable work_available_;
                std::condition_variable work_finished_;
                std::deque<Job> queue_;
                std::vector<CookedImport> completed_;
                CookingServiceStatus status_;
                bool stopping_ = false;
                std::thread worker_;
            };
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
