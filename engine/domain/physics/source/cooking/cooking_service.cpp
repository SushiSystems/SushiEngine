/**************************************************************************/
/* cooking_service.cpp                                                    */
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

#include <SushiEngine/physics/cooking/cooking_service.hpp>

#include <utility>

#include <SushiEngine/geometry/triangle_mesh.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            namespace
            {
                /**
                 * @brief Publishes a cook's stage into the service's status, under its lock.
                 *
                 * A sink rather than the service implementing @ref ICookingProgressSink
                 * itself: the sink is called from inside a cook on the worker, and letting the
                 * service *be* the sink would put a public interface on the class whose only
                 * caller is its own worker thread.
                 */
                class StatusProgressSink final : public ICookingProgressSink
                {
                public:
                    StatusProgressSink(std::mutex& mutex, CookingServiceStatus& status)
                        : mutex_(mutex), status_(status)
                    {
                    }

                    void on_progress(const CookingProgress& progress) override
                    {
                        const std::lock_guard<std::mutex> guard(mutex_);
                        status_.stage = progress.stage != nullptr ? progress.stage : "";
                        status_.completed_stages = progress.completed_stages;
                        status_.total_stages = progress.total_stages;
                    }

                private:
                    std::mutex& mutex_;
                    CookingServiceStatus& status_;
                };
            } // namespace

            CookingService::CookingService(MeshLoader loader,
                                          const MeshPostProcessorChain& chain,
                                          ICookedAssetStore* store)
                : loader_(std::move(loader)), chain_(&chain), store_(store)
            {
                worker_ = std::thread([this] { worker_loop(); });
            }

            CookingService::~CookingService()
            {
                {
                    const std::lock_guard<std::mutex> guard(mutex_);
                    stopping_ = true;
                    // Queued work is abandoned rather than drained. A destructor that finished
                    // the queue would make closing the editor take as long as the largest cook
                    // still pending, and the cache means nothing is lost — the next session
                    // re-submits and cooks what it must.
                    queue_.clear();
                }
                work_available_.notify_all();
                if (worker_.joinable())
                    worker_.join();
            }

            void CookingService::submit(std::string asset_path, ImportProfile profile)
            {
                {
                    const std::lock_guard<std::mutex> guard(mutex_);
                    if (stopping_)
                        return;
                    queue_.push_back(Job{std::move(asset_path), std::move(profile)});
                    status_.queued = queue_.size();
                }
                work_available_.notify_one();
            }

            std::vector<CookedImport> CookingService::take_completed()
            {
                std::vector<CookedImport> taken;
                const std::lock_guard<std::mutex> guard(mutex_);
                taken.swap(completed_);
                status_.completed = 0;
                return taken;
            }

            void CookingService::wait_until_idle()
            {
                std::unique_lock<std::mutex> lock(mutex_);
                work_finished_.wait(lock,
                                    [this] { return queue_.empty() && !status_.busy; });
            }

            CookingServiceStatus CookingService::status() const
            {
                const std::lock_guard<std::mutex> guard(mutex_);
                return status_;
            }

            void CookingService::worker_loop()
            {
                StatusProgressSink sink(mutex_, status_);
                for (;;)
                {
                    Job job;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        work_available_.wait(lock,
                                             [this] { return stopping_ || !queue_.empty(); });
                        if (stopping_)
                            return;
                        job = std::move(queue_.front());
                        queue_.pop_front();
                        status_.queued = queue_.size();
                        status_.asset_path = job.asset_path;
                        status_.stage.clear();
                        status_.completed_stages = 0;
                        status_.total_stages = 0;
                        status_.busy = true;
                    }

                    CookedImport import_result;
                    import_result.asset_path = job.asset_path;

                    Geometry::TriangleMesh mesh;
                    // The load is on the worker too. Reading a hundred-megabyte glTF is not
                    // obviously cheaper than cooking it, and doing it on the caller's thread
                    // would move the stall rather than remove it.
                    import_result.loaded = loader_ && loader_(job.asset_path, mesh);
                    if (import_result.loaded)
                    {
                        import_result.source_triangle_count =
                            std::uint32_t(mesh.triangle_count());
                        import_result.products =
                            chain_->run(mesh.view(), job.profile, store_, &sink);
                    }

                    {
                        const std::lock_guard<std::mutex> guard(mutex_);
                        completed_.push_back(std::move(import_result));
                        status_.completed = completed_.size();
                        status_.asset_path.clear();
                        status_.stage.clear();
                        status_.completed_stages = 0;
                        status_.total_stages = 0;
                        status_.busy = false;
                    }
                    // Notified after `busy` is cleared, or a waiter woken between the two would
                    // see an empty queue and a busy service and go back to sleep with nothing
                    // left to wake it.
                    work_finished_.notify_all();
                }
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
