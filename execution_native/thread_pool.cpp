/**************************************************************************/
/* thread_pool.cpp                                                        */
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

#include <SushiEngine/execution/backend/native/thread_pool.hpp>

#include <utility>

namespace SushiEngine
{
    namespace Execution
    {
        namespace NativeBackend
        {
            ThreadPool::ThreadPool(std::size_t worker_count)
                : worker_count_(worker_count == 0 ? 1 : worker_count)
            {
                if (worker_count_ <= 1)
                    return; // synchronous mode: no background threads at all

                workers_.reserve(worker_count_);
                for (std::size_t i = 0; i < worker_count_; ++i)
                    workers_.emplace_back([this] { worker_loop(); });
            }

            ThreadPool::~ThreadPool()
            {
                if (workers_.empty())
                    return;

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    stop_ = true;
                }
                work_available_.notify_all();
                for (std::thread& worker : workers_)
                    worker.join();
            }

            void ThreadPool::run_batch(std::vector<std::function<void()>> jobs)
            {
                if (jobs.empty())
                    return;

                if (workers_.empty())
                {
                    for (std::function<void()>& job : jobs)
                        job();
                    return;
                }

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    pending_ = jobs.size();
                    for (std::function<void()>& job : jobs)
                        queue_.push_back(std::move(job));
                }
                work_available_.notify_all();

                std::unique_lock<std::mutex> lock(mutex_);
                batch_done_.wait(lock, [this] { return pending_ == 0; });
            }

            void ThreadPool::worker_loop()
            {
                for (;;)
                {
                    std::function<void()> job;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        work_available_.wait(lock, [this] { return stop_ || !queue_.empty(); });
                        if (queue_.empty())
                        {
                            if (stop_)
                                return;
                            continue;
                        }
                        job = std::move(queue_.back());
                        queue_.pop_back();
                    }

                    job();

                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        --pending_;
                        if (pending_ == 0)
                            batch_done_.notify_one();
                    }
                }
            }
        } // namespace NativeBackend
    } // namespace Execution
} // namespace SushiEngine
