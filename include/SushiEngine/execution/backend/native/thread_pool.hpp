/**************************************************************************/
/* thread_pool.hpp                                                        */
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
/* permissions and limitations under the License.                        */
/**************************************************************************/

#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace SushiEngine
{
    namespace Execution
    {
        namespace NativeBackend
        {
            /**
             * @brief Runs the ready nodes of one dependency wave concurrently.
             *
             * `DagCompiler::run` is a wave-scheduled Kahn's-algorithm executor: at
             * any moment, every node in the current wave has no unfinished
             * predecessor and therefore no declared access overlapping any other
             * node in the same wave (the hazard tracker's ordering guarantees this
             * by construction) — so the whole wave is safe to run concurrently, one
             * node per worker, with no synchronization *within* a node's own
             * `kernel(0..capacity)` loop, which runs sequentially on whichever
             * single worker picked the node up.
             *
             * This is deliberately node-granular, not element-granular: a worker
             * that takes a node runs its whole element range before taking another,
             * rather than workers splitting one node's range between them. The ECS
             * emits one node per chunk (§4.2 of the cross-platform plan; default
             * capacity 1024) — thousands of small, independent nodes is the actual
             * shape of the workload this pool serves, and node-level dispatch draws
             * real parallelism from that without needing a second, finer-grained
             * work-stealing layer this domain does not yet need.
             *
             * `worker_count() <= 1` spawns no background threads at all; `run_batch`
             * then executes every job in order on the calling thread. This is not a
             * fallback path bolted onto a "real" implementation — it is what makes
             * the `{1, 2, max}`-worker bit-determinism obligation (UHM §7.3)
             * structural: a node's own kernel never depends on how many *other*
             * nodes ran concurrently with it, only on its own sequential loop, so
             * the total result cannot depend on worker count.
             */
            class ThreadPool
            {
                public:
                    /**
                     * @brief Spawns @p worker_count background workers (zero, for
                     * @p worker_count <= 1: `run_batch` then executes synchronously).
                     * @param worker_count Requested concurrency; clamped to at least 1.
                     */
                    explicit ThreadPool(std::size_t worker_count);

                    /** @brief Signals every worker to stop and joins them. */
                    ~ThreadPool();

                    ThreadPool(const ThreadPool&) = delete;
                    ThreadPool& operator=(const ThreadPool&) = delete;

                    /** @brief The concurrency this pool was constructed with. */
                    std::size_t worker_count() const noexcept { return worker_count_; }

                    /**
                     * @brief Runs every job in @p jobs to completion, then returns.
                     *
                     * Jobs are independent by the caller's own contract (this class
                     * enforces nothing about their access patterns — @ref
                     * NativeBackend::DagCompiler is the caller that guarantees it).
                     * Blocks the calling thread until every job has finished.
                     *
                     * @param jobs The wave's ready nodes' bound closures.
                     */
                    void run_batch(std::vector<std::function<void()>> jobs);

                private:
                    void worker_loop();

                    std::size_t worker_count_;
                    std::vector<std::thread> workers_;

                    std::mutex mutex_;
                    std::condition_variable work_available_;
                    std::condition_variable batch_done_;
                    std::vector<std::function<void()>> queue_;
                    std::size_t pending_ = 0;
                    bool stop_ = false;
            };
        } // namespace NativeBackend
    } // namespace Execution
} // namespace SushiEngine
