/**************************************************************************/
/* voice_render_pool.hpp                                                  */
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

#ifndef SUSHIENGINE_AUDIO_VOICE_RENDER_POOL_HPP
#define SUSHIENGINE_AUDIO_VOICE_RENDER_POOL_HPP

/**
 * @file voice_render_pool.hpp
 * @brief A small worker pool that spreads per-voice DSP across CPU cores.
 *
 * A single core caps how many voices can be filtered/resampled/propagated per block; AAA
 * engines spread that work over a job pool (§8 of `docs/slop/audio_system.md`). This is a
 * minimal, persistent pool the @ref VoiceManager uses to parallelise the **embarrassingly
 * parallel** phase of a block — each real voice renders its own source and per-voice
 * filters into its *own* scratch buffer, touching no shared state — while the mixdown
 * (bus accumulation, ambisonic encode) stays serial on the audio thread. Output is
 * therefore identical to the single-threaded path; only the heavy DSP moves off-core.
 *
 * @ref dispatch splits `[0, count)` across the workers **plus the calling (audio) thread**,
 * so N workers give N+1 lanes, and blocks the caller until every lane finishes. Workers
 * sleep on a condition variable between blocks (no idle spinning); the per-block wake is a
 * brief, uncontended barrier, not a data lock on the mix.
 */

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief A persistent pool of voice-DSP worker threads. */
        class VoiceRenderPool
        {
            public:
                /**
                 * @brief Spawns the worker threads.
                 * @param worker_count Number of workers (the audio thread is an extra lane);
                 *                      0 asks for one fewer than the hardware concurrency.
                 */
                explicit VoiceRenderPool(int worker_count = 0)
                {
                    if (worker_count <= 0)
                    {
                        const unsigned hw = std::thread::hardware_concurrency();
                        worker_count = hw > 2 ? static_cast<int>(hw) - 1 : 1;
                    }
                    worker_count_ = worker_count;
                    workers_.reserve(static_cast<std::size_t>(worker_count_));
                    for (int w = 0; w < worker_count_; ++w)
                        workers_.emplace_back([this, w]() { worker_loop(w); });
                }

                ~VoiceRenderPool()
                {
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        running_ = false;
                        ++epoch_;
                    }
                    go_.notify_all();
                    for (std::thread& t : workers_)
                        if (t.joinable())
                            t.join();
                }

                VoiceRenderPool(const VoiceRenderPool&) = delete;
                VoiceRenderPool& operator=(const VoiceRenderPool&) = delete;

                /** @brief The number of parallel lanes (workers + the calling thread). */
                int lanes() const noexcept { return worker_count_ + 1; }

                /**
                 * @brief Runs `job(i)` for every i in [0, count) across all lanes; blocks until done.
                 *
                 * The audio thread itself processes one lane while the workers process the
                 * rest, so no core sits idle. `job` must be safe to call concurrently for
                 * distinct indices (the voice DSP writes only its own voice's scratch).
                 *
                 * @tparam Job A callable `void(int index)`.
                 * @param count Number of work items.
                 * @param job   The per-item work.
                 */
                template <typename Job>
                void dispatch(int count, Job&& job)
                {
                    if (count <= 0)
                        return;

                    // Erase the job's type behind a trampoline so the workers can call it
                    // without the pool being a template (and with no per-block allocation).
                    using JobType = typename std::remove_reference<Job>::type;
                    auto trampoline = [](void* context, int index)
                    {
                        (*static_cast<JobType*>(context))(index);
                    };
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        job_context_ = static_cast<void*>(&job);
                        job_fn_ = trampoline;
                        count_ = count;
                        pending_ = worker_count_;
                        ++epoch_;
                    }
                    go_.notify_all();

                    // The audio thread takes the last lane.
                    run_lane(worker_count_, count, job_context_, job_fn_);

                    std::unique_lock<std::mutex> lock(mutex_);
                    done_.wait(lock, [this]() { return pending_ == 0; });
                }

            private:
                void run_lane(int lane, int count, void* context, void (*fn)(void*, int)) noexcept
                {
                    const int lanes = worker_count_ + 1;
                    for (int i = lane; i < count; i += lanes)
                        fn(context, i);
                }

                void worker_loop(int lane)
                {
                    std::uint64_t seen = 0;
                    for (;;)
                    {
                        void* context = nullptr;
                        void (*fn)(void*, int) = nullptr;
                        int count = 0;
                        {
                            std::unique_lock<std::mutex> lock(mutex_);
                            go_.wait(lock, [this, seen]() { return epoch_ != seen || !running_; });
                            if (!running_)
                                return;
                            seen = epoch_;
                            context = job_context_;
                            fn = job_fn_;
                            count = count_;
                        }
                        run_lane(lane, count, context, fn);
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            if (--pending_ == 0)
                                done_.notify_one();
                        }
                    }
                }

                std::vector<std::thread> workers_;
                std::mutex mutex_;
                std::condition_variable go_;
                std::condition_variable done_;
                std::uint64_t epoch_ = 0;
                int worker_count_ = 0;
                int count_ = 0;
                int pending_ = 0;
                void* job_context_ = nullptr;
                void (*job_fn_)(void*, int) = nullptr;
                bool running_ = true;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
