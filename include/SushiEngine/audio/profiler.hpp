/**************************************************************************/
/* profiler.hpp                                                          */
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

#ifndef SUSHIENGINE_AUDIO_PROFILER_HPP
#define SUSHIENGINE_AUDIO_PROFILER_HPP

/**
 * @file profiler.hpp
 * @brief The audio→GUI telemetry channel: the live-profiler snapshot the editor reads.
 *
 * The editor's live profiler (§11 of `docs/slop/audio_system.md`) shows what the audio
 * thread is doing — how many voices are real vs virtual, the meter on each bus, the master
 * level, a scope of the output — but it must never *touch* the audio thread's state under a
 * lock. So the audio thread **publishes** a plain-POD @ref AudioProfileSnapshot once per
 * block into a @ref AudioProfiler, and the GUI thread **reads the latest** whenever it
 * repaints, over a wait-free-for-the-writer **seqlock** (§0's one-way telemetry ring, in
 * its simplest single-latest-value form): the audio thread never blocks, and the reader
 * only ever spins if it happens to catch a publish mid-flight (a microsecond at worst).
 *
 * The snapshot is a fixed-size POD (no allocation, trivially copyable), so it crosses the
 * seqlock by value. Portable, no SDL and no SushiRuntime — the channel unit-tests with two
 * plain function calls standing in for the two threads.
 */

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief The most buses a snapshot carries meters for. */
        constexpr int PROFILE_MAX_BUSES = 32;
        /** @brief The number of downsampled scope points a snapshot carries. */
        constexpr int PROFILE_SCOPE_POINTS = 128;

        /** @brief One bus's meters in a @ref AudioProfileSnapshot. */
        struct BusMeter
        {
            float peak = 0.0f; /**< Post-fader absolute peak this block. */
            float rms = 0.0f;  /**< Post-fader RMS this block. */
        };

        /**
         * @brief One block's worth of audio-thread telemetry (a trivially-copyable POD).
         *
         * Published by the audio thread each render and read by the GUI. All levels are
         * linear; @ref cpu_load is the fraction of the block's wall-clock budget the render
         * consumed (0 when not measured). @ref scope is a downsampled mono view of the
         * master output for a waveform display.
         */
        struct AudioProfileSnapshot
        {
            std::uint64_t block_index = 0; /**< Monotonic block counter (freshness / dropped-frame check). */
            int real_voices = 0;           /**< Voices rendered this block. */
            int virtual_voices = 0;        /**< Active-but-inaudible voices. */
            int active_voices = 0;         /**< real + virtual. */
            float master_peak = 0.0f;      /**< Master output absolute (sample) peak. */
            float master_true_peak = 0.0f; /**< Inter-sample (true) peak, 4× oversampled. */
            float master_rms = 0.0f;       /**< Master output RMS. */
            float master_lufs = -70.0f;    /**< Momentary loudness (K-weighted, ITU-R BS.1770), LUFS. */
            float cpu_load = 0.0f;         /**< Render time / block budget, in [0, ~] (0 = unmeasured). */
            int bus_count = 0;             /**< Number of valid entries in @ref buses. */
            BusMeter buses[PROFILE_MAX_BUSES];
            int scope_points = 0;          /**< Number of valid entries in @ref scope. */
            float scope[PROFILE_SCOPE_POINTS];
        };

        /**
         * @brief A single-latest-value channel from the audio thread to the GUI.
         *
         * The audio thread calls @ref publish once per block (wait-free); the GUI calls
         * @ref latest to copy the most recently published snapshot (spins only if it races a
         * publish). A classic seqlock: an odd sequence marks a write in progress, so a
         * reader that sees an odd count, or a count that changed across its copy, retries.
         */
        class AudioProfiler
        {
            public:
                /**
                 * @brief Publishes a snapshot (audio thread; wait-free).
                 * @param snapshot The block's telemetry.
                 */
                void publish(const AudioProfileSnapshot& snapshot) noexcept
                {
                    const std::uint32_t s = sequence_.load(std::memory_order_relaxed);
                    sequence_.store(s + 1, std::memory_order_release); // now odd: writing
                    std::atomic_thread_fence(std::memory_order_release);
                    data_ = snapshot;
                    std::atomic_thread_fence(std::memory_order_release);
                    sequence_.store(s + 2, std::memory_order_release); // even: done
                }

                /**
                 * @brief Copies the most recent snapshot (GUI thread).
                 * @param out Filled with the latest published snapshot.
                 * @return True once a snapshot has ever been published.
                 */
                bool latest(AudioProfileSnapshot& out) const noexcept
                {
                    for (int attempt = 0; attempt < 16; ++attempt)
                    {
                        const std::uint32_t before = sequence_.load(std::memory_order_acquire);
                        if (before == 0)
                            return false; // nothing published yet
                        if (before & 1u)
                            continue; // a publish is in progress
                        out = data_;
                        std::atomic_thread_fence(std::memory_order_acquire);
                        const std::uint32_t after = sequence_.load(std::memory_order_acquire);
                        if (before == after)
                            return true;
                    }
                    return false; // repeatedly raced (writer far faster than reader); try later
                }

                /** @brief The number of blocks published so far. */
                std::uint32_t published_count() const noexcept
                {
                    return sequence_.load(std::memory_order_acquire) / 2;
                }

            private:
                std::atomic<std::uint32_t> sequence_{0};
                AudioProfileSnapshot data_{};
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
