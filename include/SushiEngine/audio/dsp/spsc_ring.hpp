/**************************************************************************/
/* spsc_ring.hpp                                                          */
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

#ifndef SUSHIENGINE_AUDIO_DSP_SPSC_RING_HPP
#define SUSHIENGINE_AUDIO_DSP_SPSC_RING_HPP

/**
 * @file spsc_ring.hpp
 * @brief The lock-free single-producer/single-consumer ring — the workhorse that
 *        joins the control plane to the audio-render plane.
 *
 * This is the one queue of the two-plane model (see `docs/design/audio_system.md`
 * §0, §3.2): the control (game/ECS) thread pushes small trivially-copyable command
 * records — play, stop, set-parameter targets, or *pointers* to immutable pre-built
 * data — and the audio thread drains them at the top of each block. It is also used
 * the other way for meters and for the "garbage" ring that hands retired pointers
 * back to a worker thread so the audio thread never calls `delete`.
 *
 * Wait-free on both ends: @ref push and @ref pop each touch only their own index
 * with a release store and read the other with an acquire load, so there is no CAS,
 * no lock, and no blocking — exactly what the audio thread's hard real-time
 * discipline requires. Correct for **one** producer and **one** consumer only.
 */

#include <atomic>
#include <cstddef>
#include <type_traits>
#include <vector>

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            /**
             * @brief A bounded, wait-free SPSC ring of trivially-copyable @p T.
             *
             * The capacity is rounded up to a power of two so index wrap is a mask,
             * not a modulo, and every slot is usable (fullness is tracked by the
             * absolute index difference, not a sacrificed slot). The two indices sit
             * on separate cache lines (`alignas(64)`) so the producer and consumer
             * never contend on the same line — the false-sharing trap that otherwise
             * halves throughput.
             *
             * @tparam T The element type. Must be trivially copyable: the ring is a
             * raw byte pipe with no construction/destruction on the hot path, matching
             * the audio thread's no-allocation rule.
             */
            template <typename T>
            class SpscRing
            {
                public:
                    static_assert(std::is_trivially_copyable<T>::value,
                                  "SpscRing<T> requires a trivially-copyable T (real-time byte pipe).");

                    /**
                     * @brief Builds a ring with capacity at least @p minimum_capacity.
                     * @param minimum_capacity The least number of elements the ring must hold;
                     *                          the actual capacity is the next power of two ≥ this
                     *                          (and ≥ 2).
                     */
                    explicit SpscRing(std::size_t minimum_capacity)
                        : buffer_(round_up_pow2(minimum_capacity)),
                          mask_(buffer_.size() - 1)
                    {
                    }

                    /**
                     * @brief Producer side: enqueues @p value if space remains.
                     * @param value The element to copy into the ring.
                     * @return True if enqueued; false if the ring is full (value dropped).
                     */
                    bool push(const T& value) noexcept
                    {
                        const std::size_t write = write_.load(std::memory_order_relaxed);
                        const std::size_t read = read_.load(std::memory_order_acquire);
                        if (write - read == buffer_.size())
                            return false; // full
                        buffer_[write & mask_] = value;
                        write_.store(write + 1, std::memory_order_release);
                        return true;
                    }

                    /**
                     * @brief Consumer side: dequeues the oldest element if any.
                     * @param out Set to the dequeued element when this returns true.
                     * @return True if an element was dequeued; false if the ring is empty.
                     */
                    bool pop(T& out) noexcept
                    {
                        const std::size_t read = read_.load(std::memory_order_relaxed);
                        const std::size_t write = write_.load(std::memory_order_acquire);
                        if (read == write)
                            return false; // empty
                        out = buffer_[read & mask_];
                        read_.store(read + 1, std::memory_order_release);
                        return true;
                    }

                    /** @brief Total number of elements the ring can hold (a power of two). */
                    std::size_t capacity() const noexcept { return buffer_.size(); }

                    /**
                     * @brief Approximate number of queued elements.
                     *
                     * A snapshot across two atomics; exact only when one side is quiescent,
                     * which is the normal case for meters/diagnostics.
                     *
                     * @return The producer/consumer index difference at the time of the call.
                     */
                    std::size_t size_approx() const noexcept
                    {
                        return write_.load(std::memory_order_acquire) -
                               read_.load(std::memory_order_acquire);
                    }

                private:
                    static std::size_t round_up_pow2(std::size_t n) noexcept
                    {
                        std::size_t p = 2;
                        while (p < n)
                            p <<= 1;
                        return p;
                    }

                    std::vector<T> buffer_;
                    std::size_t mask_;
                    alignas(64) std::atomic<std::size_t> write_{0};
                    alignas(64) std::atomic<std::size_t> read_{0};
            };
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
