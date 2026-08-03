/**************************************************************************/
/* convolution.hpp                                                        */
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

#ifndef SUSHIENGINE_AUDIO_DSP_CONVOLUTION_HPP
#define SUSHIENGINE_AUDIO_DSP_CONVOLUTION_HPP

/**
 * @file convolution.hpp
 * @brief Uniformly-partitioned overlap-save (UPOLS) fast convolution.
 *
 * The engine behind the convolution reverb (§3.5, §7, §S10 of
 * `docs/slop/audio_system.md`): convolve a running input against an arbitrarily long
 * impulse response in `O(log B)` per sample instead of `O(M)`, at a fixed latency of one
 * block `B`. The IR is cut into `⌈M/B⌉` partitions; each is transformed once to the
 * frequency domain (size `2B`). Every block, the input's `2B` spectrum is pushed into a
 * **frequency-domain delay line** and multiply-accumulated against all partition spectra;
 * one inverse FFT and an overlap-save discard of the first `B` samples yields the output
 * block. This is the uniform (UPOLS) scheme; Gardner's non-uniform partitioning (small
 * head partitions for lower latency, large tail partitions for fewer transforms) is a CPU
 * optimisation layered on the same result.
 *
 * Fixed block size `B` (a power of two) — the caller (@ref ConvolutionReverb) buffers
 * arbitrary device blocks to it. Portable `float` DSP, no SDL and no SushiRuntime.
 */

#include <algorithm>
#include <complex>
#include <cstddef>
#include <vector>

#include <SushiEngine/audio/dsp/fft.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        namespace DSP
        {
            /** @brief A partitioned overlap-save convolver against a fixed impulse response. */
            class PartitionedConvolver
            {
                public:
                    /**
                     * @brief Prepares the convolver for a block size and impulse response.
                     * @param block_size The processing block `B` (a power of two).
                     * @param ir         The impulse-response samples.
                     * @param ir_length  Number of IR samples.
                     */
                    void prepare(int block_size, const float* ir, int ir_length)
                    {
                        block_ = block_size;
                        fft_size_ = block_size * 2;
                        fft_.prepare(fft_size_);

                        partitions_ = ir_length > 0 ? (ir_length + block_ - 1) / block_ : 0;
                        ir_spectra_.assign(static_cast<std::size_t>(partitions_),
                                           std::vector<std::complex<float>>(
                                               static_cast<std::size_t>(fft_size_)));
                        for (int p = 0; p < partitions_; ++p)
                        {
                            std::vector<std::complex<float>>& spectrum =
                                ir_spectra_[static_cast<std::size_t>(p)];
                            for (int i = 0; i < fft_size_; ++i)
                                spectrum[static_cast<std::size_t>(i)] =
                                    std::complex<float>(0.0f, 0.0f);
                            for (int i = 0; i < block_; ++i)
                            {
                                const int source = p * block_ + i;
                                if (source < ir_length)
                                    spectrum[static_cast<std::size_t>(i)] =
                                        std::complex<float>(ir[source], 0.0f);
                            }
                            fft_.forward(spectrum.data());
                        }

                        fdl_.assign(static_cast<std::size_t>(partitions_ > 0 ? partitions_ : 1),
                                    std::vector<std::complex<float>>(
                                        static_cast<std::size_t>(fft_size_),
                                        std::complex<float>(0.0f, 0.0f)));
                        write_ = 0;
                        previous_input_.assign(static_cast<std::size_t>(block_), 0.0f);
                        time_.assign(static_cast<std::size_t>(fft_size_), std::complex<float>(0.0f, 0.0f));
                        accum_.assign(static_cast<std::size_t>(fft_size_), std::complex<float>(0.0f, 0.0f));
                    }

                    /** @brief Clears the delay line and input history (silences the tail). */
                    void reset() noexcept
                    {
                        for (std::vector<std::complex<float>>& s : fdl_)
                            for (std::complex<float>& c : s)
                                c = std::complex<float>(0.0f, 0.0f);
                        for (float& f : previous_input_)
                            f = 0.0f;
                        write_ = 0;
                    }

                    /** @brief The fixed processing block size `B`. */
                    int block_size() const noexcept { return block_; }

                    /** @brief The number of IR partitions (0 = a null/empty IR). */
                    int partitions() const noexcept { return partitions_; }

                    /**
                     * @brief Convolves exactly one block: `B` input samples → `B` output samples.
                     * @param in  The input block (`B` samples).
                     * @param out The output block (`B` samples); may equal @p in.
                     */
                    void process_block(const float* in, float* out) noexcept
                    {
                        if (partitions_ == 0)
                        {
                            for (int i = 0; i < block_; ++i)
                                out[i] = 0.0f;
                            return;
                        }

                        // Build the 2B analysis frame: [previous B][current B], transform it,
                        // and store it as the newest entry in the frequency-domain delay line.
                        for (int i = 0; i < block_; ++i)
                            time_[static_cast<std::size_t>(i)] = std::complex<float>(
                                previous_input_[static_cast<std::size_t>(i)], 0.0f);
                        for (int i = 0; i < block_; ++i)
                            time_[static_cast<std::size_t>(block_ + i)] = std::complex<float>(in[i], 0.0f);
                        fft_.forward(time_.data());
                        fdl_[static_cast<std::size_t>(write_)] = time_;

                        // Multiply-accumulate every partition against the matching delayed input
                        // spectrum (newest partition ↔ newest input).
                        for (int i = 0; i < fft_size_; ++i)
                            accum_[static_cast<std::size_t>(i)] = std::complex<float>(0.0f, 0.0f);
                        for (int p = 0; p < partitions_; ++p)
                        {
                            int slot = write_ - p;
                            while (slot < 0)
                                slot += partitions_;
                            const std::vector<std::complex<float>>& x =
                                fdl_[static_cast<std::size_t>(slot)];
                            const std::vector<std::complex<float>>& h =
                                ir_spectra_[static_cast<std::size_t>(p)];
                            for (int i = 0; i < fft_size_; ++i)
                                accum_[static_cast<std::size_t>(i)] +=
                                    x[static_cast<std::size_t>(i)] * h[static_cast<std::size_t>(i)];
                        }

                        fft_.inverse(accum_.data());
                        // Overlap-save: the valid (non-aliased) output is the second half.
                        for (int i = 0; i < block_; ++i)
                            out[i] = accum_[static_cast<std::size_t>(block_ + i)].real();

                        for (int i = 0; i < block_; ++i)
                            previous_input_[static_cast<std::size_t>(i)] = in[i];
                        if (++write_ >= partitions_)
                            write_ = 0;
                    }

                private:
                    RadixFFT fft_;
                    std::vector<std::vector<std::complex<float>>> ir_spectra_;
                    std::vector<std::vector<std::complex<float>>> fdl_;
                    std::vector<float> previous_input_;
                    std::vector<std::complex<float>> time_;
                    std::vector<std::complex<float>> accum_;
                    int block_ = 0;
                    int fft_size_ = 0;
                    int partitions_ = 0;
                    int write_ = 0;
            };

            /**
             * @brief Non-uniformly partitioned convolution (Gardner): a small-block head plus
             *        a large-block tail.
             *
             * The head impulse (the first @p head_length samples) is convolved with the small
             * processing block @p B for low latency; the long tail is convolved with a larger
             * block `T` so the many tail partitions cost far fewer, larger FFTs. The tail's own
             * `T`-sample buffering supplies exactly the head-length delay its contribution needs,
             * so head + tail line up with no extra latency. Same output as a single UPOLS over
             * the whole IR, at a fraction of the tail cost for multi-second impulses. Processes
             * a fixed block @p B (the caller buffers arbitrary device blocks to it).
             */
            class NonUniformConvolver
            {
                public:
                    /**
                     * @brief Prepares the head/tail split.
                     * @param block       The processing block `B` (a power of two).
                     * @param ir          The impulse response.
                     * @param ir_length   Number of IR samples.
                     * @param head_blocks How many `B`-blocks the low-latency head spans (tail
                     *                     block `T` = head_blocks·B).
                     */
                    void prepare(int block, const float* ir, int ir_length, int head_blocks = 4)
                    {
                        block_ = block;
                        if (head_blocks < 1)
                            head_blocks = 1;
                        split_ = head_blocks * block;
                        if (split_ > ir_length)
                            split_ = ir_length;
                        tail_block_ = split_ > 0 ? split_ : block;

                        head_.prepare(block, ir, split_);
                        has_tail_ = ir_length > split_;
                        if (has_tail_)
                            tail_.prepare(tail_block_, ir + split_, ir_length - split_);

                        head_out_.assign(static_cast<std::size_t>(block), 0.0f);
                        tail_in_.assign(static_cast<std::size_t>(tail_block_), 0.0f);
                        tail_out_.assign(static_cast<std::size_t>(tail_block_), 0.0f);
                        const std::size_t cap = static_cast<std::size_t>(tail_block_ * 4);
                        in_fifo_.assign(cap, 0.0f);
                        out_fifo_.assign(cap, 0.0f);
                        in_r_ = in_w_ = in_count_ = 0;
                        out_r_ = out_w_ = out_count_ = 0;
                    }

                    /** @brief Clears all state. */
                    void reset() noexcept
                    {
                        head_.reset();
                        tail_.reset();
                        std::fill(in_fifo_.begin(), in_fifo_.end(), 0.0f);
                        std::fill(out_fifo_.begin(), out_fifo_.end(), 0.0f);
                        in_r_ = in_w_ = in_count_ = 0;
                        out_r_ = out_w_ = out_count_ = 0;
                    }

                    /** @brief The head/tail split point in samples. */
                    int split() const noexcept { return split_; }

                    /** @brief Convolves one block: `B` in → `B` out (may alias). */
                    void process_block(const float* in, float* out) noexcept
                    {
                        head_.process_block(in, head_out_.data());

                        // Consume tail output produced in PRIOR blocks first, so a large-block
                        // tail run this block is not read back one block too early — this
                        // buffering is exactly the head-length delay the tail contribution needs.
                        for (int i = 0; i < block_; ++i)
                        {
                            const float tail = out_count_ > 0 ? pop(out_fifo_, out_r_, out_count_) : 0.0f;
                            out[i] = head_out_[static_cast<std::size_t>(i)] + tail;
                        }

                        if (has_tail_)
                        {
                            for (int i = 0; i < block_; ++i)
                                push(in_fifo_, in_w_, in_count_, in[i]);
                            while (in_count_ >= tail_block_)
                            {
                                for (int k = 0; k < tail_block_; ++k)
                                    tail_in_[static_cast<std::size_t>(k)] = pop(in_fifo_, in_r_, in_count_);
                                tail_.process_block(tail_in_.data(), tail_out_.data());
                                for (int k = 0; k < tail_block_; ++k)
                                    push(out_fifo_, out_w_, out_count_, tail_out_[static_cast<std::size_t>(k)]);
                            }
                        }
                    }

                private:
                    static void push(std::vector<float>& buffer, int& w, int& count,
                                     float v) noexcept
                    {
                        buffer[static_cast<std::size_t>(w)] = v;
                        if (++w >= static_cast<int>(buffer.size()))
                            w = 0;
                        ++count;
                    }
                    static float pop(std::vector<float>& buffer, int& r, int& count) noexcept
                    {
                        const float v = buffer[static_cast<std::size_t>(r)];
                        if (++r >= static_cast<int>(buffer.size()))
                            r = 0;
                        --count;
                        return v;
                    }

                    PartitionedConvolver head_;
                    PartitionedConvolver tail_;
                    std::vector<float> head_out_;
                    std::vector<float> tail_in_;
                    std::vector<float> tail_out_;
                    std::vector<float> in_fifo_;
                    std::vector<float> out_fifo_;
                    int block_ = 0;
                    int split_ = 0;
                    int tail_block_ = 0;
                    int in_r_ = 0, in_w_ = 0, in_count_ = 0;
                    int out_r_ = 0, out_w_ = 0, out_count_ = 0;
                    bool has_tail_ = false;
            };
        } // namespace DSP
    } // namespace Audio
} // namespace SushiEngine

#endif
