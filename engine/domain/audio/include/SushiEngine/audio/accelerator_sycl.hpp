/**************************************************************************/
/* accelerator_sycl.hpp                                                  */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#ifndef SUSHIENGINE_AUDIO_ACCELERATOR_SYCL_HPP
#define SUSHIENGINE_AUDIO_ACCELERATOR_SYCL_HPP

/**
 * @file accelerator_sycl.hpp
 * @brief The SushiRuntime-backed GPU batch-DSP accelerator (§12.2, §S10).
 *
 * The real implementation of the @ref SushiEngine::Audio::IDspAccelerator seam declared at
 * S0. The real-time mix stays on the CPU; this offloads **batch, latency-tolerant** DSP —
 * here long FIR convolution — onto the SushiRuntime SYCL device with **k-block lookahead**,
 * so the audio thread submits work for a future block and collects an already-finished
 * result K blocks later, never stalling on the device.
 *
 * The offload is a per-block, *stateless* convolution: the caller hands a block padded with
 * the preceding `ir_len − 1` history samples, and a SYCL `parallel_for` computes the block's
 * outputs in parallel. Each submit is independent, so slots pipeline freely — the device
 * chews block *t* while the host prepares block *t+1* and consumes block *t−K*. Buffers are
 * USM allocated through the runtime's context (`malloc_shared`), host- and device-visible,
 * so there is no explicit copy.
 *
 * This header uses `sycl` and SushiRuntime, so — unlike the rest of `include/SushiEngine/audio/`
 * — it must be included only from a **SYCL translation unit** (an `add_sushi_sycl_executable`
 * target). It is deliberately kept off the `audio/audio.hpp` umbrella for that reason. The
 * runtime's fluent API is unstable, so the coupling is confined to this one file
 * (`unsafe_context()` → queue + USM); a failure to acquire a device leaves @ref available
 * false and every caller on its CPU path.
 */

#include <cstddef>
#include <vector>

#include <sycl/sycl.hpp>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/audio/accelerator.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief An @ref IDspAccelerator that runs batch FIR convolution on the SYCL device.
         *
         * Construct with the app's runtime and the block/IR sizes; @ref set_impulse uploads
         * the filter; then pipeline with @ref submit (async, non-blocking) and @ref collect
         * (waits on that slot). @ref lookahead slots pipeline device work behind the host.
         */
        class SyclDspAccelerator final : public IDspAccelerator
        {
            public:
                /**
                 * @brief Builds the accelerator and its USM slot ring.
                 * @param runtime   The app runtime (borrowed; provides the device queue + USM).
                 * @param block     The block size (samples per submit).
                 * @param max_ir    The longest impulse response supported.
                 * @param lookahead How many blocks may be in flight (the pipeline depth).
                 */
                SyclDspAccelerator(SushiRuntime::API::Runtime& runtime, int block, int max_ir,
                                   int lookahead)
                    : block_(block), max_ir_(max_ir), slots_(lookahead + 1)
                {
                    try
                    {
                        context_ = &runtime.unsafe_context();
                        padded_ = block_ + max_ir_ - 1;
                        ir_dev_ = context_->malloc_shared<float>(static_cast<std::size_t>(max_ir_));
                        for (int i = 0; i < max_ir_; ++i)
                            ir_dev_[i] = 0.0f;
                        slot_.resize(static_cast<std::size_t>(slots_));
                        for (Slot& s : slot_)
                        {
                            s.in = context_->malloc_shared<float>(static_cast<std::size_t>(padded_));
                            s.out = context_->malloc_shared<float>(static_cast<std::size_t>(block_));
                            for (int i = 0; i < padded_; ++i)
                                s.in[i] = 0.0f;
                            for (int i = 0; i < block_; ++i)
                                s.out[i] = 0.0f;
                        }
                        available_ = ir_dev_ != nullptr;
                    }
                    catch (...)
                    {
                        available_ = false;
                    }
                }

                ~SyclDspAccelerator() override
                {
                    if (context_ == nullptr)
                        return;
                    try
                    {
                        for (Slot& s : slot_)
                        {
                            if (s.in) context_->free_usm(s.in);
                            if (s.out) context_->free_usm(s.out);
                        }
                        if (ir_dev_) context_->free_usm(ir_dev_);
                    }
                    catch (...)
                    {
                    }
                }

                SyclDspAccelerator(const SyclDspAccelerator&) = delete;
                SyclDspAccelerator& operator=(const SyclDspAccelerator&) = delete;

                bool available() const noexcept override { return available_; }

                /** @brief The block size (samples per @ref submit). */
                int block_size() const noexcept { return block_; }

                /** @brief The pipeline depth (blocks that may be in flight). */
                int lookahead() const noexcept { return slots_ - 1; }

                /** @brief The number of history samples a padded input block must carry. */
                int history() const noexcept { return ir_len_ > 0 ? ir_len_ - 1 : 0; }

                /**
                 * @brief Uploads the impulse response to the device (off the audio thread).
                 * @param ir     The filter taps.
                 * @param ir_len Number of taps (clamped to the constructed maximum).
                 */
                void set_impulse(const float* ir, int ir_len)
                {
                    if (!available_)
                        return;
                    ir_len_ = ir_len < max_ir_ ? ir_len : max_ir_;
                    for (int i = 0; i < ir_len_; ++i)
                        ir_dev_[i] = ir[i];
                }

                /**
                 * @brief Submits one padded block for convolution (async, non-blocking).
                 *
                 * @p padded_input holds `history() + block_size()` samples: the previous
                 * `history()` input samples followed by this block's `block_size()` samples.
                 * The kernel is enqueued and this returns immediately; the result is read
                 * later with @ref collect on the same slot.
                 *
                 * @param slot         The pipeline slot (0 .. lookahead()).
                 * @param padded_input The history-prefixed input (`history()+block_size()` samples).
                 */
                void submit(int slot, const float* padded_input)
                {
                    if (!available_)
                        return;
                    Slot& s = slot_[static_cast<std::size_t>(slot % slots_)];
                    const int pad = history() + block_;
                    for (int i = 0; i < pad; ++i)
                        s.in[i] = padded_input[i];

                    const int block = block_;
                    const int taps = ir_len_;
                    const int hist = history();
                    float* in = s.in;
                    float* out = s.out;
                    const float* ir = ir_dev_;
                    sycl::queue& queue = context_->get_queue();
                    s.event = queue.submit([&](sycl::handler& handler) {
                        handler.parallel_for(sycl::range<1>(static_cast<std::size_t>(block)),
                                             [=](sycl::id<1> idx) {
                                                 const int n = static_cast<int>(idx[0]);
                                                 float acc = 0.0f;
                                                 for (int k = 0; k < taps; ++k)
                                                     acc += in[n + hist - k] * ir[k];
                                                 out[n] = acc;
                                             });
                    });
                    s.pending = true;
                }

                /**
                 * @brief Waits for a slot's result and copies it out (blocks only on that slot).
                 * @param slot   The pipeline slot submitted earlier.
                 * @param output The block-sized output buffer to fill.
                 */
                void collect(int slot, float* output)
                {
                    Slot& s = slot_[static_cast<std::size_t>(slot % slots_)];
                    if (!available_ || !s.pending)
                    {
                        for (int i = 0; i < block_; ++i)
                            output[i] = 0.0f;
                        return;
                    }
                    s.event.wait();
                    for (int i = 0; i < block_; ++i)
                        output[i] = s.out[i];
                    s.pending = false;
                }

            private:
                struct Slot
                {
                    float* in = nullptr;
                    float* out = nullptr;
                    sycl::event event;
                    bool pending = false;
                };

                SushiRuntime::Execution::RuntimeContext* context_ = nullptr;
                float* ir_dev_ = nullptr;
                std::vector<Slot> slot_;
                int block_ = 0;
                int max_ir_ = 0;
                int padded_ = 0;
                int ir_len_ = 0;
                int slots_ = 1;
                bool available_ = false;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
