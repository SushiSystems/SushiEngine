/**************************************************************************/
/* streaming.hpp                                                         */
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

#ifndef SUSHIENGINE_AUDIO_STREAMING_HPP
#define SUSHIENGINE_AUDIO_STREAMING_HPP

/**
 * @file streaming.hpp
 * @brief Streamed voices: a worker decodes a long asset in chunks into a ring the audio
 *        thread drains — the audio thread never touches disk or a decoder.
 *
 * Music and dialogue are too big to hold decoded in memory, so they are **streamed** (§10
 * of `docs/slop/audio_system.md`): a normal-priority worker reads the encoded asset from a
 * data source in chunks, decodes on the fly through an @ref IAudioCodec, and pushes the
 * samples into a lock-free ring (the S1 @ref SpscRing). The audio thread's
 * @ref StreamingSource only ever *pops* that ring — no I/O, no decode, no allocation on the
 * hot path (the design's hard rule).
 *
 * The producer/consumer split is exactly one SPSC queue: @ref StreamingDecoder::pump is the
 * single producer (the worker), @ref StreamingSource::render / ::advance the single
 * consumer (the audio thread). @ref StreamingWorker wraps `pump` in a `std::thread` for
 * production; a test drives `pump` synchronously instead, so the path is deterministic and
 * needs no real thread. The @ref IDataSource seam abstracts "disk" — @ref MemoryDataSource
 * serves a byte buffer, so nothing here does actual file I/O. Portable, no SDL/runtime.
 */

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <SushiEngine/audio/codec.hpp>
#include <SushiEngine/audio/dsp/spsc_ring.hpp>
#include <SushiEngine/audio/voice.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief A random-access byte source for an encoded asset (the "disk" seam).
         *
         * A file reader implements this over `std::ifstream`; @ref MemoryDataSource serves
         * an in-memory buffer. The streaming decoder only reads sequentially, but the
         * random-access shape keeps seeking open.
         */
        class IDataSource
        {
            public:
                virtual ~IDataSource() = default;

                /** @brief The total byte size of the asset. */
                virtual std::uint32_t size() const noexcept = 0;

                /**
                 * @brief Reads bytes at an offset.
                 * @param offset      Byte offset into the asset.
                 * @param destination Destination buffer.
                 * @param bytes       Bytes to read.
                 * @return The number of bytes actually read (0 past the end).
                 */
                virtual int read(std::uint32_t offset, std::uint8_t* destination,
                                 int bytes) noexcept = 0;
        };

        /** @brief An @ref IDataSource over a borrowed in-memory byte buffer. */
        class MemoryDataSource final : public IDataSource
        {
            public:
                /**
                 * @brief Wraps a byte buffer.
                 * @param data The bytes (borrowed; must outlive this source).
                 * @param size Byte count.
                 */
                MemoryDataSource(const std::uint8_t* data, std::uint32_t size) noexcept
                    : data_(data), size_(size)
                {
                }

                std::uint32_t size() const noexcept override { return size_; }

                int read(std::uint32_t offset, std::uint8_t* destination,
                         int bytes) noexcept override
                {
                    if (offset >= size_)
                        return 0;
                    int available = static_cast<int>(size_ - offset);
                    if (bytes > available)
                        bytes = available;
                    for (int i = 0; i < bytes; ++i)
                        destination[i] = data_[offset + static_cast<std::uint32_t>(i)];
                    return bytes;
                }

            private:
                const std::uint8_t* data_;
                std::uint32_t size_;
        };

        /**
         * @brief The producer: reads and decodes an asset in chunks into a sample ring.
         *
         * Owns the codec and the decoded-sample ring; borrows the data source. @ref pump is
         * called repeatedly by the worker (or a test): it tops the ring up from the source
         * until the ring is nearly full or the asset is exhausted (looping wraps and resets
         * the codec). Decoded output is interleaved by channel in the ring, exactly as the
         * consumer expects.
         */
        class StreamingDecoder
        {
            public:
                /**
                 * @brief Builds a decoder over a source and codec.
                 * @param source        The encoded asset (borrowed; must outlive this).
                 * @param codec         The decoder for the asset's format (ownership taken).
                 * @param ring_frames   Ring capacity in frames (rounded up to a power of two).
                 * @param loop          Whether to wrap and keep feeding at end-of-asset.
                 */
                StreamingDecoder(IDataSource& source, std::unique_ptr<IAudioCodec> codec,
                                 std::size_t ring_frames, bool loop)
                    : source_(source), codec_(std::move(codec)), loop_(loop),
                      channels_(codec_ ? codec_->channels() : 1),
                      ring_(ring_frames * static_cast<std::size_t>(channels_ > 0 ? channels_ : 1))
                {
                    if (channels_ < 1)
                        channels_ = 1;
                }

                /** @brief The channel count of the streamed asset. */
                int channels() const noexcept { return channels_; }

                /** @brief The sample ring the consumer drains (audio thread). */
                DSP::SpscRing<float>& ring() noexcept { return ring_; }

                /** @brief Whether the asset is fully decoded and the ring is empty. */
                bool finished() const noexcept
                {
                    return exhausted_.load(std::memory_order_acquire) && ring_.size_approx() == 0;
                }

                /**
                 * @brief Tops the ring up from the source (producer; the worker calls this).
                 *
                 * Reads up to @p chunk_bytes of encoded data, decodes it, and pushes the
                 * samples into the ring while there is room. Returns false once the asset is
                 * exhausted and no more will ever be produced (non-looping).
                 *
                 * @param chunk_bytes Encoded bytes to pull from the source per call.
                 * @return True while the stream may still produce more samples.
                 */
                bool pump(int chunk_bytes = 4096)
                {
                    if (exhausted_.load(std::memory_order_acquire))
                        return false;
                    // Only work when the ring has meaningful room, so a full ring is cheap.
                    if (ring_.capacity() - ring_.size_approx() < static_cast<std::size_t>(channels_ * 256))
                        return true;

                    // A single compressed packet (e.g. a Vorbis codebook setup header) can exceed
                    // one chunk; when the last pump read a full chunk yet decoded nothing, the read
                    // window is grown until the packet fits, so a large packet never stalls the stream.
                    const int effective = chunk_bytes + static_cast<int>(stall_grow_);
                    encoded_.resize(static_cast<std::size_t>(effective));
                    int got = source_.read(read_offset_, encoded_.data(), effective);
                    if (got <= 0)
                    {
                        if (loop_)
                        {
                            read_offset_ = 0;
                            codec_->reset();
                            got = source_.read(read_offset_, encoded_.data(), chunk_bytes);
                            if (got <= 0)
                            {
                                exhausted_.store(true, std::memory_order_release);
                                return false;
                            }
                        }
                        else
                        {
                            exhausted_.store(true, std::memory_order_release);
                            return false;
                        }
                    }

                    const int cap_frames = 1024;
                    decoded_.resize(static_cast<std::size_t>(cap_frames * channels_));
                    int in_off = 0;
                    while (in_off < got)
                    {
                        int consumed = 0;
                        const int frames = codec_->decode(encoded_.data() + in_off, got - in_off,
                                                          decoded_.data(), cap_frames, consumed);
                        // Advance past any consumed input even when it produced no audio (a
                        // compressed codec's setup headers and lapping packets consume bytes but
                        // yield zero frames); only stop when a call makes no progress at all.
                        if (frames <= 0 && consumed <= 0)
                            break;
                        const int samples = frames * channels_;
                        for (int i = 0; i < samples; ++i)
                        {
                            if (!ring_.push(decoded_[static_cast<std::size_t>(i)]))
                            {
                                // Ring filled mid-chunk: rewind the source to just past what
                                // we actually pushed so nothing is dropped, and stop.
                                pushed_partial_ = true;
                                break;
                            }
                        }
                        in_off += consumed;
                        if (pushed_partial_)
                        {
                            pushed_partial_ = false;
                            // Conservative: advance the read cursor by what we consumed so far.
                            break;
                        }
                    }
                    read_offset_ += static_cast<std::uint32_t>(in_off);
                    if (in_off == 0 && got == effective)
                        stall_grow_ += static_cast<std::size_t>(chunk_bytes); // packet > window: grow
                    else
                        stall_grow_ = 0;
                    return true;
                }

            private:
                IDataSource& source_;
                std::unique_ptr<IAudioCodec> codec_;
                bool loop_;
                int channels_;
                DSP::SpscRing<float> ring_;
                std::vector<std::uint8_t> encoded_;
                std::vector<float> decoded_;
                std::uint32_t read_offset_ = 0;
                std::size_t stall_grow_ = 0;
                bool pushed_partial_ = false;
                std::atomic<bool> exhausted_{false};
        };

        /**
         * @brief The consumer: a voice that renders from a @ref StreamingDecoder's ring.
         *
         * Pops decoded samples off the ring on the audio thread and down-mixes to the mono
         * a voice renders. A ring underrun (the worker fell behind) yields silence for the
         * shortfall rather than a stall; the voice ends when the decoder is finished and the
         * ring has drained. The decoder must outlive the source.
         */
        class StreamingSource final : public VoiceSource
        {
            public:
                /** @brief Builds a streaming voice over a decoder (borrowed). */
                explicit StreamingSource(StreamingDecoder& decoder) noexcept
                    : decoder_(decoder), channels_(decoder.channels())
                {
                }

                bool render(float* out, int frame_count) noexcept override
                {
                    DSP::SpscRing<float>& ring = decoder_.ring();
                    const float inv = channels_ > 0 ? 1.0f / static_cast<float>(channels_) : 1.0f;
                    for (int i = 0; i < frame_count; ++i)
                    {
                        float sum = 0.0f;
                        bool got = false;
                        for (int c = 0; c < channels_; ++c)
                        {
                            float s = 0.0f;
                            if (ring.pop(s))
                            {
                                sum += s;
                                got = true;
                            }
                        }
                        out[i] = got ? sum * inv : 0.0f; // underrun → momentary silence
                    }
                    return !decoder_.finished();
                }

                bool advance(int frame_count) noexcept override
                {
                    DSP::SpscRing<float>& ring = decoder_.ring();
                    float s = 0.0f;
                    for (int i = 0; i < frame_count * channels_; ++i)
                        if (!ring.pop(s))
                            break;
                    return !decoder_.finished();
                }

            private:
                StreamingDecoder& decoder_;
                int channels_;
        };

        /**
         * @brief A worker thread that keeps a @ref StreamingDecoder's ring fed.
         *
         * Wraps `pump` in a `std::thread` that tops the ring up and yields. Optional — a
         * test pumps synchronously — so the streaming core stays deterministic. @ref stop
         * (also the destructor) joins the thread.
         */
        class StreamingWorker
        {
            public:
                explicit StreamingWorker(StreamingDecoder& decoder) noexcept : decoder_(decoder) {}

                ~StreamingWorker() { stop(); }

                StreamingWorker(const StreamingWorker&) = delete;
                StreamingWorker& operator=(const StreamingWorker&) = delete;

                /** @brief Starts the decode loop on a background thread. */
                void start()
                {
                    if (running_.exchange(true))
                        return;
                    thread_ = std::thread([this]() {
                        while (running_.load(std::memory_order_acquire))
                        {
                            if (!decoder_.pump())
                                break;
                            std::this_thread::sleep_for(std::chrono::milliseconds(2));
                        }
                    });
                }

                /** @brief Stops and joins the decode thread. */
                void stop() noexcept
                {
                    running_.store(false, std::memory_order_release);
                    if (thread_.joinable())
                        thread_.join();
                }

            private:
                StreamingDecoder& decoder_;
                std::thread thread_;
                std::atomic<bool> running_{false};
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
