/**************************************************************************/
/* codec.hpp                                                             */
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

#ifndef SUSHIENGINE_AUDIO_CODEC_HPP
#define SUSHIENGINE_AUDIO_CODEC_HPP

/**
 * @file codec.hpp
 * @brief Audio codecs by content class — the `IAudioCodec` seam plus the from-scratch
 *        PCM and IMA-ADPCM implementations.
 *
 * A bank stores each sound in the codec that suits it (§10, §13 of
 * `docs/slop/audio_system.md`): **PCM** for tiny, critical SFX where size is no object;
 * **IMA-ADPCM** for the dense, repetitive SFX where a 4:1 squeeze is free because the
 * quantisation artefacts hide under the content. Compressed streaming formats
 * (Vorbis/Opus, for music and dialogue) slot in behind the same @ref IAudioCodec seam
 * later — they need a third-party decoder library, a dependency-provisioning decision,
 * so they are intentionally not built here; the seam keeps the bank and voice code
 * oblivious to which decoder runs.
 *
 * The codec is a **stateful sequential decoder**: @ref IAudioCodec::reset then repeated
 * @ref IAudioCodec::decode calls walk an encoded byte stream forward, reporting frames
 * produced and bytes consumed — the shape a streaming worker needs. @ref decode_all is
 * the one-shot convenience for a resident (fully in-memory) sound. Output is always
 * interleaved `float`; multi-channel is interleaved in, interleaved out. Portable, no
 * SDL and no SushiRuntime, so it unit-tests against a hand-encoded buffer.
 *
 * The ADPCM here is a **continuous** nibble stream (no per-block WAV headers) because
 * the bank owns both ends — @ref ImaAdpcmCodec::encode produces exactly what
 * @ref ImaAdpcmCodec decodes, bit-for-bit reproducible.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief The wire format a sound's samples are stored in. */
        enum class AudioCodecKind : std::uint32_t
        {
            PCMFloat = 0, /**< 32-bit float samples, interleaved (largest, exact). */
            PCM16 = 1,    /**< 16-bit signed samples, interleaved (2× float density). */
            ImaAdpcm = 2, /**< 4-bit IMA-ADPCM, continuous nibble stream (≈4× density). */
            Opus = 3,     /**< Length-framed libopus packets (music/dialogue; needs SE_AUDIO_OPUS). */
            Vorbis = 4    /**< Length-framed libvorbis packets (music/dialogue; needs SE_AUDIO_VORBIS). */
        };

        /**
         * @brief The decode seam: a stateful, forward-only sample decoder.
         *
         * Each concrete codec decodes one channel-interleaved stream. Streaming pulls it
         * chunk by chunk; a resident sound uses @ref decode_all. `reset` returns the
         * decoder to the stream start (also used to re-loop or seek-to-zero).
         */
        class IAudioCodec
        {
            public:
                virtual ~IAudioCodec() = default;

                /** @brief The channel count of the stream this codec decodes. */
                virtual int channels() const noexcept = 0;

                /** @brief Rewinds decoder state to the start of the stream. */
                virtual void reset() noexcept = 0;

                /**
                 * @brief Decodes forward from an encoded buffer into interleaved float.
                 * @param in            The encoded byte buffer (a chunk, or the whole stream).
                 * @param in_bytes      Bytes available in @p in.
                 * @param out           Interleaved float output.
                 * @param frame_capacity Frames (not samples) @p out can hold.
                 * @param in_consumed   Set to the number of input bytes consumed.
                 * @return The number of frames produced.
                 */
                virtual int decode(const std::uint8_t* in, int in_bytes, float* out,
                                   int frame_capacity, int& in_consumed) noexcept = 0;
        };

        /** @brief PCM codec: 16-bit or float samples to interleaved float. */
        class PCMCodec final : public IAudioCodec
        {
            public:
                /**
                 * @brief Builds a PCM codec.
                 * @param channels  Interleaved channel count.
                 * @param sixteen_bit True for `PCM16` input, false for `PCMFloat`.
                 */
                PCMCodec(int channels, bool sixteen_bit) noexcept
                    : channels_(channels), sixteen_bit_(sixteen_bit)
                {
                }

                int channels() const noexcept override { return channels_; }
                void reset() noexcept override {}

                int decode(const std::uint8_t* in, int in_bytes, float* out, int frame_capacity,
                           int& in_consumed) noexcept override
                {
                    const int sample_bytes = sixteen_bit_ ? 2 : 4;
                    const int frame_bytes = sample_bytes * channels_;
                    if (frame_bytes <= 0)
                    {
                        in_consumed = 0;
                        return 0;
                    }
                    int frames = in_bytes / frame_bytes;
                    if (frames > frame_capacity)
                        frames = frame_capacity;
                    const int samples = frames * channels_;
                    if (sixteen_bit_)
                    {
                        for (int i = 0; i < samples; ++i)
                        {
                            const std::uint8_t lo = in[i * 2];
                            const std::uint8_t hi = in[i * 2 + 1];
                            const std::int16_t s = static_cast<std::int16_t>(
                                static_cast<std::uint16_t>(lo) | (static_cast<std::uint16_t>(hi) << 8));
                            out[i] = static_cast<float>(s) / 32768.0f;
                        }
                    }
                    else
                    {
                        for (int i = 0; i < samples; ++i)
                        {
                            std::uint32_t bits = static_cast<std::uint32_t>(in[i * 4]) |
                                                 (static_cast<std::uint32_t>(in[i * 4 + 1]) << 8) |
                                                 (static_cast<std::uint32_t>(in[i * 4 + 2]) << 16) |
                                                 (static_cast<std::uint32_t>(in[i * 4 + 3]) << 24);
                            float f;
                            std::memcpy(&f, &bits, sizeof(f));
                            out[i] = f;
                        }
                    }
                    in_consumed = frames * frame_bytes;
                    return frames;
                }

            private:
                int channels_;
                bool sixteen_bit_;
        };

        /**
         * @brief IMA-ADPCM codec: a continuous 4-bit nibble stream ↔ interleaved float.
         *
         * The classic IMA adaptive differential PCM: each 4-bit code is a step up or down
         * a predictor, the step size itself adapting per sample. Two nibbles per byte
         * (low nibble first); for stereo the two channels' nibbles alternate. Predictor
         * and step index persist across @ref decode calls, so a chunked stream decodes
         * seamlessly. @ref encode is the matching quantiser the bank builder uses.
         */
        class ImaAdpcmCodec final : public IAudioCodec
        {
            public:
                explicit ImaAdpcmCodec(int channels) noexcept : channels_(channels)
                {
                    reset();
                }

                int channels() const noexcept override { return channels_; }

                void reset() noexcept override
                {
                    for (int c = 0; c < kMaxChannels; ++c)
                    {
                        predictor_[c] = 0;
                        step_index_[c] = 0;
                    }
                    half_ = false;
                    pending_ = 0;
                }

                int decode(const std::uint8_t* in, int in_bytes, float* out, int frame_capacity,
                           int& in_consumed) noexcept override
                {
                    int ch = channels_ > 0 ? channels_ : 1;
                    if (ch > kMaxChannels)
                        ch = kMaxChannels;

                    int byte_pos = 0; // next fresh byte in `in` (already-consumed bytes precede it)
                    int frames = 0;
                    while (frames < frame_capacity)
                    {
                        // A frame needs `ch` nibbles; count what is reachable this call: a
                        // carried high nibble in `pending_` plus two per remaining byte.
                        const int nibbles_left = (half_ ? 1 : 0) + (in_bytes - byte_pos) * 2;
                        if (nibbles_left < ch)
                            break;
                        for (int c = 0; c < ch; ++c)
                        {
                            std::uint8_t nib;
                            if (half_)
                            {
                                nib = static_cast<std::uint8_t>((pending_ >> 4) & 0x0f);
                                half_ = false;
                            }
                            else
                            {
                                pending_ = in[byte_pos++]; // byte is now fully consumed
                                nib = static_cast<std::uint8_t>(pending_ & 0x0f);
                                half_ = true;
                            }
                            out[frames * ch + c] = decode_nibble(nib, c);
                        }
                        ++frames;
                    }
                    in_consumed = byte_pos;
                    return frames;
                }

                /**
                 * @brief Encodes interleaved float into a continuous IMA-ADPCM nibble stream.
                 * @param in       Interleaved float samples.
                 * @param frames   Number of frames.
                 * @param channels Channel count.
                 * @param out      Appended with the encoded bytes.
                 */
                static void encode(const float* in, int frames, int channels,
                                   std::vector<std::uint8_t>& out)
                {
                    int ch = channels > 0 ? channels : 1;
                    if (ch > kMaxChannels)
                        ch = kMaxChannels;
                    int predictor[kMaxChannels] = {0};
                    int step_index[kMaxChannels] = {0};
                    std::uint8_t byte = 0;
                    bool high = false;
                    for (int f = 0; f < frames; ++f)
                    {
                        for (int c = 0; c < ch; ++c)
                        {
                            const float s = in[f * ch + c];
                            int sample = static_cast<int>(s * 32768.0f);
                            if (sample > 32767) sample = 32767;
                            if (sample < -32768) sample = -32768;
                            const std::uint8_t nib =
                                encode_nibble(sample, predictor[c], step_index[c]);
                            if (!high)
                                byte = static_cast<std::uint8_t>(nib & 0x0f);
                            else
                            {
                                byte = static_cast<std::uint8_t>(byte | (nib << 4));
                                out.push_back(byte);
                            }
                            high = !high;
                        }
                    }
                    if (high)
                        out.push_back(byte); // flush the trailing low nibble
                }

            private:
                static constexpr int kMaxChannels = 2;

                static const int* step_table() noexcept
                {
                    static const int table[89] = {
                        7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37,
                        41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173,
                        190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658,
                        724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
                        2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484,
                        7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500,
                        20350, 22385, 24623, 27086, 29794, 32767};
                    return table;
                }

                static const int* index_table() noexcept
                {
                    static const int table[16] = {-1, -1, -1, -1, 2, 4, 6, 8,
                                                  -1, -1, -1, -1, 2, 4, 6, 8};
                    return table;
                }

                float decode_nibble(std::uint8_t nib, int c) noexcept
                {
                    const int step = step_table()[step_index_[c]];
                    int diff = step >> 3;
                    if (nib & 4) diff += step;
                    if (nib & 2) diff += step >> 1;
                    if (nib & 1) diff += step >> 2;
                    if (nib & 8) predictor_[c] -= diff; else predictor_[c] += diff;
                    if (predictor_[c] > 32767) predictor_[c] = 32767;
                    if (predictor_[c] < -32768) predictor_[c] = -32768;
                    step_index_[c] += index_table()[nib];
                    if (step_index_[c] < 0) step_index_[c] = 0;
                    if (step_index_[c] > 88) step_index_[c] = 88;
                    return static_cast<float>(predictor_[c]) / 32768.0f;
                }

                static std::uint8_t encode_nibble(int sample, int& predictor, int& step_index) noexcept
                {
                    const int step = step_table()[step_index];
                    int diff = sample - predictor;
                    std::uint8_t nib = 0;
                    if (diff < 0)
                    {
                        nib = 8;
                        diff = -diff;
                    }
                    int temp = step;
                    if (diff >= temp) { nib |= 4; diff -= temp; }
                    temp >>= 1;
                    if (diff >= temp) { nib |= 2; diff -= temp; }
                    temp >>= 1;
                    if (diff >= temp) { nib |= 1; }
                    // Mirror the decoder to keep predictor in lock-step.
                    int rebuilt = step >> 3;
                    if (nib & 4) rebuilt += step;
                    if (nib & 2) rebuilt += step >> 1;
                    if (nib & 1) rebuilt += step >> 2;
                    if (nib & 8) predictor -= rebuilt; else predictor += rebuilt;
                    if (predictor > 32767) predictor = 32767;
                    if (predictor < -32768) predictor = -32768;
                    step_index += index_table()[nib];
                    if (step_index < 0) step_index = 0;
                    if (step_index > 88) step_index = 88;
                    return nib;
                }

                int channels_;
                int predictor_[kMaxChannels] = {0, 0};
                int step_index_[kMaxChannels] = {0, 0};
                bool half_ = false; /**< A carried high nibble of `pending_` awaits the next frame. */
                std::uint8_t pending_ = 0;
        };

        /**
         * @brief Decodes a whole encoded buffer to interleaved float in one shot.
         * @param codec    The (freshly reset) codec to run.
         * @param in       The encoded byte buffer.
         * @param in_bytes Bytes in @p in.
         * @param out      Filled with interleaved float (cleared first).
         */
        inline void decode_all(IAudioCodec& codec, const std::uint8_t* in, int in_bytes,
                               std::vector<float>& out)
        {
            out.clear();
            codec.reset();
            const int chunk_frames = 1024;
            std::vector<float> scratch(static_cast<std::size_t>(chunk_frames * codec.channels()));
            int offset = 0;
            while (offset < in_bytes)
            {
                int consumed = 0;
                const int frames = codec.decode(in + offset, in_bytes - offset, scratch.data(),
                                                chunk_frames, consumed);
                if (frames <= 0 || consumed <= 0)
                    break;
                const int samples = frames * codec.channels();
                out.insert(out.end(), scratch.begin(), scratch.begin() + samples);
                offset += consumed;
            }
        }
    } // namespace Audio
} // namespace SushiEngine

#endif
