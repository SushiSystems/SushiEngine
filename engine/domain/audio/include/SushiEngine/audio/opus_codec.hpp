/**************************************************************************/
/* opus_codec.hpp                                                        */
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

#ifndef SUSHIENGINE_AUDIO_OPUS_CODEC_HPP
#define SUSHIENGINE_AUDIO_OPUS_CODEC_HPP

/**
 * @file opus_codec.hpp
 * @brief The Opus codec (`IAudioCodec`) — compressed music/dialogue for the bank.
 *
 * The high-quality compressed content class of §10 / §13, behind the same
 * @ref IAudioCodec seam as PCM/ADPCM. It uses **core libopus** (`opus_encode_float` /
 * `opus_decode_float`) over the bank's own **length-framed packet** container — a
 * `[uint16 length][packet]` stream — rather than Ogg-Opus, because the bank owns both
 * ends (like the ADPCM path), which keeps the decoder chunk-incremental and Ogg-free.
 *
 * This header pulls in libopus, so — like `accelerator_sycl.hpp` — it is **not** on the
 * `audio/audio.hpp` umbrella and is compiled only where `Opus::opus` is linked
 * (guarded by the `SE_AUDIO_OPUS` build option). @ref ImaAdpcmCodec-style: @ref encode
 * produces exactly what @ref decode consumes.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opus.h>
#include <opusfile.h>

#include <SushiEngine/audio/audio.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief An @ref IAudioCodec over core libopus with a length-framed packet stream. */
        class OpusCodec final : public IAudioCodec
        {
            public:
                /**
                 * @brief Builds an Opus decoder.
                 * @param channels    Channel count (1 or 2).
                 * @param sample_rate Decode sample rate (Opus supports 8/12/16/24/48 kHz).
                 */
                OpusCodec(int channels, int sample_rate) noexcept
                    : channels_(channels < 1 ? 1 : channels), sample_rate_(sample_rate)
                {
                    temp_.assign(static_cast<std::size_t>(kMaxFrame * channels_), 0.0f);
                }

                ~OpusCodec() override
                {
                    if (decoder_ != nullptr)
                        opus_decoder_destroy(decoder_);
                }

                OpusCodec(const OpusCodec&) = delete;
                OpusCodec& operator=(const OpusCodec&) = delete;

                int channels() const noexcept override { return channels_; }

                void reset() noexcept override
                {
                    if (decoder_ != nullptr)
                        opus_decoder_ctl(decoder_, OPUS_RESET_STATE);
                }

                int decode(const std::uint8_t* in, int in_bytes, float* out, int frame_capacity,
                           int& in_consumed) noexcept override
                {
                    if (decoder_ == nullptr)
                    {
                        int err = 0;
                        decoder_ = opus_decoder_create(sample_rate_, channels_, &err);
                        if (err != OPUS_OK || decoder_ == nullptr)
                        {
                            in_consumed = in_bytes;
                            return 0;
                        }
                    }

                    int total = 0;
                    int off = 0;
                    while (true)
                    {
                        if (off + 2 > in_bytes)
                            break; // no room for a length header
                        const int length = static_cast<int>(in[off]) |
                                        (static_cast<int>(in[off + 1]) << 8);
                        if (off + 2 + length > in_bytes)
                            break; // partial packet: leave it for the next call
                        const int got = opus_decode_float(decoder_, in + off + 2, length,
                                                          temp_.data(), kMaxFrame, 0);
                        if (got < 0)
                        {
                            off += 2 + length; // corrupt packet: skip it
                            continue;
                        }
                        if (total + got > frame_capacity)
                            break; // output full: leave this packet for the next call
                        const int samples = got * channels_;
                        for (int i = 0; i < samples; ++i)
                            out[total * channels_ + i] = temp_[static_cast<std::size_t>(i)];
                        total += got;
                        off += 2 + length;
                    }
                    in_consumed = off;
                    return total;
                }

                /**
                 * @brief Encodes interleaved float to the length-framed Opus packet stream.
                 * @param in          Interleaved float samples.
                 * @param frames      Number of frames.
                 * @param channels    Channel count.
                 * @param sample_rate Sample rate (8/12/16/24/48 kHz).
                 * @param bitrate     Target bitrate in bits/s (e.g. 96000 music, 32000 voice).
                 * @param out         Appended with the encoded packet stream.
                 * @return True on success.
                 */
                static bool encode(const float* in, int frames, int channels, int sample_rate,
                                   int bitrate, std::vector<std::uint8_t>& out)
                {
                    int err = 0;
                    OpusEncoder* enc =
                        opus_encoder_create(sample_rate, channels, OPUS_APPLICATION_AUDIO, &err);
                    if (err != OPUS_OK || enc == nullptr)
                        return false;
                    opus_encoder_ctl(enc, OPUS_SET_BITRATE(bitrate));

                    const int frame = sample_rate / 50; // 20 ms
                    std::vector<float> chunk(static_cast<std::size_t>(frame * channels), 0.0f);
                    std::vector<unsigned char> packet(4000);
                    for (int f = 0; f < frames; f += frame)
                    {
                        const int n = (f + frame <= frames) ? frame : (frames - f);
                        for (int i = 0; i < n * channels; ++i)
                            chunk[static_cast<std::size_t>(i)] = in[f * channels + i];
                        for (int i = n * channels; i < frame * channels; ++i)
                            chunk[static_cast<std::size_t>(i)] = 0.0f; // zero-pad the last frame
                        const int length =
                            opus_encode_float(enc, chunk.data(), frame, packet.data(),
                                              static_cast<int>(packet.size()));
                        if (length < 0)
                        {
                            opus_encoder_destroy(enc);
                            return false;
                        }
                        out.push_back(static_cast<std::uint8_t>(length & 0xff));
                        out.push_back(static_cast<std::uint8_t>((length >> 8) & 0xff));
                        out.insert(out.end(), packet.begin(), packet.begin() + length);
                    }
                    opus_encoder_destroy(enc);
                    return true;
                }

            private:
                static constexpr int kMaxFrame = 5760; // 120 ms at 48 kHz (largest Opus frame)

                OpusDecoder* decoder_ = nullptr;
                std::vector<float> temp_;
                int channels_;
                int sample_rate_;
        };

        /**
         * @brief Registers @ref OpusCodec as the bank's factory for @ref AudioCodecKind::Opus.
         *
         * Call once at startup from a TU that links libopus; afterwards the header-only bank
         * decodes Opus media transparently through @ref make_codec. @ref set_external_codec_factory.
         */
        /**
         * @brief Decodes a complete in-memory Ogg-Opus (`.opus`) blob to interleaved float.
         *
         * The standard-container interop path: whereas @ref OpusCodec streams the bank's own
         * length-framed packets, this ingests an externally authored Ogg-Opus file (via opusfile's
         * `op_open_memory` / `op_read_float`) so a bank baker can pull in `.opus` assets. Opus always
         * decodes at 48 kHz.
         *
         * @param data        The Ogg-Opus file bytes.
         * @param bytes       Byte count.
         * @param out         Cleared and filled with interleaved float samples.
         * @param channels    Set to the file's channel count.
         * @param sample_rate Set to 48000 (Opus's decode rate).
         * @return True on success.
         */
        inline bool decode_ogg_opus(const std::uint8_t* data, std::size_t bytes,
                                    std::vector<float>& out, int& channels, int& sample_rate)
        {
            int err = 0;
            OggOpusFile* of =
                op_open_memory(reinterpret_cast<const unsigned char*>(data), bytes, &err);
            if (of == nullptr || err != 0)
            {
                if (of != nullptr)
                    op_free(of);
                return false;
            }
            channels = op_channel_count(of, -1);
            if (channels < 1)
                channels = 1;
            sample_rate = 48000;
            out.clear();
            std::vector<float> buffer(static_cast<std::size_t>(5760 * channels), 0.0f);
            for (;;)
            {
                const int got =
                    op_read_float(of, buffer.data(), static_cast<int>(buffer.size()), nullptr);
                if (got <= 0)
                    break;
                out.insert(out.end(), buffer.begin(),
                           buffer.begin() + static_cast<std::size_t>(got * channels));
            }
            op_free(of);
            return true;
        }

        inline void register_opus_codec() noexcept
        {
            add_external_codec_factory(
                [](AudioCodecKind kind, int channels, int sample_rate) -> std::unique_ptr<IAudioCodec>
                {
                    if (kind == AudioCodecKind::Opus)
                        return std::unique_ptr<IAudioCodec>(new OpusCodec(channels, sample_rate));
                    return nullptr;
                });
        }
    } // namespace Audio
} // namespace SushiEngine

#endif
