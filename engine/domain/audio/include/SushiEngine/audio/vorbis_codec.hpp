/**************************************************************************/
/* vorbis_codec.hpp                                                      */
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

#ifndef SUSHIENGINE_AUDIO_VORBIS_CODEC_HPP
#define SUSHIENGINE_AUDIO_VORBIS_CODEC_HPP

/**
 * @file vorbis_codec.hpp
 * @brief The Vorbis codec (`IAudioCodec`) — a second compressed music/dialogue class.
 *
 * The Vorbis counterpart of `opus_codec.hpp`, behind the same @ref IAudioCodec seam and the
 * same bank-owned **length-framed packet** container `[u32 length][packet]…`. It uses the
 * low-level **libvorbis** API (no Ogg framing): @ref encode emits Vorbis's three setup
 * packets (identification, comment, codebooks) followed by the audio packets, and @ref decode
 * feeds the three headers to `vorbis_synthesis_headerin`, then each audio packet to
 * `vorbis_synthesis` — draining `vorbis_synthesis_pcmout` incrementally so a packet that
 * yields more frames than the output holds carries over to the next call, exactly like the
 * chunked model the bank's `decode_all` and the streaming decoder expect.
 *
 * Pulls in libvorbis, so — like `opus_codec.hpp` — it rides no umbrella and compiles only
 * where `Vorbis::vorbis`/`Vorbis::vorbisenc` link (guarded by `SE_AUDIO_VORBIS`).
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <vorbis/codec.h>
#include <vorbis/vorbisenc.h>

#include <SushiEngine/audio/audio.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief An @ref IAudioCodec over low-level libvorbis with a length-framed packet stream. */
        class VorbisCodec final : public IAudioCodec
        {
            public:
                /**
                 * @brief Builds a Vorbis decoder.
                 * @param channels    Channel count (1 or 2).
                 * @param sample_rate Decode sample rate (informational; Vorbis carries its own).
                 */
                VorbisCodec(int channels, int sample_rate) noexcept
                    : channels_(channels < 1 ? 1 : channels)
                {
                    (void)sample_rate; // Vorbis carries its own rate in the setup header
                    vorbis_info_init(&info_);
                    vorbis_comment_init(&comment_);
                }

                ~VorbisCodec() override
                {
                    if (synthesis_inited_)
                    {
                        vorbis_block_clear(&block_);
                        vorbis_dsp_clear(&dsp_);
                    }
                    vorbis_comment_clear(&comment_);
                    vorbis_info_clear(&info_);
                }

                VorbisCodec(const VorbisCodec&) = delete;
                VorbisCodec& operator=(const VorbisCodec&) = delete;

                int channels() const noexcept override { return channels_; }

                void reset() noexcept override
                {
                    // A forward decoder cannot rewind the Vorbis synthesis state without the
                    // headers again; callers that loop re-create the codec, so this is a no-op.
                }

                int decode(const std::uint8_t* in, int in_bytes, float* out, int frame_capacity,
                           int& in_consumed) noexcept override
                {
                    int total = drain_pcm(out, frame_capacity, 0);
                    int off = 0;
                    while (total < frame_capacity)
                    {
                        if (off + 4 > in_bytes)
                            break;
                        std::uint32_t length = 0;
                        std::memcpy(&length, in + off, 4);
                        if (off + 4 + static_cast<int>(length) > in_bytes)
                            break;

                        ogg_packet packet;
                        std::memset(&packet, 0, sizeof(packet));
                        packet.packet = const_cast<unsigned char*>(in + off + 4);
                        packet.bytes = static_cast<long>(length);
                        packet.b_o_s = (headers_seen_ == 0) ? 1 : 0;

                        if (headers_seen_ < 3)
                        {
                            if (vorbis_synthesis_headerin(&info_, &comment_, &packet) != 0)
                            {
                                // Corrupt header stream: give up on this input.
                                off += 4 + static_cast<int>(length);
                                in_consumed = off;
                                return total;
                            }
                            ++headers_seen_;
                            off += 4 + static_cast<int>(length);
                            if (headers_seen_ == 3)
                            {
                                vorbis_synthesis_init(&dsp_, &info_);
                                vorbis_block_init(&dsp_, &block_);
                                synthesis_inited_ = true;
                                if (info_.channels > 0)
                                    channels_ = info_.channels;
                            }
                            continue;
                        }

                        if (vorbis_synthesis(&block_, &packet) == 0)
                            vorbis_synthesis_blockin(&dsp_, &block_);
                        off += 4 + static_cast<int>(length);
                        total = drain_pcm(out, frame_capacity, total);
                    }
                    in_consumed = off;
                    return total;
                }

                /**
                 * @brief Encodes interleaved float to the length-framed Vorbis packet stream.
                 * @param in          Interleaved float samples.
                 * @param frames      Number of frames.
                 * @param channels    Channel count.
                 * @param sample_rate Sample rate.
                 * @param quality     VBR quality in [-0.1, 1.0] (0.4 ≈ ~128 kb/s stereo).
                 * @param out         Appended with the encoded packet stream.
                 * @return True on success.
                 */
                static bool encode(const float* in, int frames, int channels, int sample_rate,
                                   float quality, std::vector<std::uint8_t>& out)
                {
                    vorbis_info info;
                    vorbis_info_init(&info);
                    if (vorbis_encode_init_vbr(&info, channels, sample_rate, quality) != 0)
                    {
                        vorbis_info_clear(&info);
                        return false;
                    }

                    vorbis_comment comment;
                    vorbis_comment_init(&comment);
                    vorbis_dsp_state dsp;
                    vorbis_analysis_init(&dsp, &info);
                    vorbis_block block;
                    vorbis_block_init(&dsp, &block);

                    ogg_packet h_id, h_comment, h_setup;
                    vorbis_analysis_headerout(&dsp, &comment, &h_id, &h_comment, &h_setup);
                    append_packet(out, h_id);
                    append_packet(out, h_comment);
                    append_packet(out, h_setup);

                    const int block_frames = 1024;
                    int written = 0;
                    while (written <= frames)
                    {
                        const int n = (written < frames)
                                          ? ((frames - written < block_frames) ? frames - written
                                                                               : block_frames)
                                          : 0;
                        if (n > 0)
                        {
                            float** buffer = vorbis_analysis_buffer(&dsp, n);
                            for (int c = 0; c < channels; ++c)
                                for (int i = 0; i < n; ++i)
                                    buffer[c][i] = in[(written + i) * channels + c];
                            vorbis_analysis_wrote(&dsp, n);
                        }
                        else
                        {
                            vorbis_analysis_wrote(&dsp, 0); // signal end of stream
                        }
                        written += (n > 0) ? n : (frames - written + 1);

                        while (vorbis_analysis_blockout(&dsp, &block) == 1)
                        {
                            vorbis_analysis(&block, nullptr);
                            vorbis_bitrate_addblock(&block);
                            ogg_packet packet;
                            while (vorbis_bitrate_flushpacket(&dsp, &packet) == 1)
                                append_packet(out, packet);
                        }
                        if (n == 0)
                            break;
                    }

                    vorbis_block_clear(&block);
                    vorbis_dsp_clear(&dsp);
                    vorbis_comment_clear(&comment);
                    vorbis_info_clear(&info);
                    return true;
                }

            private:
                static void append_packet(std::vector<std::uint8_t>& out, const ogg_packet& packet)
                {
                    const std::uint32_t length = static_cast<std::uint32_t>(packet.bytes);
                    const std::size_t base = out.size();
                    out.resize(base + 4 + length);
                    std::memcpy(out.data() + base, &length, 4);
                    std::memcpy(out.data() + base + 4, packet.packet, length);
                }

                // Drains available synthesised PCM into out (interleaved), up to the capacity.
                int drain_pcm(float* out, int frame_capacity, int total) noexcept
                {
                    if (!synthesis_inited_)
                        return total;
                    float** pcm = nullptr;
                    int available = 0;
                    while (total < frame_capacity &&
                           (available = vorbis_synthesis_pcmout(&dsp_, &pcm)) > 0)
                    {
                        const int take =
                            (available < frame_capacity - total) ? available : frame_capacity - total;
                        for (int i = 0; i < take; ++i)
                            for (int c = 0; c < channels_; ++c)
                                out[(total + i) * channels_ + c] =
                                    pcm[c < info_.channels ? c : 0][i];
                        vorbis_synthesis_read(&dsp_, take);
                        total += take;
                    }
                    return total;
                }

                vorbis_info info_;
                vorbis_comment comment_;
                vorbis_dsp_state dsp_;
                vorbis_block block_;
                int channels_;
                int headers_seen_ = 0;
                bool synthesis_inited_ = false;
        };

        /**
         * @brief Registers @ref VorbisCodec as the bank's factory for @ref AudioCodecKind::Vorbis.
         *
         * Call once at startup from a TU that links libvorbis. Composes with @ref register_opus_codec
         * — the shared factory dispatches on kind, so both can be active at once.
         */
        inline void register_vorbis_codec() noexcept
        {
            add_external_codec_factory(
                [](AudioCodecKind kind, int channels, int sample_rate) -> std::unique_ptr<IAudioCodec>
                {
                    if (kind == AudioCodecKind::Vorbis)
                        return std::unique_ptr<IAudioCodec>(new VorbisCodec(channels, sample_rate));
                    return nullptr;
                });
        }
    } // namespace Audio
} // namespace SushiEngine

#endif
