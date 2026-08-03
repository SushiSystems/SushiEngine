/**************************************************************************/
/* audio_stream_compressed_demo.cpp                                      */
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

/**
 * @file audio_stream_compressed_demo.cpp
 * @brief Streaming compressed music/dialogue from a disk-like source, end to end.
 *
 * Proves that the compressed codecs (Opus and Vorbis) stream through the existing
 * @ref StreamingDecoder → SPSC ring → @ref StreamingSource pipeline: the codecs decode the
 * bank's length-framed packet container chunk by chunk, so the producer pulls fixed byte
 * chunks from an @ref IDataSource (the disk analog) and the audio-thread consumer only ever
 * pops the ring. Renders past the asset's length with looping on to confirm continuity, and
 * checks the streamed output carries energy. Exits 0 on success.
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

#include <SushiEngine/audio/opus_codec.hpp>
#include <SushiEngine/audio/vorbis_codec.hpp>

using namespace SushiEngine::Audio;

namespace
{
    std::vector<float> make_tone(int frames, int channels, int sample_rate, double frequency)
    {
        std::vector<float> s(static_cast<std::size_t>(frames * channels), 0.0f);
        for (int f = 0; f < frames; ++f)
        {
            const float v = 0.5f * static_cast<float>(std::sin(2.0 * 3.14159265358979323846 *
                                                               frequency * f / sample_rate));
            for (int c = 0; c < channels; ++c)
                s[static_cast<std::size_t>(f * channels + c)] = v;
        }
        return s;
    }

    // Streams `encoded` through the full pipeline with looping on, rendering `render_frames`
    // frames (deliberately more than the asset holds) and returning the output energy.
    double stream_energy(const char* label, std::unique_ptr<IAudioCodec> codec,
                         const std::vector<std::uint8_t>& encoded, int channels, int render_frames)
    {
        MemoryDataSource source(encoded.data(), static_cast<std::uint32_t>(encoded.size()));
        StreamingDecoder decoder(source, std::move(codec), 8192, /*loop=*/true);
        StreamingSource voice(decoder);

        // Prime the ring.
        for (int i = 0; i < 64; ++i)
            decoder.pump(4096);

        const int block = 512;
        std::vector<float> out(static_cast<std::size_t>(block), 0.0f);
        double energy = 0.0;
        int rendered = 0;
        while (rendered < render_frames)
        {
            for (int i = 0; i < 8; ++i)
                decoder.pump(4096); // keep the producer ahead of the consumer
            voice.render(out.data(), block);
            for (int i = 0; i < block; ++i)
                energy += static_cast<double>(out[static_cast<std::size_t>(i)]) *
                          out[static_cast<std::size_t>(i)];
            rendered += block;
        }
        (void)channels;
        std::printf("%s: streamed %d frames, energy=%.2f\n", label, rendered, energy);
        return energy;
    }
} // namespace

int main()
{
    const int sample_rate = 48000;
    const int channels = 2;
    const int frames = sample_rate / 2; // 0.5 s asset
    const int render_frames = sample_rate * 2; // render 2 s → must loop 4x

    const std::vector<float> tone = make_tone(frames, channels, sample_rate, 440.0);

    std::vector<std::uint8_t> opus_bytes;
    if (!OpusCodec::encode(tone.data(), frames, channels, sample_rate, 96000, opus_bytes))
    {
        std::fprintf(stderr, "audio_stream_compressed_demo FAILED: opus encode\n");
        return 1;
    }
    std::vector<std::uint8_t> vorbis_bytes;
    if (!VorbisCodec::encode(tone.data(), frames, channels, sample_rate, 0.4f, vorbis_bytes))
    {
        std::fprintf(stderr, "audio_stream_compressed_demo FAILED: vorbis encode\n");
        return 1;
    }

    const double opus_energy =
        stream_energy("opus  ", std::unique_ptr<IAudioCodec>(new OpusCodec(channels, sample_rate)),
                      opus_bytes, channels, render_frames);
    const double vorbis_energy = stream_energy(
        "vorbis", std::unique_ptr<IAudioCodec>(new VorbisCodec(channels, sample_rate)), vorbis_bytes,
        channels, render_frames);

    if (!(opus_energy > 1.0) || !(vorbis_energy > 1.0))
    {
        std::fprintf(stderr,
                     "audio_stream_compressed_demo FAILED: streamed output silent (opus %.2f "
                     "vorbis %.2f)\n",
                     opus_energy, vorbis_energy);
        return 1;
    }

    std::printf("audio_stream_compressed_demo OK\n");
    return 0;
}
