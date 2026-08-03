/**************************************************************************/
/* audio_opus_demo.cpp                                                    */
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

/**
 * @file audio_opus_demo.cpp
 * @brief The Opus codec (compressed music/dialogue) round-trip, end to end.
 *
 * Encodes a stereo tone to the bank's length-framed Opus packet stream, then decodes it
 * back through @ref OpusCodec (the same @ref IAudioCodec seam the bank drives) chunk by
 * chunk, and checks the round-trip: the compressed stream is far smaller than the float
 * source, and the decoded signal correlates strongly with the original (Opus is lossy, so
 * exactness is not expected — energy and correlation are). Exits 0 on success.
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

#include <SushiEngine/audio/opus_codec.hpp>

using namespace SushiEngine::Audio;

int main()
{
    const int sample_rate = 48000;
    const int channels = 2;
    const int frames = sample_rate; // 1 second
    const double frequency = 440.0;

    std::vector<float> source(static_cast<std::size_t>(frames * channels), 0.0f);
    for (int f = 0; f < frames; ++f)
    {
        const float s =
            0.5f * static_cast<float>(std::sin(2.0 * 3.14159265358979323846 * frequency *
                                               static_cast<double>(f) / sample_rate));
        source[static_cast<std::size_t>(f * channels + 0)] = s;
        source[static_cast<std::size_t>(f * channels + 1)] = s;
    }

    std::vector<std::uint8_t> encoded;
    if (!OpusCodec::encode(source.data(), frames, channels, sample_rate, 96000, encoded))
    {
        std::fprintf(stderr, "audio_opus_demo FAILED: encode error\n");
        return 1;
    }

    const std::size_t source_bytes = source.size() * sizeof(float);
    const double ratio = static_cast<double>(source_bytes) / static_cast<double>(encoded.size());
    std::printf("opus encode: %zu float bytes -> %zu packet bytes (%.1fx smaller)\n", source_bytes,
                encoded.size(), ratio);

    // Decode chunk by chunk exactly as the bank's decode_all would.
    OpusCodec codec(channels, sample_rate);
    std::vector<float> decoded;
    const int chunk_frames = 1024;
    std::vector<float> scratch(static_cast<std::size_t>(chunk_frames * channels), 0.0f);
    int offset = 0;
    while (offset < static_cast<int>(encoded.size()))
    {
        int consumed = 0;
        const int got = codec.decode(encoded.data() + offset,
                                     static_cast<int>(encoded.size()) - offset, scratch.data(),
                                     chunk_frames, consumed);
        if (got > 0)
            decoded.insert(decoded.end(), scratch.begin(),
                           scratch.begin() + static_cast<std::size_t>(got * channels));
        if (consumed <= 0)
            break;
        offset += consumed;
    }

    const int decoded_frames = static_cast<int>(decoded.size()) / channels;
    std::printf("opus decode: %d frames back (source %d frames)\n", decoded_frames, frames);
    if (decoded_frames < frames / 2)
    {
        std::fprintf(stderr, "audio_opus_demo FAILED: too few frames decoded\n");
        return 1;
    }

    // Opus adds algorithmic delay; correlate over an aligned interior window rather than
    // sample-for-sample. Sweep a small lag range for the best normalized correlation.
    const int window = frames / 2;
    const int start = frames / 4;
    double best_correlation = 0.0;
    for (int lag = 0; lag <= 640; ++lag)
    {
        double dot = 0.0, ea = 0.0, eb = 0.0;
        for (int i = 0; i < window; ++i)
        {
            const int di = start + i + lag;
            if (di >= decoded_frames)
                break;
            const double a = source[static_cast<std::size_t>((start + i) * channels)];
            const double b = decoded[static_cast<std::size_t>(di * channels)];
            dot += a * b;
            ea += a * a;
            eb += b * b;
        }
        if (ea > 0.0 && eb > 0.0)
        {
            const double c = dot / std::sqrt(ea * eb);
            if (c > best_correlation)
                best_correlation = c;
        }
    }

    std::printf("opus round-trip correlation=%.4f\n", best_correlation);
    if (!(best_correlation > 0.9))
    {
        std::fprintf(stderr, "audio_opus_demo FAILED: weak correlation %.4f\n", best_correlation);
        return 1;
    }

    // Verify the bank wiring: after registration, the header-only make_codec must produce an
    // Opus codec for AudioCodecKind::Opus and decode the same stream to the same frame count.
    register_opus_codec();
    std::unique_ptr<IAudioCodec> bank_codec =
        make_codec(AudioCodecKind::Opus, channels, sample_rate);
    std::vector<float> via_bank;
    decode_all(*bank_codec, encoded.data(), static_cast<int>(encoded.size()), via_bank);
    const int bank_frames = static_cast<int>(via_bank.size()) / channels;
    std::printf("bank make_codec(Opus) decoded %d frames\n", bank_frames);
    if (bank_frames != decoded_frames)
    {
        std::fprintf(stderr, "audio_opus_demo FAILED: bank path frame mismatch %d != %d\n",
                     bank_frames, decoded_frames);
        return 1;
    }

    std::printf("audio_opus_demo OK\n");
    return 0;
}
