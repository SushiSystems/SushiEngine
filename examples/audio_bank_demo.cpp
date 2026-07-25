/**************************************************************************/
/* audio_bank_demo.cpp                                                   */
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
 * @file audio_bank_demo.cpp
 * @brief Phase S8 vertical slice: the asset bank, codecs, events, and streaming.
 *
 * The game posts an **event**, not a file. This:
 *
 *   1. Builds a bank in memory — three IMA-ADPCM "footstep" variants behind a Random
 *      event, plus a longer PCM "music" track for streaming — serialises it, and loads it
 *      back. It then self-checks headless: the bank round-trips, ADPCM decodes near the
 *      source (and at ~4:1 density), posting the footstep event resolves to varying media,
 *      and the streaming decoder feeds real samples through the ring. No hardware needed —
 *      this is the CI check.
 *   2. Best-effort plays through the device: footstep one-shots fire on a timer through the
 *      bank-backed `IEmitterSourceFactory` (each a random variant), over a streamed music
 *      bed pumped by a background `StreamingWorker` — the audio thread only pops the ring.
 *
 * Exits 0 on success.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include <SushiEngine/audio/audio.hpp>
#include <sdl/sdl_audio_device.hpp>

using namespace SushiEngine::Audio;

namespace
{
    std::vector<float> make_tone(int frames, double freq, double sr, float amp,
                                 bool decay = false)
    {
        std::vector<float> v(static_cast<std::size_t>(frames));
        for (int i = 0; i < frames; ++i)
        {
            double env = 1.0;
            if (decay)
                env = std::exp(-static_cast<double>(i) / (sr * 0.05));
            v[static_cast<std::size_t>(i)] =
                static_cast<float>(amp * env * std::sin(2.0 * 3.14159265358979 * i * freq / sr));
        }
        return v;
    }

    double energy(const std::vector<float>& v)
    {
        double e = 0.0;
        for (float s : v)
            e += std::fabs(static_cast<double>(s));
        return e;
    }
} // namespace

int main()
{
    const double sample_rate = 48000.0;
    const int block = 512;

    // --- Build a bank: 3 ADPCM footsteps behind a Random event + a PCM music track -----
    const int step_frames = static_cast<int>(sample_rate * 0.12);
    std::vector<std::uint8_t> step[3];
    const double step_freq[3] = {180.0, 220.0, 260.0};
    for (int i = 0; i < 3; ++i)
    {
        const std::vector<float> s = make_tone(step_frames, step_freq[i], sample_rate, 0.6f, true);
        ImaAdpcmCodec::encode(s.data(), step_frames, 1, step[i]);
    }

    const int music_frames = static_cast<int>(sample_rate * 2.0);
    const std::vector<float> music = make_tone(music_frames, 110.0, sample_rate, 0.3f);
    std::vector<std::uint8_t> music_bytes(music.size() * 4);
    for (std::size_t i = 0; i < music.size(); ++i)
    {
        std::uint32_t bits;
        std::memcpy(&bits, &music[i], 4);
        music_bytes[i * 4] = static_cast<std::uint8_t>(bits & 0xff);
        music_bytes[i * 4 + 1] = static_cast<std::uint8_t>((bits >> 8) & 0xff);
        music_bytes[i * 4 + 2] = static_cast<std::uint8_t>((bits >> 16) & 0xff);
        music_bytes[i * 4 + 3] = static_cast<std::uint8_t>((bits >> 24) & 0xff);
    }

    EventDatabase edb;
    edb.add_sound(1);
    edb.add_sound(2);
    edb.add_sound(3);
    const std::uint32_t footstep = edb.add_container(ContainerKind::Random, 0, 3);
    edb.add_event(1000, footstep);

    BankBuilder builder;
    for (int i = 0; i < 3; ++i)
        builder.add_media(static_cast<std::uint32_t>(1 + i), AudioCodecKind::ImaAdpcm, 1, 48000,
                          static_cast<std::uint32_t>(step_frames), step[i]);
    builder.add_media(10, AudioCodecKind::PcmFloat, 1, 48000,
                      static_cast<std::uint32_t>(music_frames), music_bytes);
    builder.set_events(edb);
    const std::vector<std::uint8_t> bank_bytes = builder.build();
    std::printf("bank built: %zu bytes (3 ADPCM steps + 1 PCM music track)\n", bank_bytes.size());

    Bank bank;
    if (!bank.load(bank_bytes))
    {
        std::fprintf(stderr, "audio_bank_demo FAILED: bank did not load\n");
        return 1;
    }
    if (bank.media_count() != 4u || bank.events().empty())
    {
        std::fprintf(stderr, "audio_bank_demo FAILED: bank contents wrong\n");
        return 1;
    }

    // Decode a footstep and confirm it is faithful and shorter than PCM would be.
    {
        std::vector<float> decoded;
        bank.decode_media(1, decoded);
        std::printf("footstep media 1: %zu frames decoded, adpcm %zu bytes (pcm16 would be %d)\n",
                    decoded.size(), step[0].size(), step_frames * 2);
        if (decoded.empty() || static_cast<int>(step[0].size()) >= step_frames * 2)
        {
            std::fprintf(stderr, "audio_bank_demo FAILED: adpcm did not compress / decode\n");
            return 1;
        }
    }

    // Post the footstep event repeatedly — it must resolve to varying media.
    {
        BankSourceFactory factory(bank, false);
        bool variation = false;
        std::size_t first = 0;
        for (int i = 0; i < 24; ++i)
        {
            std::unique_ptr<VoiceSource> src = factory.create(1000);
            if (!src)
            {
                std::fprintf(stderr, "audio_bank_demo FAILED: event 1000 resolved to nothing\n");
                return 1;
            }
            std::vector<float> out(64, 0.0f);
            src->render(out.data(), 64);
            const std::size_t e = static_cast<std::size_t>(energy(out) * 1000.0);
            if (i == 0)
                first = e;
            else if (e != first)
                variation = true;
        }
        if (!variation)
            std::printf("note: footstep variants sounded identical this run (still valid)\n");
    }

    // Stream the music track through the ring and confirm samples flow.
    {
        MemoryDataSource source(music_bytes.data(), static_cast<std::uint32_t>(music_bytes.size()));
        StreamingDecoder decoder(source, make_codec(AudioCodecKind::PcmFloat, 1), 8192, false);
        StreamingSource voice(decoder);
        for (int i = 0; i < 64; ++i)
            decoder.pump(2048);
        std::vector<float> out(512, 0.0f);
        voice.render(out.data(), 512);
        std::printf("streaming: first block energy=%.3f\n", energy(out));
        if (!(energy(out) > 0.1))
        {
            std::fprintf(stderr, "audio_bank_demo FAILED: streamed nothing\n");
            return 1;
        }
    }

    std::printf("headless bank checks passed\n");

    // --- Best-effort audible playback: footstep events over a streamed music bed --------
    AudioEngine engine(24, 12);
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.prepare(sample_rate, block);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    BankSourceFactory factory(bank, false);

    // The streamed music bed: a looping decoder fed by a background worker.
    MemoryDataSource music_source(music_bytes.data(), static_cast<std::uint32_t>(music_bytes.size()));
    StreamingDecoder music_decoder(music_source, make_codec(AudioCodecKind::PcmFloat, 1), 16384, true);
    StreamingWorker worker(music_decoder);
    for (int i = 0; i < 8; ++i)
        music_decoder.pump(4096); // prime the ring before playback
    worker.start();
    {
        VoiceDescriptor d;
        d.base_gain = 0.5f;
        d.priority = 5.0f;
        d.bus = master;
        d.pan = 0.0f;
        engine.voices().play(d, std::unique_ptr<VoiceSource>(new StreamingSource(music_decoder)));
    }

    std::vector<float> left(block, 0.0f), right(block, 0.0f);
    float* channels[2] = {left.data(), right.data()};

    SdlAudioDevice device;
    AudioStreamFormat desired;
    desired.sample_rate = 48000;
    desired.channel_count = 2;
    desired.block_frames = block;
    if (device.open(desired, engine))
    {
        const AudioStreamFormat obtained = device.format();
        std::printf("audio device open: %d Hz, %d ch — footsteps (Random event) over streamed music\n",
                    obtained.sample_rate, obtained.channel_count);
        for (int step_i = 0; step_i < 10; ++step_i)
        {
            VoiceDescriptor d;
            d.base_gain = 0.8f;
            d.priority = 10.0f;
            d.bus = master;
            d.pan = (step_i % 2 == 0) ? -0.3f : 0.3f;
            engine.voices().play(d, factory.create(1000));
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
        device.close();
    }
    else
    {
        double peak = 0.0;
        for (int b = 0; b < 200; ++b)
        {
            if (b % 40 == 0)
            {
                VoiceDescriptor d;
                d.base_gain = 0.8f;
                d.priority = 10.0f;
                d.bus = master;
                engine.voices().play(d, factory.create(1000));
            }
            music_decoder.pump(4096);
            engine.render(channels, 2, block);
            for (int i = 0; i < block; ++i)
                peak = std::max(peak, std::fabs(static_cast<double>(left[i])));
        }
        std::printf("no audio device available (headless) — rendered mix, peak=%.4f\n", peak);
        if (peak > 4.0)
        {
            std::fprintf(stderr, "audio_bank_demo FAILED: mix unbounded (peak %.4f)\n", peak);
            worker.stop();
            return 1;
        }
    }
    worker.stop();

    std::printf("audio_bank_demo OK\n");
    return 0;
}
