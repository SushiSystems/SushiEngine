/**************************************************************************/
/* test_audio_bank.cpp                                                    */
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

// Unit_Audio: the phase-S8 asset pipeline — the codecs (PCM round-trip, IMA-ADPCM
// encode/decode fidelity + 4:1 density + chunked==one-shot streaming safety), the bank
// binary (build → load round-trip, media lookup/decode, rejection of malformed data),
// the event/container model (Sound/Random/Sequence/Switch/Blend selection), the
// bank-backed IEmitterSourceFactory, and the streaming decoder/source (a memory source
// pumped into the ring, rendered, and ending after it drains). Header-only, no device.

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    std::vector<float> make_tone(int frames, double freq, double sr = 48000.0, float amp = 0.4f)
    {
        std::vector<float> v(static_cast<std::size_t>(frames));
        for (int i = 0; i < frames; ++i)
            v[static_cast<std::size_t>(i)] =
                amp * static_cast<float>(std::sin(2.0 * 3.14159265358979 * i * freq / sr));
        return v;
    }

    double mean_abs_error(const std::vector<float>& a, const std::vector<float>& b)
    {
        const int n = static_cast<int>(std::min(a.size(), b.size()));
        double e = 0.0;
        for (int i = 0; i < n; ++i)
            e += std::fabs(static_cast<double>(a[static_cast<std::size_t>(i)]) -
                           b[static_cast<std::size_t>(i)]);
        return n > 0 ? e / n : 0.0;
    }

    std::vector<std::uint8_t> pcm16_bytes(const std::vector<float>& in)
    {
        std::vector<std::uint8_t> out(in.size() * 2);
        for (std::size_t i = 0; i < in.size(); ++i)
        {
            int s = static_cast<int>(in[i] * 32768.0f);
            s = s > 32767 ? 32767 : (s < -32768 ? -32768 : s);
            out[i * 2] = static_cast<std::uint8_t>(s & 0xff);
            out[i * 2 + 1] = static_cast<std::uint8_t>((s >> 8) & 0xff);
        }
        return out;
    }
}

// PCM16 decodes back to the source within quantisation error.
TEST(Unit_Audio, CodecPcm16RoundTrip)
{
    const std::vector<float> tone = make_tone(1000, 440.0);
    const std::vector<std::uint8_t> bytes = pcm16_bytes(tone);
    PCMCodec codec(1, true);
    std::vector<float> out;
    decode_all(codec, bytes.data(), static_cast<int>(bytes.size()), out);
    ASSERT_EQ(out.size(), tone.size());
    EXPECT_LT(mean_abs_error(out, tone), 1.0 / 32768.0 + 1e-6);
}

// IMA-ADPCM: ~4:1 density, low error, and a chunked decode equals the one-shot decode.
TEST(Unit_Audio, CodecAdpcmFidelityAndStreamingSafety)
{
    const int frames = 4000;
    const std::vector<float> tone = make_tone(frames, 330.0);
    std::vector<std::uint8_t> enc;
    ImaAdpcmCodec::encode(tone.data(), frames, 1, enc);
    // 4-bit per sample → about a quarter of PCM16.
    EXPECT_NEAR(static_cast<double>(enc.size()), frames * 0.5, frames * 0.05);

    ImaAdpcmCodec codec(1);
    std::vector<float> one_shot;
    decode_all(codec, enc.data(), static_cast<int>(enc.size()), one_shot);
    ASSERT_EQ(one_shot.size(), tone.size());
    EXPECT_LT(mean_abs_error(one_shot, tone), 0.02); // lossy but faithful

    // Odd-sized chunks must reconstruct the same stream (byte-boundary carry).
    ImaAdpcmCodec chunked(1);
    chunked.reset();
    std::vector<float> pieced;
    std::vector<float> scratch(37);
    int off = 0;
    while (off < static_cast<int>(enc.size()))
    {
        int consumed = 0;
        const int got = chunked.decode(enc.data() + off, static_cast<int>(enc.size()) - off,
                                       scratch.data(), 37, consumed);
        if (got <= 0 || consumed <= 0)
            break;
        for (int i = 0; i < got; ++i)
            pieced.push_back(scratch[static_cast<std::size_t>(i)]);
        off += consumed;
    }
    ASSERT_EQ(pieced.size(), one_shot.size());
    for (std::size_t i = 0; i < pieced.size(); ++i)
        EXPECT_EQ(pieced[i], one_shot[i]);
}

// A bank round-trips: build → load, and a media entry decodes to the original samples.
TEST(Unit_Audio, BankBuildLoadRoundTrip)
{
    const int frames = 2000;
    const std::vector<float> tone = make_tone(frames, 220.0);
    const std::vector<std::uint8_t> pcm = pcm16_bytes(tone);

    BankBuilder builder;
    builder.add_media(42, AudioCodecKind::PCM16, 1, 48000, frames, pcm);
    const std::vector<std::uint8_t> bytes = builder.build();

    Bank bank;
    ASSERT_TRUE(bank.load(bytes));
    EXPECT_EQ(bank.media_count(), 1u);

    BankMediaInformation info;
    ASSERT_TRUE(bank.find_media(42, info));
    EXPECT_EQ(info.channels, 1u);
    EXPECT_EQ(info.sample_rate, 48000u);
    EXPECT_EQ(info.frame_count, static_cast<std::uint32_t>(frames));

    std::vector<float> decoded;
    ASSERT_TRUE(bank.decode_media(42, decoded));
    EXPECT_LT(mean_abs_error(decoded, tone), 1.0 / 32768.0 + 1e-6);

    // Unknown media and malformed data both fail cleanly.
    EXPECT_FALSE(bank.find_media(999, info));
    Bank junk;
    const std::vector<std::uint8_t> garbage(8, 0xab);
    EXPECT_FALSE(junk.load(garbage));
}

// The event/container model selects per its rules.
TEST(Unit_Audio, EventContainerSelection)
{
    EventDatabase db;
    const std::uint32_t a = db.add_sound(1);
    const std::uint32_t b = db.add_sound(2);
    const std::uint32_t c = db.add_sound(3);
    (void)a; (void)b; (void)c; // children 0,1,2 are contiguous

    const std::uint32_t random = db.add_container(ContainerKind::Random, 0, 3);
    const std::uint32_t sequence = db.add_container(ContainerKind::Sequence, 0, 3);
    const std::uint32_t sw = db.add_container(ContainerKind::Switch, 0, 3);
    const std::uint32_t blend = db.add_container(ContainerKind::Blend, 0, 3);
    db.add_event(10, random);
    db.add_event(11, sequence);
    db.add_event(12, sw);
    db.add_event(13, blend);
    db.add_event(14, a); // an event straight to a leaf

    ResolveContext context;
    EXPECT_EQ(db.resolve(14, context), 1u); // leaf

    // Random stays within the child media set.
    bool saw_variation = false;
    std::uint32_t first = db.resolve(10, context);
    for (int i = 0; i < 32; ++i)
    {
        const std::uint32_t r = db.resolve(10, context);
        EXPECT_TRUE(r == 1u || r == 2u || r == 3u);
        if (r != first)
            saw_variation = true;
    }
    EXPECT_TRUE(saw_variation);

    // Sequence cycles 1,2,3,1,...
    EXPECT_EQ(db.resolve(11, context), 1u);
    EXPECT_EQ(db.resolve(11, context), 2u);
    EXPECT_EQ(db.resolve(11, context), 3u);
    EXPECT_EQ(db.resolve(11, context), 1u);

    // Switch indexes by the selector.
    context.switch_value = 2;
    EXPECT_EQ(db.resolve(12, context), 3u);
    context.switch_value = 0;
    EXPECT_EQ(db.resolve(12, context), 1u);

    // Blend maps [0,1] across the children.
    context.blend = 0.0f;
    EXPECT_EQ(db.resolve(13, context), 1u);
    context.blend = 1.0f;
    EXPECT_EQ(db.resolve(13, context), 3u);

    EXPECT_EQ(db.resolve(777, context), INVALID_SOUND); // unknown event
}

// The bank-backed factory turns a posted event into a playable voice.
TEST(Unit_Audio, BankSourceFactoryCreatesVoices)
{
    const int frames = 500;
    std::vector<std::uint8_t> e0, e1;
    ImaAdpcmCodec::encode(make_tone(frames, 300.0).data(), frames, 1, e0);
    ImaAdpcmCodec::encode(make_tone(frames, 500.0).data(), frames, 1, e1);

    EventDatabase db;
    db.add_sound(20);
    db.add_sound(21);
    const std::uint32_t rnd = db.add_container(ContainerKind::Random, 0, 2);
    db.add_event(1, rnd);

    BankBuilder builder;
    builder.add_media(20, AudioCodecKind::ImaAdpcm, 1, 48000, frames, e0);
    builder.add_media(21, AudioCodecKind::ImaAdpcm, 1, 48000, frames, e1);
    builder.set_events(db);

    Bank bank;
    ASSERT_TRUE(bank.load(builder.build()));

    BankSourceFactory factory(bank, false);
    for (int i = 0; i < 5; ++i)
    {
        std::unique_ptr<VoiceSource> source = factory.create(1);
        ASSERT_NE(source, nullptr);
        std::vector<float> out(64, 0.0f);
        EXPECT_TRUE(source->render(out.data(), 64));
    }
    EXPECT_EQ(factory.create(999), nullptr); // unknown event, no direct media
}

// Streaming: pump a memory source into the ring, render it, and it ends after it drains.
TEST(Unit_Audio, StreamingDecoderFeedsSource)
{
    const int frames = 3000;
    const std::vector<float> tone = make_tone(frames, 440.0);
    std::vector<std::uint8_t> enc;
    ImaAdpcmCodec::encode(tone.data(), frames, 1, enc);

    MemoryDataSource source(enc.data(), static_cast<std::uint32_t>(enc.size()));
    StreamingDecoder decoder(source, std::unique_ptr<IAudioCodec>(new ImaAdpcmCodec(1)), 8192, false);
    StreamingSource voice(decoder);

    // Fully pump the (small) asset into the ring.
    for (int i = 0; i < 64; ++i)
        decoder.pump(512);

    double energy = 0.0;
    int rendered = 0;
    std::vector<float> out(256, 0.0f);
    bool alive = true;
    for (int b = 0; b < 40 && alive; ++b)
    {
        alive = voice.render(out.data(), 256);
        for (float s : out)
            energy += std::fabs(static_cast<double>(s));
        rendered += 256;
        for (int i = 0; i < 8; ++i) // keep the (already-exhausted) pump honest
            decoder.pump(512);
    }
    EXPECT_GT(energy, 1.0);                  // real audio came through
    EXPECT_TRUE(decoder.finished());         // asset drained
    EXPECT_GE(rendered, frames);             // at least the whole asset played
}

// Layer plays all children; Blend cross-fades the two the parameter straddles.
TEST(Unit_Audio, EventLayerAndBlend)
{
    EventDatabase db;
    db.add_sound(1); // node 0
    db.add_sound(2); // node 1
    const std::uint32_t layer = db.add_container(ContainerKind::Layer, 0, 2);
    const std::uint32_t blend = db.add_container(ContainerKind::Blend, 0, 2);
    db.add_event(10, layer);
    db.add_event(11, blend);

    ResolveContext context;
    std::vector<ResolvedSound> out;

    db.resolve_all(10, context, out); // Layer → both sounds
    EXPECT_EQ(out.size(), 2u);

    context.blend = 0.5f;
    db.resolve_all(11, context, out); // Blend mid → both, equal-power (~0.707 each)
    ASSERT_EQ(out.size(), 2u);
    EXPECT_NEAR(out[0].gain, 0.70710678f, 0.02f);
    EXPECT_NEAR(out[1].gain, 0.70710678f, 0.02f);

    context.blend = 0.0f;
    db.resolve_all(11, context, out); // Blend fully to child 0 → only sound 1 sounds
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].media_id, 1u);
    EXPECT_NEAR(out[0].gain, 1.0f, 0.02f);
}

// Weighted Random favours the heavier child.
TEST(Unit_Audio, EventWeightedRandom)
{
    EventDatabase db;
    const std::uint32_t a = db.add_sound(1);
    const std::uint32_t b = db.add_sound(2);
    db.set_weight(a, 9.0f);
    db.set_weight(b, 1.0f);
    const std::uint32_t rnd = db.add_container(ContainerKind::Random, 0, 2);
    db.add_event(20, rnd);

    ResolveContext context;
    std::vector<ResolvedSound> out;
    int picked_a = 0;
    for (int i = 0; i < 2000; ++i)
    {
        context.seed = static_cast<std::uint32_t>(i);
        db.resolve_all(20, context, out);
        ASSERT_EQ(out.size(), 1u);
        if (out[0].media_id == 1u)
            ++picked_a;
    }
    EXPECT_GT(picked_a, 1500); // ~90% weight → clearly majority
    EXPECT_LT(picked_a, 2000);
}

// A layered voice mixes its children at their gains.
TEST(Unit_Audio, LayeredVoiceMixes)
{
    std::vector<float> a(64, 1.0f), b(64, 0.5f);
    LayeredVoiceSource layered;
    layered.add(std::unique_ptr<VoiceSource>(new BufferSource(a.data(), 64, true)), 0.5f);
    layered.add(std::unique_ptr<VoiceSource>(new BufferSource(b.data(), 64, true)), 0.5f);
    layered.prepare(48000.0, 64);
    EXPECT_EQ(layered.layer_count(), 2u);

    std::vector<float> out(64, 0.0f);
    EXPECT_TRUE(layered.render(out.data(), 64));
    // 1.0*0.5 + 0.5*0.5 = 0.75
    EXPECT_NEAR(out[0], 0.75f, 0.005f);
    EXPECT_NEAR(out[32], 0.75f, 0.005f);
}
