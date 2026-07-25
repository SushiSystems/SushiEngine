/**************************************************************************/
/* test_audio_advanced.cpp                                              */
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
 * @file test_audio_advanced.cpp
 * @brief The dependency-free AAA gap-closers: authoring, MagLS + anthropometric, ray tracing.
 */

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    // A synthetic HRTF: a delayed, level-shaded impulse per ear driven by the azimuth, so a left
    // source energizes the left ear earlier and louder. Needs no external SOFA file.
    class SyntheticHrtf final : public IHrtfDatabase
    {
        public:
            int ir_length() const noexcept override { return 64; }
            double sample_rate() const noexcept override { return 48000.0; }

            void get_hrir(float front, float left, float up, float* left_ir,
                          float* right_ir) const noexcept override
            {
                (void)up;
                for (int i = 0; i < ir_length(); ++i)
                {
                    left_ir[i] = 0.0f;
                    right_ir[i] = 0.0f;
                }
                float len = std::sqrt(front * front + left * left);
                if (len < 1e-6f)
                    len = 1.0f;
                const float lateral = left / len; // +1 left, -1 right
                const int left_delay = 4 + static_cast<int>((1.0f - lateral) * 10.0f);
                const int right_delay = 4 + static_cast<int>((1.0f + lateral) * 10.0f);
                left_ir[left_delay] = 0.6f + 0.4f * lateral;
                right_ir[right_delay] = 0.6f - 0.4f * lateral;
            }
    };
} // namespace

TEST(Unit_Audio, AuthoringFlattenPreservesContainerSemantics)
{
    AudioAuthoringProject project;
    const std::uint32_t m0 = project.add_media("a", AudioCodecKind::PcmFloat, 1, 48000, 128);
    const std::uint32_t m1 = project.add_media("b", AudioCodecKind::PcmFloat, 1, 48000, 128);

    const int blend = project.create_container(ContainerKind::Blend);
    project.add_child(blend, project.create_sound(m0));
    project.add_child(blend, project.create_sound(m1));
    const int layer = project.create_container(ContainerKind::Layer);
    project.add_child(layer, project.create_sound(m0));
    project.add_child(layer, project.create_sound(m1));

    const EventId music = project.create_event("music", blend);
    const EventId stinger = project.create_event("stinger", layer);

    EventDatabase db;
    ASSERT_TRUE(project.flatten(db));

    ResolveContext lo;
    lo.blend = 0.0f;
    ResolveContext hi;
    hi.blend = 1.0f;
    EXPECT_EQ(db.resolve(music, lo), m0);
    EXPECT_EQ(db.resolve(music, hi), m1);

    std::vector<ResolvedSound> sounds;
    db.resolve_all(stinger, ResolveContext{}, sounds);
    EXPECT_EQ(sounds.size(), 2u);
}

TEST(Unit_Audio, AuthoringBakeRoundTripsThroughBank)
{
    AudioAuthoringProject project;
    const std::uint32_t m = project.add_media("x", AudioCodecKind::PcmFloat, 1, 48000, 64);
    const int root = project.create_sound(m);
    const EventId ev = project.create_event("one", root);

    BankBuilder builder;
    ASSERT_TRUE(project.bake(builder, [&](std::uint32_t) -> const std::vector<std::uint8_t>& {
        static std::vector<std::uint8_t> bytes(64 * sizeof(float), 0);
        return bytes;
    }));
    const std::vector<std::uint8_t> blob = builder.build();
    Bank bank;
    ASSERT_TRUE(bank.load(blob.data(), blob.size()));
    EXPECT_EQ(bank.events().resolve(ev, ResolveContext{}), m);
}

TEST(Unit_Audio, MaglsDecodePreservesLaterality)
{
    SyntheticHrtf hrtf;
    MaglsBinauralDecoder magls;
    ASSERT_TRUE(magls.configure(3, hrtf, 48000.0, 256, 200, 1500.0));
    EXPECT_EQ(magls.channel_count(), 16);
    EXPECT_TRUE(magls.valid());

    BinauralSpatializer spat;
    spat.configure(3, 48000.0, 256);
    spat.set_magls_decoder(&magls);
    ASSERT_TRUE(spat.uses_magls());

    const int block = 256;
    std::vector<float> mono(static_cast<std::size_t>(block), 0.0f);
    for (int i = 0; i < block; ++i)
        mono[static_cast<std::size_t>(i)] =
            0.5f * static_cast<float>(std::sin(2.0 * 3.14159265 * 440.0 * i / 48000.0));

    auto energy = [&](float lateral, double& le, double& re) {
        std::vector<float> l(static_cast<std::size_t>(block)), r(static_cast<std::size_t>(block));
        le = 0.0;
        re = 0.0;
        for (int b = 0; b < 8; ++b)
        {
            std::fill(l.begin(), l.end(), 0.0f);
            std::fill(r.begin(), r.end(), 0.0f);
            spat.begin_block(block);
            spat.encode(mono.data(), block, 0.0f, lateral, 0.0f, 1.0f);
            spat.decode_binaural(l.data(), r.data(), block);
            if (b >= 4)
                for (int i = 0; i < block; ++i)
                {
                    le += l[static_cast<std::size_t>(i)] * l[static_cast<std::size_t>(i)];
                    re += r[static_cast<std::size_t>(i)] * r[static_cast<std::size_t>(i)];
                }
        }
    };

    double ll, lr, rl, rr;
    energy(1.0f, ll, lr);
    energy(-1.0f, rl, rr);
    EXPECT_GT(ll, lr);
    EXPECT_GT(rr, rl);
}

TEST(Unit_Audio, AnthropometricWarpLengthensItd)
{
    SyntheticHrtf hrtf;
    const int n = hrtf.ir_length();

    auto arrival = [](const float* ir, int len) {
        int idx = 0;
        float peak = 0.0f;
        for (int i = 0; i < len; ++i)
            if (std::fabs(ir[i]) > peak)
            {
                peak = std::fabs(ir[i]);
                idx = i;
            }
        return idx;
    };

    std::vector<float> bl(static_cast<std::size_t>(n)), br(static_cast<std::size_t>(n));
    hrtf.get_hrir(0.0f, 1.0f, 0.0f, bl.data(), br.data());
    const int base_itd = std::abs(arrival(br.data(), n) - arrival(bl.data(), n));

    AnthropometricHrtfDatabase big(hrtf, 0.11f, 0.0875f);
    std::vector<float> gl(static_cast<std::size_t>(n)), gr(static_cast<std::size_t>(n));
    big.get_hrir(0.0f, 1.0f, 0.0f, gl.data(), gr.data());
    const int big_itd = std::abs(arrival(gr.data(), n) - arrival(gl.data(), n));

    EXPECT_GE(big_itd, base_itd);
}

TEST(Unit_Audio, RayTracedRt60MatchesSabine)
{
    const float lx = 10.0f, ly = 6.0f, lz = 4.0f, alpha = 0.2f;
    AcousticMesh room;
    AcousticMaterial wall;
    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
    {
        wall.absorption[b] = alpha;
        wall.scattering[b] = 0.2f;
    }
    room.add_box(AudioVec3{lx * 0.5f, ly * 0.5f, lz * 0.5f},
                 AudioVec3{lx * 0.5f, ly * 0.5f, lz * 0.5f}, room.add_material(wall));

    RayTracedAcoustics tracer;
    RayTraceParams params;
    params.rays = 8000;
    params.receiver_radius = 0.6f;
    const RoomImpulseResponse rir =
        tracer.bake(room, AudioVec3{2, 3, 2}, AudioVec3{8, 3, 2}, params);

    const double volume = lx * ly * lz;
    const double surface = 2.0 * (lx * ly + ly * lz + lx * lz);
    const double sabine = 0.161 * volume / (surface * alpha);
    ASSERT_GT(rir.rt60[0], 0.0f);
    const double ratio = rir.rt60[0] / sabine;
    EXPECT_GT(ratio, 0.4);
    EXPECT_LT(ratio, 2.5);
}

TEST(Unit_Audio, MaekawaDiffractionRisesWithFrequency)
{
    AcousticMesh barrier;
    barrier.add_box(AudioVec3{5.0f, 0.0f, 1.5f}, AudioVec3{0.1f, 3.0f, 1.5f},
                    barrier.add_material(AcousticMaterial::concrete()));
    AcousticBlas blas;
    blas.build(barrier);
    AcousticScene scene;
    AcousticInstance instance;
    instance.blas = &blas;
    instance.set_position(AudioVec3{0, 0, 0});
    scene.add_instance(instance);
    scene.commit();

    float db[ACOUSTIC_BAND_COUNT];
    RayTracedAcoustics::maekawa_diffraction_db(scene, barrier, AudioVec3{0, 0, 1},
                                               AudioVec3{10, 0, 1}, db);
    EXPECT_GT(db[0], 0.0f);
    EXPECT_GE(db[ACOUSTIC_BAND_COUNT - 1], db[0]);

    float clear[ACOUSTIC_BAND_COUNT];
    RayTracedAcoustics::maekawa_diffraction_db(scene, barrier, AudioVec3{0, 5, 1},
                                               AudioVec3{2, 5, 1}, clear);
    EXPECT_FLOAT_EQ(clear[0], 0.0f);
}
