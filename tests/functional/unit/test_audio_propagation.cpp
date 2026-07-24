/**************************************************************************/
/* test_audio_propagation.cpp                                            */
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

// Unit_Audio: the phase-S3 propagation model — the cubic-Lagrange fractional delay
// line, ISO 9613-1 air absorption, the distance-attenuation laws, and the per-source
// propagation processor whose changing delay is the Doppler shift (rising on approach,
// falling on recede, none when stationary, snapped on a teleport). Pure header-only
// maths against the real code, no device and no mocks.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/dsp/air_absorption.hpp>
#include <SushiEngine/audio/dsp/fractional_delay.hpp>
#include <SushiEngine/audio/propagation.hpp>
#include <SushiEngine/audio/voice.hpp>

using namespace SushiEngine::Audio;

namespace
{
    const double kTwoPi = 6.28318530717958647692;

    // Runs a phase-continuous sine of `freq` through `prop` while the distance ramps
    // linearly from `d0` to `d1` over `blocks` blocks of `n` samples, returning the
    // concatenated output.
    std::vector<float> run_propagation(SourcePropagation& prop, const VoiceDescriptor& desc,
                                       const Dsp::Atmosphere& atmo, double freq, double sr,
                                       int n, int blocks, float d0, float d1)
    {
        std::vector<float> out;
        out.reserve(static_cast<std::size_t>(n * blocks));
        std::vector<float> in(static_cast<std::size_t>(n));
        std::vector<float> block_out(static_cast<std::size_t>(n));
        long long sample_index = 0;
        for (int b = 0; b < blocks; ++b)
        {
            for (int i = 0; i < n; ++i)
            {
                in[static_cast<std::size_t>(i)] =
                    static_cast<float>(std::sin(kTwoPi * freq * sample_index / sr));
                ++sample_index;
            }
            const float t = blocks > 1 ? static_cast<float>(b) / (blocks - 1) : 0.0f;
            const float distance = d0 + t * (d1 - d0);
            prop.process(in.data(), block_out.data(), n, distance, atmo, desc);
            out.insert(out.end(), block_out.begin(), block_out.end());
        }
        return out;
    }

    // Upward-zero-crossing frequency estimate over a sample window.
    double crossing_frequency(const std::vector<float>& x, int start, int end, double sr)
    {
        int crossings = 0;
        for (int i = start + 1; i < end; ++i)
            if (x[static_cast<std::size_t>(i - 1)] <= 0.0f && x[static_cast<std::size_t>(i)] > 0.0f)
                ++crossings;
        return static_cast<double>(crossings) / ((end - start) / sr);
    }

    VoiceDescriptor spatial_descriptor()
    {
        VoiceDescriptor d;
        d.base_gain = 1.0f;
        d.spatial = true;
        d.model = DistanceModel::Inverse;
        d.min_distance = 1.0f;
        d.max_distance = 400.0f;
        d.rolloff = 1.0f;
        d.doppler_scale = 1.0f;
        d.propagation_delay = true;
        return d;
    }
} // namespace

TEST(Unit_Audio, FractionalDelayLineIsAccurate)
{
    Dsp::FractionalDelayLine line;
    line.prepare(64);
    for (int i = 0; i <= 20; ++i)
        line.push(static_cast<float>(i));
    EXPECT_NEAR(line.read(1.0f), 19.0f, 1e-4f); // last-1
    EXPECT_NEAR(line.read(5.0f), 15.0f, 1e-4f); // integer delay exact
    EXPECT_NEAR(line.read(5.5f), 14.5f, 1e-3f); // fractional interpolation

    // A delayed sine matches the analytic shift to high accuracy.
    Dsp::FractionalDelayLine s;
    s.prepare(256);
    const double sr = 48000.0, f = 200.0, delay = 17.3;
    double max_error = 0.0;
    for (int n = 0; n < 3000; ++n)
    {
        s.push(static_cast<float>(std::sin(kTwoPi * f * n / sr)));
        const float y = s.read(static_cast<float>(delay));
        if (n > 60)
        {
            const double want = std::sin(kTwoPi * f * (n - delay) / sr);
            max_error = std::max(max_error, std::fabs(y - want));
        }
    }
    EXPECT_LT(max_error, 1e-3);
}

TEST(Unit_Audio, AirAbsorptionAndSpeedOfSound)
{
    Dsp::Atmosphere atmo; // 20 C, 50%, 101.325 kPa
    EXPECT_NEAR(Dsp::speed_of_sound(atmo), 343.42f, 0.1f);

    // Absorption grows with frequency.
    const double a1k = Dsp::air_absorption_db_per_meter(1000.0, atmo);
    const double a4k = Dsp::air_absorption_db_per_meter(4000.0, atmo);
    const double a10k = Dsp::air_absorption_db_per_meter(10000.0, atmo);
    EXPECT_GT(a4k, a1k);
    EXPECT_GT(a10k, a4k);

    // The modelled cutoff falls with distance (nearer = brighter).
    const float near = Dsp::air_absorption_cutoff(1.0f, atmo);
    const float mid = Dsp::air_absorption_cutoff(50.0f, atmo);
    const float far = Dsp::air_absorption_cutoff(500.0f, atmo);
    EXPECT_GE(near, mid);
    EXPECT_GE(mid, far);
    EXPECT_LE(far, 5000.0f);
}

TEST(Unit_Audio, DistanceAttenuationModels)
{
    // Full gain within min; zero at/beyond max for every model.
    EXPECT_FLOAT_EQ(distance_attenuation(DistanceModel::Inverse, 0.5f, 1.0f, 100.0f, 1.0f), 1.0f);
    EXPECT_FLOAT_EQ(distance_attenuation(DistanceModel::Linear, 100.0f, 1.0f, 100.0f, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(distance_attenuation(DistanceModel::Inverse, 200.0f, 1.0f, 100.0f, 1.0f), 0.0f);

    // Linear halfway is 0.5.
    EXPECT_NEAR(distance_attenuation(DistanceModel::Linear, 50.5f, 1.0f, 100.0f, 1.0f), 0.5f, 1e-3f);

    // Monotonic decreasing within range.
    float previous = 2.0f;
    for (float d = 1.0f; d < 100.0f; d += 5.0f)
    {
        const float g = distance_attenuation(DistanceModel::Inverse, d, 1.0f, 100.0f, 1.0f);
        EXPECT_LE(g, previous);
        previous = g;
    }
}

TEST(Unit_Audio, PropagationDelayMatchesDistanceOverSpeedOfSound)
{
    const double sr = 48000.0;
    const int n = 256;
    Dsp::Atmosphere atmo;
    VoiceDescriptor desc = spatial_descriptor();

    SourcePropagation prop;
    prop.prepare(sr, n, 40000);

    const float distance = 34.3f; // ~ delay of distance / c * sr samples
    const float expected_delay = distance / Dsp::speed_of_sound(atmo) * static_cast<float>(sr);

    // Feed an impulse, then silence; find where it emerges.
    std::vector<float> out;
    std::vector<float> in(static_cast<std::size_t>(n), 0.0f);
    std::vector<float> block_out(static_cast<std::size_t>(n));
    for (int b = 0; b < 40; ++b)
    {
        if (b == 0)
            in[0] = 1.0f;
        else if (b == 1)
            in[0] = 0.0f;
        prop.process(in.data(), block_out.data(), n, distance, atmo, desc);
        out.insert(out.end(), block_out.begin(), block_out.end());
    }

    int peak_index = 0;
    float peak = 0.0f;
    for (int i = 0; i < static_cast<int>(out.size()); ++i)
    {
        const float a = std::fabs(out[static_cast<std::size_t>(i)]);
        if (a > peak)
        {
            peak = a;
            peak_index = i;
        }
    }
    EXPECT_NEAR(static_cast<float>(peak_index), expected_delay, 150.0f);
}

TEST(Unit_Audio, PropagationDopplerRisesApproachingFallsReceding)
{
    const double sr = 48000.0, freq = 1000.0;
    const int n = 256, blocks = 200;
    Dsp::Atmosphere atmo;
    VoiceDescriptor desc = spatial_descriptor();

    SourcePropagation approaching;
    approaching.prepare(sr, n, 40000);
    const std::vector<float> a = run_propagation(approaching, desc, atmo, freq, sr, n, blocks, 120.0f, 20.0f);
    const double approach_hz = crossing_frequency(a, n * 40, n * (blocks - 5), sr);

    SourcePropagation receding;
    receding.prepare(sr, n, 40000);
    const std::vector<float> r = run_propagation(receding, desc, atmo, freq, sr, n, blocks, 20.0f, 120.0f);
    const double recede_hz = crossing_frequency(r, n * 40, n * (blocks - 5), sr);

    EXPECT_GT(approach_hz, freq + 5.0); // pitched up
    EXPECT_LT(recede_hz, freq - 5.0);   // pitched down
}

TEST(Unit_Audio, PropagationHasNoDopplerWhenStationary)
{
    const double sr = 48000.0, freq = 1000.0;
    const int n = 256, blocks = 120;
    Dsp::Atmosphere atmo;
    VoiceDescriptor desc = spatial_descriptor();

    SourcePropagation prop;
    prop.prepare(sr, n, 40000);
    const std::vector<float> out = run_propagation(prop, desc, atmo, freq, sr, n, blocks, 40.0f, 40.0f);
    const double measured = crossing_frequency(out, n * 40, n * (blocks - 2), sr);
    EXPECT_NEAR(measured, freq, 15.0); // no shift for a fixed distance
}

TEST(Unit_Audio, PropagationDelayToggleDisablesDoppler)
{
    const double sr = 48000.0, freq = 1000.0;
    const int n = 256, blocks = 120;
    Dsp::Atmosphere atmo;
    VoiceDescriptor desc = spatial_descriptor();
    desc.propagation_delay = false; // delay + Doppler off

    SourcePropagation prop;
    prop.prepare(sr, n, 40000);
    const std::vector<float> out = run_propagation(prop, desc, atmo, freq, sr, n, blocks, 120.0f, 20.0f);
    // Even with the distance shrinking fast, no delay means no pitch shift.
    const double measured = crossing_frequency(out, n * 10, n * (blocks - 2), sr);
    EXPECT_NEAR(measured, freq, 15.0);
}

TEST(Unit_Audio, PropagationTeleportSnapsWithoutSweep)
{
    const double sr = 48000.0, freq = 1000.0;
    const int n = 256;
    Dsp::Atmosphere atmo;
    VoiceDescriptor desc = spatial_descriptor();

    SourcePropagation prop;
    prop.prepare(sr, n, 60000);

    std::vector<float> in(static_cast<std::size_t>(n));
    std::vector<float> out(static_cast<std::size_t>(n));
    long long idx = 0;
    auto feed = [&](float distance) {
        for (int i = 0; i < n; ++i)
            in[static_cast<std::size_t>(i)] = static_cast<float>(std::sin(kTwoPi * freq * idx++ / sr));
        prop.process(in.data(), out.data(), n, distance, atmo, desc);
    };

    // Settle at a moderate distance (filling the delay history), then teleport nearer.
    // A near teleport reads from history that already exists, so the snap's effect is
    // cleanly observable; a far teleport would (correctly) be silent until the sound in
    // flight arrives.
    for (int b = 0; b < 40; ++b)
        feed(60.0f);
    feed(8.0f); // teleport: must snap, not sweep

    // After the snap, hold at 8 m and confirm the pitch is back to normal within a few
    // blocks (a sweep would keep it detuned for many blocks) and that no sample blew up.
    float peak = 0.0f;
    std::vector<float> tail;
    for (int b = 0; b < 30; ++b)
    {
        feed(8.0f);
        for (int i = 0; i < n; ++i)
        {
            const float a = std::fabs(out[static_cast<std::size_t>(i)]);
            peak = std::max(peak, a);
            EXPECT_TRUE(std::isfinite(out[static_cast<std::size_t>(i)]));
        }
        if (b >= 8)
            tail.insert(tail.end(), out.begin(), out.end());
    }
    EXPECT_GT(peak, 0.0f);  // audible (near source), not swept into silence
    EXPECT_LT(peak, 1.0f);  // distance-attenuated, never amplified
    const double settled = crossing_frequency(tail, 0, static_cast<int>(tail.size()), sr);
    EXPECT_NEAR(settled, freq, 20.0); // pitch normal again: the delay snapped
}
