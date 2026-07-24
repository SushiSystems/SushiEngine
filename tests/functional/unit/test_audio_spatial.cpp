/**************************************************************************/
/* test_audio_spatial.cpp                                                */
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

// Unit_Audio: the phase-S4 spatializer — real (ACN/SN3D) spherical harmonics, the
// ambisonic encode/decode kernel, and the analytic binaural decode (left/right level
// cues, the interaural time difference, front symmetry, and head-tracking through the
// engine). Pure header-only maths against the real code, no device and no mocks.

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    const double kTwoPi = 6.28318530717958647692;

    double rms(const std::vector<float>& x)
    {
        double sum = 0.0;
        for (float v : x)
            sum += static_cast<double>(v) * v;
        return x.empty() ? 0.0 : std::sqrt(sum / x.size());
    }

    double sh_kernel(int order, const float* a, const float* b)
    {
        double sum = 0.0;
        for (int i = 0; i < Dsp::ambisonic_channel_count(order); ++i)
            sum += static_cast<double>(a[i]) * b[i];
        return sum;
    }

    // Renders a mono tone through the spatializer from a head-relative direction and
    // returns the settled per-ear signals.
    void render_direction(BinauralSpatializer& spatializer, int n, int blocks, float dx, float dy,
                          float dz, std::vector<float>& left, std::vector<float>& right)
    {
        std::vector<float> in(static_cast<std::size_t>(n));
        long long idx = 0;
        for (int b = 0; b < blocks; ++b)
        {
            for (int i = 0; i < n; ++i)
                in[static_cast<std::size_t>(i)] = static_cast<float>(std::sin(kTwoPi * 500.0 * idx++ / 48000.0));
            std::vector<float> l(static_cast<std::size_t>(n), 0.0f), r(static_cast<std::size_t>(n), 0.0f);
            spatializer.begin_block(n);
            spatializer.encode(in.data(), n, dx, dy, dz, 1.0f);
            spatializer.decode_binaural(l.data(), r.data(), n);
            if (b >= blocks / 2)
            {
                left.insert(left.end(), l.begin(), l.end());
                right.insert(right.end(), r.begin(), r.end());
            }
        }
    }
} // namespace

TEST(Unit_Audio, SphericalHarmonicsAcnSn3dConvention)
{
    const int order = 3;
    EXPECT_EQ(Dsp::ambisonic_channel_count(order), 16);

    float g[16];
    // W is always 1; first-order channels are the direction's y, z, x.
    Dsp::ambisonic_encode_gains(order, 1.0f, 0.0f, 0.0f, g); // front
    EXPECT_NEAR(g[0], 1.0f, 1e-4f);
    EXPECT_NEAR(g[1], 0.0f, 1e-4f); // y
    EXPECT_NEAR(g[2], 0.0f, 1e-4f); // z
    EXPECT_NEAR(g[3], 1.0f, 1e-4f); // x

    Dsp::ambisonic_encode_gains(order, 0.0f, 1.0f, 0.0f, g); // left
    EXPECT_NEAR(g[1], 1.0f, 1e-4f);
    EXPECT_NEAR(g[3], 0.0f, 1e-4f);

    Dsp::ambisonic_encode_gains(order, 0.0f, 0.0f, 1.0f, g); // up
    EXPECT_NEAR(g[2], 1.0f, 1e-4f);
}

TEST(Unit_Audio, SphericalHarmonicsAreOrthogonal)
{
    // Accumulate the Gram matrix over a near-uniform set of directions; off-diagonals
    // must vanish and the l=1 diagonal must be 1/(2l+1) = 1/3 (SN3D).
    const int order = 3;
    const int channels = Dsp::ambisonic_channel_count(order);
    std::vector<double> gram(static_cast<std::size_t>(channels * channels), 0.0);
    const int samples = 4000;
    for (int k = 0; k < samples; ++k)
    {
        const double z = 2.0 * (k + 0.5) / samples - 1.0;      // uniform in cos
        const double phi = kTwoPi * ((k * 97) % samples) / samples;
        const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
        float g[16];
        Dsp::ambisonic_encode_gains(order, static_cast<float>(r * std::cos(phi)),
                                    static_cast<float>(r * std::sin(phi)), static_cast<float>(z), g);
        for (int i = 0; i < channels; ++i)
            for (int j = 0; j < channels; ++j)
                gram[static_cast<std::size_t>(i * channels + j)] += static_cast<double>(g[i]) * g[j];
    }
    double max_off = 0.0;
    for (int i = 0; i < channels; ++i)
        for (int j = 0; j < channels; ++j)
            if (i != j)
                max_off = std::max(max_off, std::fabs(gram[static_cast<std::size_t>(i * channels + j)]) / samples);
    EXPECT_LT(max_off, 0.05);
    EXPECT_NEAR(gram[0] / samples, 1.0, 0.05);                       // W
    EXPECT_NEAR(gram[static_cast<std::size_t>(3 * channels + 3)] / samples, 1.0 / 3.0, 0.05); // X
}

TEST(Unit_Audio, AmbisonicKernelPeaksAtSourceDirection)
{
    const int order = 3;
    float source[16], front[16], side[16], back[16];
    Dsp::ambisonic_encode_gains(order, 1.0f, 0.0f, 0.0f, source);
    Dsp::ambisonic_encode_gains(order, 1.0f, 0.0f, 0.0f, front);
    Dsp::ambisonic_encode_gains(order, 0.0f, 1.0f, 0.0f, side);
    Dsp::ambisonic_encode_gains(order, -1.0f, 0.0f, 0.0f, back);
    EXPECT_GT(sh_kernel(order, source, front), sh_kernel(order, source, side));
    EXPECT_GT(sh_kernel(order, source, side), sh_kernel(order, source, back));
}

TEST(Unit_Audio, HeadRelativeDirectionRotatesIntoListenerFrame)
{
    float x = 0.0f, y = 0.0f, z = 0.0f;
    // Facing +x: a +x source stays in front.
    head_relative_direction(1, 0, 0, 1, 0, 0, 0, 0, 1, x, y, z);
    EXPECT_NEAR(x, 1.0f, 1e-4f);
    EXPECT_NEAR(y, 0.0f, 1e-4f);

    // Facing +y (turned left): a +x source is now on the right (y = -1).
    head_relative_direction(1, 0, 0, 0, 1, 0, 0, 0, 1, x, y, z);
    EXPECT_NEAR(x, 0.0f, 1e-4f);
    EXPECT_NEAR(y, -1.0f, 1e-4f);
}

TEST(Unit_Audio, SpatializerFrontSourceIsSymmetric)
{
    BinauralSpatializer spatializer;
    spatializer.configure(3, 48000.0, 256);
    std::vector<float> left, right;
    render_direction(spatializer, 256, 20, 1.0f, 0.0f, 0.0f, left, right);
    const double l = rms(left), r = rms(right);
    EXPECT_GT(l, 0.05);
    EXPECT_NEAR(l, r, 0.05 * l); // dead ahead: the ears match
}

TEST(Unit_Audio, SpatializerLeftRightLevelCues)
{
    BinauralSpatializer spatializer;
    spatializer.configure(3, 48000.0, 256);

    std::vector<float> left, right;
    render_direction(spatializer, 256, 20, 0.0f, 1.0f, 0.0f, left, right); // source left
    EXPECT_GT(rms(left), rms(right) * 1.15);

    left.clear();
    right.clear();
    render_direction(spatializer, 256, 20, 0.0f, -1.0f, 0.0f, left, right); // source right
    EXPECT_GT(rms(right), rms(left) * 1.15);
}

TEST(Unit_Audio, SpatializerLeftSourceDelaysRightEar)
{
    BinauralSpatializer spatializer;
    spatializer.configure(3, 48000.0, 256);
    std::vector<float> left, right;
    render_direction(spatializer, 256, 20, 0.0f, 1.0f, 0.0f, left, right); // source on the left

    // Cross-correlate: the right ear should lag the left (positive best lag).
    int best_lag = 0;
    double best = -1e18;
    for (int lag = -40; lag <= 40; ++lag)
    {
        double sum = 0.0;
        int count = 0;
        for (int i = 64; i + 64 < static_cast<int>(left.size()); ++i)
        {
            const int j = i + lag;
            if (j >= 0 && j < static_cast<int>(right.size()))
            {
                sum += static_cast<double>(left[static_cast<std::size_t>(i)]) *
                       right[static_cast<std::size_t>(j)];
                ++count;
            }
        }
        if (count > 0)
        {
            sum /= count;
            if (sum > best)
            {
                best = sum;
                best_lag = lag;
            }
        }
    }
    EXPECT_GT(best_lag, 0); // right ear delayed relative to left
}

TEST(Unit_Audio, SpatializerSilentWhenNothingEncoded)
{
    BinauralSpatializer spatializer;
    spatializer.configure(3, 48000.0, 128);
    std::vector<float> l(128, 0.0f), r(128, 0.0f);
    spatializer.begin_block(128);
    // No encode this block.
    spatializer.decode_binaural(l.data(), r.data(), 128);
    EXPECT_EQ(rms(l), 0.0);
    EXPECT_EQ(rms(r), 0.0);
    EXPECT_FALSE(spatializer.has_content());
}

TEST(Unit_Audio, EngineHeadTrackingReaimsSource)
{
    AudioEngine engine(4, 4);
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.set_ambisonic_order(3);
    engine.voices().set_max_propagation_distance(50.0f);
    engine.prepare(48000.0, 256);

    VoiceDescriptor d;
    d.base_gain = 1.0f;
    d.priority = 1.0f;
    d.bus = master;
    d.spatial = true;
    d.model = DistanceModel::Inverse;
    d.min_distance = 1.0f;
    d.max_distance = 50.0f;
    d.propagation_delay = false;
    d.position = AudioVec3{4.0f, 0.0f, 0.0f}; // source dead ahead in the world
    const int voice = engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(500.0f, 1.0f)));

    auto measure = [&](AudioVec3 forward, double& lr, double& rr) {
        engine.voices().set_listener(ListenerState{AudioVec3{0, 0, 0}, forward, AudioVec3{0, 0, 1}});
        std::vector<float> left, right, l(256), r(256);
        float* ch[2] = {l.data(), r.data()};
        for (int b = 0; b < 30; ++b)
        {
            engine.render(ch, 2, 256);
            if (b >= 15)
            {
                left.insert(left.end(), l.begin(), l.end());
                right.insert(right.end(), r.begin(), r.end());
            }
        }
        lr = rms(left);
        rr = rms(right);
    };

    double fl = 0, fr = 0, tl = 0, tr = 0;
    measure(AudioVec3{1, 0, 0}, fl, fr);  // facing the source: symmetric
    measure(AudioVec3{0, 1, 0}, tl, tr);  // turned left: source now on the right
    EXPECT_NEAR(fl, fr, 0.10 * fl);
    (void)voice;
    EXPECT_GT(tr, tl * 1.1);
}
