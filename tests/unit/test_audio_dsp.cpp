/**************************************************************************/
/* test_audio_dsp.cpp                                                    */
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

// Unit_Audio: the phase-S1 DSP core in isolation — the lock-free SPSC ring, the
// flush-to-zero denormal guard, the SIMD sample kernels, the filter set (one-pole,
// RBJ biquad, Cytomic TPT-SVF), and the block graph including its one-block feedback
// semantics. Pure header-only maths against the real code, no device and no mocks.

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/dsp/dsp.hpp>

using namespace SushiEngine::Audio::Dsp;

namespace
{
    double rms(const std::vector<float>& x, int start = 0)
    {
        double sum = 0.0;
        int n = 0;
        for (std::size_t i = static_cast<std::size_t>(start); i < x.size(); ++i)
        {
            sum += static_cast<double>(x[i]) * static_cast<double>(x[i]);
            ++n;
        }
        return n > 0 ? std::sqrt(sum / n) : 0.0;
    }

    // Feeds a sine of `freq` through one filter-sample callable and returns settled RMS.
    template <typename ProcessOneSample>
    double sine_through(ProcessOneSample process, double freq, double sample_rate, int frames)
    {
        std::vector<float> out(static_cast<std::size_t>(frames));
        const double two_pi = 6.28318530717958647692;
        for (int i = 0; i < frames; ++i)
        {
            const float x = static_cast<float>(std::sin(two_pi * freq * i / sample_rate));
            out[static_cast<std::size_t>(i)] = process(x);
        }
        return rms(out, frames / 2); // settle: measure the second half
    }
} // namespace

TEST(Unit_Audio, SpscRingFifoOrderAndFullEmpty)
{
    SpscRing<int> ring(4); // rounds to a power of two >= 4
    EXPECT_GE(ring.capacity(), 4u);

    int v = -1;
    EXPECT_FALSE(ring.pop(v)); // empty

    for (int i = 0; i < static_cast<int>(ring.capacity()); ++i)
        EXPECT_TRUE(ring.push(i));
    EXPECT_FALSE(ring.push(999)); // full: drop

    for (int i = 0; i < static_cast<int>(ring.capacity()); ++i)
    {
        EXPECT_TRUE(ring.pop(v));
        EXPECT_EQ(v, i); // FIFO order
    }
    EXPECT_FALSE(ring.pop(v)); // empty again
}

TEST(Unit_Audio, SpscRingWrapsAroundManyTimes)
{
    SpscRing<int> ring(8);
    int v = 0;
    // Push/pop far more than capacity to exercise index wrap.
    for (int i = 0; i < 10000; ++i)
    {
        ASSERT_TRUE(ring.push(i));
        ASSERT_TRUE(ring.pop(v));
        EXPECT_EQ(v, i);
    }
    EXPECT_EQ(ring.size_approx(), 0u);
}

TEST(Unit_Audio, ScopedNoDenormalsFlushesToZero)
{
    if (!ScopedNoDenormals::is_supported())
    {
        SUCCEED() << "denormal control not available on this target";
        return;
    }
    ScopedNoDenormals guard;
    volatile float x = 1e-30f;
    x = x * 1e-12f; // 1e-42: a denormal, flushed to zero under FTZ
    EXPECT_EQ(x, 0.0f);
}

TEST(Unit_Audio, OnePoleLowPassHasUnityDcGain)
{
    OnePole filter;
    filter.set_low_pass(1000.0, 48000.0);
    filter.reset();
    float y = 0.0f;
    for (int i = 0; i < 20000; ++i)
        y = filter.process_low_pass(1.0f); // constant (DC) input
    EXPECT_NEAR(y, 1.0f, 1e-3f); // converges to the input level

    // High-pass of a DC input settles to zero.
    filter.reset();
    float hp = 0.0f;
    for (int i = 0; i < 20000; ++i)
        hp = filter.process_high_pass(1.0f);
    EXPECT_NEAR(hp, 0.0f, 1e-3f);
}

TEST(Unit_Audio, BiquadLowPassPassesLowAttenuatesHigh)
{
    const double sr = 48000.0;
    Biquad low;
    low.set_low_pass(500.0, 0.707, sr);
    low.reset();
    const double low_rms = sine_through([&](float x) { return low.process(x); }, 50.0, sr, 8192);

    Biquad high;
    high.set_low_pass(500.0, 0.707, sr);
    high.reset();
    const double high_rms = sine_through([&](float x) { return high.process(x); }, 10000.0, sr, 8192);

    // A unit sine has RMS ~0.707; 50 Hz passes a 500 Hz low-pass nearly untouched,
    // 10 kHz is deep in the stopband.
    EXPECT_NEAR(low_rms, 0.707, 0.05);
    EXPECT_LT(high_rms, 0.05);
    EXPECT_LT(high_rms, low_rms * 0.1);
}

TEST(Unit_Audio, BiquadHighPassMirrorsLowPass)
{
    const double sr = 48000.0;
    Biquad hp;
    hp.set_high_pass(500.0, 0.707, sr);
    hp.reset();
    const double low_rms = sine_through([&](float x) { return hp.process(x); }, 50.0, sr, 8192);
    hp.set_high_pass(500.0, 0.707, sr);
    hp.reset();
    const double high_rms = sine_through([&](float x) { return hp.process(x); }, 10000.0, sr, 8192);
    EXPECT_LT(low_rms, 0.05);          // low tone rejected
    EXPECT_NEAR(high_rms, 0.707, 0.05); // high tone passes
}

TEST(Unit_Audio, StateVariableFilterSplitsBands)
{
    const double sr = 48000.0;
    StateVariableFilter svf;
    svf.set(1000.0, 0.707, sr);
    svf.reset();

    // DC: energy exits the low-pass, not the high-pass.
    float lp = 0.0f, hp = 0.0f;
    for (int i = 0; i < 20000; ++i)
    {
        svf.process(1.0f);
        lp = svf.low_pass();
        hp = svf.high_pass();
    }
    EXPECT_NEAR(lp, 1.0f, 1e-2f);
    EXPECT_NEAR(hp, 0.0f, 1e-2f);

    // A high sine comes out of the high-pass, not the low-pass.
    svf.set(1000.0, 0.707, sr);
    svf.reset();
    std::vector<float> lp_out, hp_out;
    const double two_pi = 6.28318530717958647692;
    for (int i = 0; i < 8192; ++i)
    {
        svf.process(static_cast<float>(std::sin(two_pi * 12000.0 * i / sr)));
        lp_out.push_back(svf.low_pass());
        hp_out.push_back(svf.high_pass());
    }
    EXPECT_LT(rms(lp_out, 4096), 0.1);
    EXPECT_GT(rms(hp_out, 4096), 0.5);
}

TEST(Unit_Audio, SimdGainAndMixMatchScalarIncludingRemainder)
{
    const int n = 13; // deliberately not a multiple of 4 to hit the scalar tail
    std::vector<float> buf(n), ref(n);
    for (int i = 0; i < n; ++i)
        buf[i] = ref[i] = static_cast<float>(i) - 6.0f;

    Simd::apply_gain(buf.data(), n, 0.5f);
    for (int i = 0; i < n; ++i)
        EXPECT_FLOAT_EQ(buf[i], ref[i] * 0.5f);

    std::vector<float> dst(n, 1.0f), src(n);
    for (int i = 0; i < n; ++i)
        src[i] = static_cast<float>(i);
    Simd::mix_accumulate(dst.data(), src.data(), n, 2.0f);
    for (int i = 0; i < n; ++i)
        EXPECT_FLOAT_EQ(dst[i], 1.0f + static_cast<float>(i) * 2.0f);

    std::vector<float> ramp(4, 1.0f);
    Simd::apply_gain_ramp(ramp.data(), 4, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(ramp[0], 0.0f);
    EXPECT_FLOAT_EQ(ramp[3], 1.0f);
}

TEST(Unit_Audio, EqualPowerPanKeepsConstantPower)
{
    float l = 0.0f, r = 0.0f;
    Simd::equal_power_pan(0.0f, l, r); // centre
    EXPECT_NEAR(l, 0.70710678f, 1e-4f);
    EXPECT_NEAR(r, 0.70710678f, 1e-4f);

    Simd::equal_power_pan(-1.0f, l, r); // hard left
    EXPECT_NEAR(l, 1.0f, 1e-4f);
    EXPECT_NEAR(r, 0.0f, 1e-4f);

    Simd::equal_power_pan(1.0f, l, r); // hard right
    EXPECT_NEAR(l, 0.0f, 1e-4f);
    EXPECT_NEAR(r, 1.0f, 1e-4f);

    for (float p = -1.0f; p <= 1.0f; p += 0.1f)
    {
        Simd::equal_power_pan(p, l, r);
        EXPECT_NEAR(l * l + r * r, 1.0f, 1e-4f); // constant power
    }
}

TEST(Unit_Audio, BlockGraphMixesSourcesInTopologicalOrder)
{
    BlockGraph graph;
    const NodeId a = graph.add_node(std::unique_ptr<Node>(new SineNode(100.0f, 0.5f)));
    const NodeId b = graph.add_node(std::unique_ptr<Node>(new SineNode(200.0f, 0.5f)));
    const NodeId mix = graph.add_node(std::unique_ptr<Node>(new MixNode(2)));
    const NodeId gain = graph.add_node(std::unique_ptr<Node>(new GainNode(2.0f)));
    graph.connect(a, 0, mix, 0);
    graph.connect(b, 0, mix, 1);
    graph.connect(mix, 0, gain, 0);
    graph.set_graph_output(gain, 0);
    graph.prepare(48000.0, 256);

    // Sources precede the mix, which precedes the gain.
    const std::vector<NodeId>& order = graph.order();
    ASSERT_EQ(order.size(), 4u);
    auto position = [&](NodeId id) {
        for (std::size_t i = 0; i < order.size(); ++i)
            if (order[i] == id)
                return static_cast<int>(i);
        return -1;
    };
    EXPECT_LT(position(a), position(mix));
    EXPECT_LT(position(b), position(mix));
    EXPECT_LT(position(mix), position(gain));

    // Run a few blocks; the summed, gained output must stay bounded and be non-silent.
    double peak = 0.0, energy = 0.0;
    for (int block = 0; block < 8; ++block)
    {
        graph.process(256);
        const float* out = graph.output_buffer();
        ASSERT_NE(out, nullptr);
        for (int i = 0; i < 256; ++i)
        {
            const double a_abs = std::fabs(static_cast<double>(out[i]));
            peak = std::max(peak, a_abs);
            energy += a_abs;
        }
    }
    EXPECT_GT(energy, 0.0);   // not silent
    EXPECT_LE(peak, 2.001);   // 2 * (0.5 + 0.5) worst case
}

// A constant source and a self-feeding adder, defined here, pin the one-block
// feedback semantics of the graph precisely.
namespace
{
    class ConstNode final : public Node
    {
        public:
            explicit ConstNode(float value) noexcept : Node(0, 1), value_(value) {}
            void process(const float* const*, float* const* outputs, int frame_count) noexcept override
            {
                for (int i = 0; i < frame_count; ++i)
                    outputs[0][i] = value_;
            }
        private:
            float value_;
    };

    // out = in0 + coefficient * in1, where in1 is a feedback tap of this node's output.
    class FeedbackAdder final : public Node
    {
        public:
            explicit FeedbackAdder(float coefficient) noexcept : Node(2, 1), coefficient_(coefficient) {}
            void process(const float* const* inputs, float* const* outputs, int frame_count) noexcept override
            {
                for (int i = 0; i < frame_count; ++i)
                    outputs[0][i] = inputs[0][i] + coefficient_ * inputs[1][i];
            }
        private:
            float coefficient_;
    };
} // namespace

TEST(Unit_Audio, BlockGraphFeedbackEdgeIsOneBlockDelay)
{
    BlockGraph graph;
    const NodeId source = graph.add_node(std::unique_ptr<Node>(new ConstNode(1.0f)));
    const NodeId adder = graph.add_node(std::unique_ptr<Node>(new FeedbackAdder(0.5f)));
    graph.connect(source, 0, adder, 0);
    graph.connect(adder, 0, adder, 1, /*feedback=*/true); // one-block self-feedback
    graph.set_graph_output(adder, 0);
    graph.prepare(48000.0, 64);

    // v_k = 1 + 0.5 * v_{k-1}, v_0 read against a zero-initialized buffer.
    // Sequence: 1.0, 1.5, 1.75, 1.875, ... converging to 2.0.
    const float expected[5] = {1.0f, 1.5f, 1.75f, 1.875f, 1.9375f};
    for (int block = 0; block < 5; ++block)
    {
        graph.process(64);
        const float* out = graph.output_buffer();
        ASSERT_NE(out, nullptr);
        EXPECT_NEAR(out[0], expected[block], 1e-5f);
        EXPECT_NEAR(out[63], expected[block], 1e-5f); // whole block latched together
    }
}
