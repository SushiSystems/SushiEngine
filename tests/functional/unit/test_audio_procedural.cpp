/**************************************************************************/
/* test_audio_procedural.cpp                                            */
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

// Unit_Audio: the phase-S10 procedural sources — modal impact synthesis (a strike rings
// then decays to silence; a longer material rings longer; harder strikes are louder) and
// wind (louder as the speed rises; a given seed is bit-reproducible). Header-only maths.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    double render_energy(VoiceSource& src, double sr, int blocks, int block)
    {
        double e = 0.0;
        std::vector<float> buf(static_cast<std::size_t>(block));
        for (int b = 0; b < blocks; ++b)
        {
            src.render(buf.data(), block);
            for (float s : buf)
                e += static_cast<double>(s) * s;
        }
        (void)sr;
        return e;
    }
}

// A modal impact rings after the strike and eventually goes silent.
TEST(Unit_Audio, ModalImpactRingsAndDecays)
{
    Dsp::ModalResonatorBank bank;
    bank.set_material(1, 400.0f); // metal
    bank.prepare(48000.0);
    EXPECT_GT(bank.mode_count(), 0u);
    bank.strike(1.0f);
    EXPECT_TRUE(bank.is_ringing());

    double early = 0.0, late = 0.0;
    std::vector<float> buf(480);
    for (int b = 0; b < 20; ++b)
    {
        bank.process_block(buf.data(), 480);
        for (float s : buf)
        {
            if (b < 3)
                early += static_cast<double>(s) * s;
            else if (b >= 17)
                late += static_cast<double>(s) * s;
        }
    }
    EXPECT_GT(early, 0.0);
    EXPECT_LT(late, early); // decaying

    // Enough silence and even a long metal ring falls below the audibility threshold.
    for (int b = 0; b < 48000 / 480 * 4; ++b) // ~4 s
        bank.process_block(buf.data(), 480);
    EXPECT_FALSE(bank.is_ringing());
}

// A harder strike is louder; wood (short) rings less than metal (long).
TEST(Unit_Audio, ModalImpactEnergyScales)
{
    auto strike_energy = [](int material, float base, float energy) {
        ModalImpactSource src(material, base, energy);
        src.prepare(48000.0, 512);
        return render_energy(src, 48000.0, 40, 512);
    };
    const double soft = strike_energy(1, 400.0f, 0.3f);
    const double hard = strike_energy(1, 400.0f, 1.0f);
    EXPECT_GT(hard, soft);

    const double wood = strike_energy(0, 400.0f, 1.0f);
    const double metal = strike_energy(1, 400.0f, 1.0f);
    EXPECT_GT(metal, wood); // metal's longer ring carries more energy over the window
}

// Wind gets louder as the speed rises, and is deterministic for a fixed seed.
TEST(Unit_Audio, WindSpeedAndDeterminism)
{
    auto wind_rms = [](float speed) {
        WindSource wind(0xC0FFEEu);
        wind.prepare(48000.0, 512);
        wind.set_speed(speed);
        double e = 0.0;
        std::vector<float> buf(512);
        for (int b = 0; b < 30; ++b)
        {
            wind.render(buf.data(), 512);
            for (float s : buf)
                e += static_cast<double>(s) * s;
        }
        return std::sqrt(e / (30 * 512));
    };
    EXPECT_GT(wind_rms(1.0f), wind_rms(0.1f));

    // Same seed → identical output.
    WindSource a(42u), b(42u);
    a.prepare(48000.0, 256);
    b.prepare(48000.0, 256);
    a.set_speed(0.7f);
    b.set_speed(0.7f);
    std::vector<float> ba(256), bb(256);
    a.render(ba.data(), 256);
    b.render(bb.data(), 256);
    for (int i = 0; i < 256; ++i)
        EXPECT_FLOAT_EQ(ba[i], bb[i]);
}
