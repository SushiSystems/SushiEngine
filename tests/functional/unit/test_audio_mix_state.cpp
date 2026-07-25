/**************************************************************************/
/* test_audio_mix_state.cpp                                             */
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

// Unit_Audio: state-based mixing — a snapshot recalled with transition 0 snaps the bus
// gains, and with a transition time cross-fades them there over subsequent blocks.

#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

TEST(Unit_Audio, MixSnapshotSnapAndTransition)
{
    const double sr = 48000.0;
    const int block = 256;
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    const int music = mixer.add_bus(master);
    const int ambience = mixer.add_bus(master);
    mixer.set_master(master);
    mixer.prepare(sr, block);

    MixStateSet states;
    MixSnapshot combat;
    combat.id = 1;
    combat.gains = {{music, 1.0f}, {ambience, 0.2f}};
    states.add(combat);

    // Snap (transition 0): the bus gains jump to the snapshot immediately.
    ASSERT_TRUE(states.transition_to(mixer, 1, 0.0, sr));
    EXPECT_EQ(states.active(), 1u);
    EXPECT_NEAR(mixer.bus_gain(music).target(), 1.0f, 1e-4f);
    EXPECT_NEAR(mixer.bus_gain(ambience).target(), 0.2f, 1e-4f);
    // A process step confirms the snapped value is already in effect.
    mixer.begin_block(block);
    mixer.process(block);
    EXPECT_NEAR(mixer.bus_gain(ambience).current(), 0.2f, 1e-3f);

    // A timed transition to a new state cross-fades rather than jumping.
    MixSnapshot explore;
    explore.id = 2;
    explore.gains = {{ambience, 1.0f}};
    states.add(explore);
    ASSERT_TRUE(states.transition_to(mixer, 2, 1.0, sr)); // 1 s ramp from 0.2 → 1.0
    // A couple of short blocks in: the gain has started moving but is nowhere near done.
    mixer.begin_block(block);
    mixer.process(block);
    const float mid = mixer.bus_gain(ambience).current();
    EXPECT_GT(mid, 0.2f);
    EXPECT_LT(mid, 0.9f);
    EXPECT_EQ(states.active(), 2u);

    // Unknown state id is rejected.
    EXPECT_FALSE(states.transition_to(mixer, 99, 0.0, sr));
}
