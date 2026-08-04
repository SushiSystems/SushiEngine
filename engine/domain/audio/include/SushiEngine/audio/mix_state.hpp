/**************************************************************************/
/* mix_state.hpp                                                          */
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

#ifndef SUSHIENGINE_AUDIO_MIX_STATE_HPP
#define SUSHIENGINE_AUDIO_MIX_STATE_HPP

/**
 * @file mix_state.hpp
 * @brief State-based mixing — named mixer snapshots the game recalls with a transition.
 *
 * A game changes its whole mix by *state*: "combat" pushes music and lowers ambience,
 * "underwater" ducks everything and darkens it, "paused" dims the world under the menu
 * (§8 of `docs/design/audio_system.md`). Each state is a **snapshot** — a set of target bus
 * gains — and switching state cross-fades every bus toward the new snapshot over a
 * transition time. This is the Wwise States / FMOD snapshot mechanism.
 *
 * A snapshot only names the buses it changes; buses it omits are left alone. Applying it
 * configures each named bus's post-fader @ref SmoothedValue with the transition time and
 * sets its target, so the mixer slews there over the next blocks — no clicks, no per-frame
 * driving. Portable, header-only; drives the existing @ref MixerGraph.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/audio/mixer.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief One bus's target gain within a @ref MixSnapshot. */
        struct MixBusGain
        {
            int bus = 0;        /**< Target bus id. */
            float gain = 1.0f;  /**< Linear target gain. */
        };

        /** @brief A named mixer state: the bus gains it imposes. */
        struct MixSnapshot
        {
            std::uint32_t id = 0;            /**< The state handle the game recalls. */
            std::vector<MixBusGain> gains;   /**< Buses this state changes (others untouched). */
        };

        /**
         * @brief A set of mixer snapshots with a cross-fading recall.
         *
         * Register snapshots with @ref add, then @ref transition_to(id) to slew the mixer
         * to that state over a transition. The active state is remembered so a UI can show
         * it. Only the buses a snapshot names move; the rest hold.
         */
        class MixStateSet
        {
            public:
                /** @brief Adds or replaces a snapshot by its id. */
                void add(const MixSnapshot& snapshot)
                {
                    for (MixSnapshot& s : snapshots_)
                    {
                        if (s.id == snapshot.id)
                        {
                            s = snapshot;
                            return;
                        }
                    }
                    snapshots_.push_back(snapshot);
                }

                /** @brief The id of the most recently applied state. */
                std::uint32_t active() const noexcept { return active_; }

                /** @brief The number of registered snapshots. */
                std::size_t count() const noexcept { return snapshots_.size(); }

                /**
                 * @brief Cross-fades the mixer to a snapshot over @p transition_seconds.
                 *
                 * Configures each named bus's fader slew to the transition time and sets its
                 * target gain; the mixer slews there as it processes subsequent blocks. A
                 * transition of 0 snaps immediately.
                 *
                 * @param mixer              The mixer to drive.
                 * @param id                 The snapshot to recall.
                 * @param transition_seconds The cross-fade time.
                 * @param sample_rate        The stream sample rate (for the slew rate).
                 * @return True if the snapshot existed and was applied.
                 */
                bool transition_to(MixerGraph& mixer, std::uint32_t id, double transition_seconds,
                                   double sample_rate)
                {
                    const MixSnapshot* snapshot = nullptr;
                    for (const MixSnapshot& s : snapshots_)
                        if (s.id == id)
                        {
                            snapshot = &s;
                            break;
                        }
                    if (snapshot == nullptr)
                        return false;

                    for (const MixBusGain& bg : snapshot->gains)
                    {
                        if (bg.bus < 0 || bg.bus >= mixer.bus_count())
                            continue;
                        SmoothedValue& gain = mixer.bus_gain(bg.bus);
                        if (transition_seconds <= 0.0)
                        {
                            gain.snap(bg.gain);
                        }
                        else
                        {
                            gain.configure(transition_seconds, sample_rate);
                            gain.set_target(bg.gain);
                        }
                    }
                    active_ = id;
                    return true;
                }

            private:
                std::vector<MixSnapshot> snapshots_;
                std::uint32_t active_ = 0;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
