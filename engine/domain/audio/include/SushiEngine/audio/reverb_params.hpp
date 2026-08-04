/**************************************************************************/
/* reverb_params.hpp                                                      */
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

#ifndef SUSHIENGINE_AUDIO_REVERB_PARAMS_HPP
#define SUSHIENGINE_AUDIO_REVERB_PARAMS_HPP

/**
 * @file reverb_params.hpp
 * @brief The I3DL2 reverb parameter set — a dependency-free POD, split out so it can be
 *        an ECS component field.
 *
 * This is deliberately the *lightest possible* header: the @ref I3DL2Reverb struct and
 * its presets, nothing else (only `<cmath>` for the preset maths). The reverb
 * **algorithm** (the FDN, the `IReverb` seam, the mixer adapter) lives in `reverb.hpp`
 * and pulls in the DSP core; a `ReverbZone` ECS component (`sim/components.hpp`) needs
 * only the *data*, so it includes this and not the engine. Keeping the parameter POD
 * free of the DSP is the Interface-Segregation cut between "what a designer authors"
 * and "what renders it". See `docs/design/audio_system.md` §7.
 *
 * The struct is a plain aggregate of `float`s, so it is trivially copyable — the ECS
 * component requirement (`component_id<T>()`).
 */

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief The I3DL2 / EAX reverb parameter set — the public reverb vocabulary.
         *
         * Levels are in **decibels** (I3DL2's native millibels ÷ 100; 0 dB = full,
         * negative attenuates), times in **seconds**, and Diffusion / Density / WetDryMix
         * in **percent [0, 100]**. The defaults are a neutral small-hall.
         */
        struct I3DL2Reverb
        {
            float room = -6.0f;            ///< Overall reverb level (dB, ≤ 0).
            float room_hf = -3.0f;         ///< HF reverb level relative to @ref room (dB, ≤ 0).
            float decay_time = 1.5f;       ///< Broadband RT60 in seconds (0.1 .. 20).
            float decay_hf_ratio = 0.5f;   ///< `RT60_hf / RT60_dc` (0.1 .. 2); < 1 → darker tail.
            float reflections = -12.0f;    ///< Early-reflection level (dB) — rendered from S7.
            float reflections_delay = 0.01f; ///< Early-reflection onset in seconds — S7.
            float reverb = 0.0f;           ///< Late-reverb level (dB, ≤ 0).
            float reverb_delay = 0.02f;    ///< Late-reverb predelay in seconds (ReverbDelay).
            float diffusion = 100.0f;      ///< Echo-buildup density in percent [0, 100].
            float density = 100.0f;        ///< Modal density in percent [0, 100].
            float hf_reference = 5000.0f;  ///< Reference frequency for the HF controls (Hz).
            float wet_dry_mix = 100.0f;    ///< Wet percent [0, 100]; 100 = fully wet (aux use).

            /** @brief A neutral, general-purpose small space. */
            static I3DL2Reverb generic() { return I3DL2Reverb{}; }

            /** @brief A small, fairly dead room. */
            static I3DL2Reverb room_small()
            {
                I3DL2Reverb r;
                r.room = -10.0f; r.room_hf = -6.0f;
                r.decay_time = 0.5f; r.decay_hf_ratio = 0.6f;
                r.reverb_delay = 0.007f;
                return r;
            }

            /** @brief A large, bright concert hall. */
            static I3DL2Reverb concert_hall()
            {
                I3DL2Reverb r;
                r.room = -4.0f; r.room_hf = -2.0f;
                r.decay_time = 2.9f; r.decay_hf_ratio = 0.7f;
                r.reverb_delay = 0.035f;
                return r;
            }

            /** @brief A long, dark cave with a slow, muffled tail. */
            static I3DL2Reverb cave()
            {
                I3DL2Reverb r;
                r.room = -2.0f; r.room_hf = -10.0f;
                r.decay_time = 4.5f; r.decay_hf_ratio = 0.4f;
                r.reverb_delay = 0.05f; r.diffusion = 100.0f; r.density = 100.0f;
                return r;
            }
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
