/**************************************************************************/
/* panel_state.hpp                                                        */
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

#pragma once

/**
 * @file panel_state.hpp
 * @brief The scratch state panels keep between frames, owned in one place.
 *
 * A panel that remembers a checkbox, a sample grid, or a half-finished rebind needs
 * that state to outlive the frame, and the cheapest way to get it is a function-local
 * static. That is also the worst way: process-global, one copy no matter how many
 * editors or views exist, invisible to anything that inspects editor state, and
 * impossible to reset when a scene is replaced. Holding it here instead gives each
 * panel a named home reachable through @ref EditorContext.
 *
 * Only dependency-free scratch lives here — nothing that would drag a subsystem's
 * headers into the editor's core. The animation panels' authoring state is larger and
 * speaks in animation types, so those keep their own structs beside their panels and
 * the main loop owns the instances, the same way it owns the preview objects.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/input/rebinding.hpp>

namespace SushiEngine
{
    namespace Input
    {
        class InputContext;
    } // namespace Input

    namespace Editor
    {
        /**
         * @brief How much of the weather being asked for is actually being simulated.
         *
         * A *recent* ratio and not a cumulative one, which matters because this number's
         * whole purpose is to be acted on: a session that ran an hour at the wrong rate and
         * then corrected it would carry that hour in a running total forever, so the panel
         * would keep reporting a problem that had been fixed and the button that fixes it
         * would over-correct every time it was pressed.
         *
         * Anchored rather than sampled per frame because the mirror is asynchronous — a
         * readback lands every few frames, so a frame-to-frame difference is mostly the
         * readback's own cadence. The gate is in *game* seconds for the same reason the nest
         * steps in them: it makes the window a fixed amount of weather rather than a fixed
         * amount of wall clock.
         */
        struct ClockLag
        {
            double anchor_asked = 0.0;
            double anchor_simulated = 0.0;
            double ratio = 1.0;
            bool primed = false;

            /**
             * @brief Fold one observation in.
             * @param asked     Game seconds the simulation has accumulated.
             * @param simulated Game seconds the nest has actually stepped.
             * @return The smoothed ratio, 1 when nothing is being dropped.
             */
            double update(double asked, double simulated)
            {
                if (!primed || asked < anchor_asked || simulated < anchor_simulated)
                {
                    anchor_asked = asked;
                    anchor_simulated = simulated;
                    primed = true;
                    return ratio;
                }
                const double demanded = asked - anchor_asked;
                const double served = simulated - anchor_simulated;
                // A window of a simulated minute: long enough that the readback's own
                // latency is not the measurement, short enough to follow a rate change.
                if (demanded < 60.0)
                    return ratio;
                const double instant = served > 1.0 ? demanded / served : demanded;
                // Smoothed, because the frame rate this ultimately measures is itself noisy
                // and a readout that jumps is one nobody can act on.
                ratio = ratio * 0.7 + instant * 0.3;
                anchor_asked = asked;
                anchor_simulated = simulated;
                return ratio;
            }
        };

        /** @brief The Meteorology panel's between-frame state. */
        struct MeteorologyPanelState
        {
            ClockLag clock_lag;         /**< The asked-versus-simulated ratio, smoothed. */
            bool whole_domain = false;  /**< Profile the full column, not just the lowest 6 km. */
        };

        /** @brief The Environment panel's weather-map and wind-injection controls. */
        struct WeatherMapState
        {
            /** Number of cells per side of the sampled map; its grid is `cells * cells`. */
            static constexpr int CELLS = 44;

            int field = 0;                    /**< Which weather field the map shows. */
            float span_degrees = 45.0f;       /**< Half-width of the sampled region. */
            std::vector<float> samples;       /**< `CELLS * CELLS` sampled values. */

            float inject_radius_km = 700.0f;   /**< Radius of an injected wind anomaly. */
            float inject_amplitude_mps = 12.0f;/**< Its peak speed. */
            int inject_sign = 0;               /**< Cyclonic or anticyclonic. */
        };

        /** @brief The particle panel's effect-library browser state. */
        struct EffectLibraryState
        {
            std::vector<std::string> files; /**< Effect files found under the project. */
            bool scanned = false;           /**< Whether the scan has run this session. */
            std::string status;             /**< Result of the last load or save. */
            char name_buffer[64] = "Effect";/**< Filename for the next save. */
        };

        /**
         * @brief The Input Manager's in-progress rebind.
         *
         * `listener` is armed while the window waits for a key; `action` and `context` name
         * what the captured control will be written to. All three are cleared together, so a
         * cancelled or timed-out capture cannot leave a half-armed rebind behind.
         */
        struct RebindState
        {
            Input::RebindingListener listener;
            std::string action;
            Input::InputContext* context = nullptr;
        };

        /**
         * @brief Every panel's between-frame scratch, held by the editor context.
         *
         * One aggregate rather than loose members so a panel's state is found by name and a
         * future "reset the editor's transient state" has exactly one thing to clear.
         */
        struct PanelState
        {
            MeteorologyPanelState meteorology;
            WeatherMapState weather_map;
            EffectLibraryState effect_library;
            RebindState rebind;

            /** The rigged asset the Animator panel's Load Character field holds. */
            std::string character_path = "examples/assets/rigged_arm_anim.gltf";
        };
    } // namespace Editor
} // namespace SushiEngine
