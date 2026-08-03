/**************************************************************************/
/* audio_panels.hpp                                                      */
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

#pragma once

/**
 * @file audio_panels.hpp
 * @brief The editor's audio authoring UI — the mixer, the profiler, and the per-entity
 *        Inspector sections for emitters, reverb zones, and the listener (S9 §11).
 *
 * Free functions drawn by the editor shell: two dockable windows (the Audio Mixer with
 * live bus faders + meters, and the Audio Profiler with voice counts, meters, and an
 * output scope) plus three Inspector sections the entity Inspector calls when the
 * selected entity carries the matching audio component. All read/write the world through
 * the `IWorldEditor` seam and hear their edits live through the @ref AudioEditorSystem.
 */

#include <SushiEngine/simulation/simulation.hpp>

#include "../core/editor_context.hpp"
#include "audio_editor_system.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Audio Mixer window: master + per-bus faders and live meters.
         * @param context Editor state (for the panel-open flag and console).
         * @param audio   The live audio system (bus gains, meters, device toggle).
         */
        void draw_audio_mixer_panel(EditorContext& context, AudioEditorSystem& audio);

        /**
         * @brief Draws the Audio Profiler window: voice counts, meters, and the output scope.
         * @param context Editor state (for the panel-open flag).
         * @param audio   The live audio system (its latest profiler snapshot).
         */
        void draw_audio_profiler_panel(EditorContext& context, AudioEditorSystem& audio);

        /**
         * @brief Inspector section for an entity's Audio Emitter (drawn if it has one).
         * @param context Editor state (undo history).
         * @param world   The world to read/write the emitter parameters on.
         * @param id      The selected entity.
         * @param audio   The live audio system (for the Play audition).
         */
        void draw_audio_emitter_inspector(EditorContext& context, Simulation::IWorldEditor& world,
                                          Simulation::EntityId id, AudioEditorSystem& audio);

        /**
         * @brief Inspector section for an entity's Reverb Zone (drawn if it has one).
         * @param context Editor state (undo history).
         * @param world   The world to read/write the zone parameters on.
         * @param id      The selected entity.
         */
        void draw_reverb_zone_inspector(EditorContext& context, Simulation::IWorldEditor& world,
                                        Simulation::EntityId id);

        /**
         * @brief Inspector section for an entity's Audio Listener (drawn if it has one).
         * @param context Editor state (undo history).
         * @param world   The world to read/write the listener parameters on.
         * @param id      The selected entity.
         */
        void draw_audio_listener_inspector(EditorContext& context, Simulation::IWorldEditor& world,
                                           Simulation::EntityId id);
    } // namespace Editor
} // namespace SushiEngine
