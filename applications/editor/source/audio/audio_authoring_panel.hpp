/**************************************************************************/
/* audio_authoring_panel.hpp                                              */
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

#ifndef SUSHIENGINE_EDITOR_AUDIO_AUTHORING_PANEL_HPP
#define SUSHIENGINE_EDITOR_AUDIO_AUTHORING_PANEL_HPP

/**
 * @file audio_authoring_panel.hpp
 * @brief The sound-designer's authoring DAW panel — a view over @ref AudioAuthoringProject.
 *
 * The ImGui surface for the mutable audio project (`audio/authoring.hpp`): browse the media
 * table, grow and re-weight the container graph (Sound / Random / Sequence / Blend / Switch /
 * Layer), root events, and audition any sound through the live @ref AudioEditorSystem. A thin
 * view — all model state lives in the project, which bakes to a runtime bank.
 *
 * Deliberately self-contained: it takes the project, the audio system, and an open flag directly
 * rather than reaching through `EditorContext`, so it wires into the editor shell in three lines
 * without depending on the rest of the editor's evolving panel plumbing.
 */

namespace SushiEngine
{
    namespace Audio
    {
        class AudioAuthoringProject;
    }

    namespace Editor
    {
        class AudioEditorSystem;

        /**
         * @brief Draws the audio authoring panel.
         * @param project The mutable authoring project (edited in place).
         * @param audio   The live audio system used to audition sounds.
         * @param open    Panel visibility flag (cleared when the user closes the window).
         */
        void draw_audio_authoring_panel(Audio::AudioAuthoringProject& project,
                                        AudioEditorSystem& audio, bool* open);
    } // namespace Editor
} // namespace SushiEngine

#endif
