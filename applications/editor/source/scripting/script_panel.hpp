/**************************************************************************/
/* script_panel.hpp                                                       */
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
 * @file script_panel.hpp
 * @brief Scripts in the Inspector: their fields, and creating a new one.
 *
 * A domain of its own rather than a section of the Inspector because it owns rules the
 * Inspector has no business knowing — what a valid script type name is, what a fresh
 * script's source looks like, and how a definition is registered with the simulation so
 * its fields can be reflected at all.
 */

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Adds a script's type to the Add Component catalog if it is not there yet.
         *
         * The Inspector's Add Component flow attaches a script by type name; the catalog is
         * what makes that type offerable again on the next entity, so registering it is part
         * of attaching it.
         *
         * @param context Editor state; edits the script-definition catalog.
         * @param script The freshly attached component whose type is being registered.
         */
        void register_script_definition(EditorContext& context,
                                        const Simulation::ScriptComponent& script);

        /**
         * @brief Draws the reflected fields of one script component.
         *
         * Reads the registered definition for the script's type and emits a widget per
         * declared field, each bracketed as its own undo step.
         *
         * @param context Editor state; drives undo and reads the script definitions.
         * @param world The world holding the component.
         * @param script The script component to edit in place.
         * @return True if any field changed this frame.
         */
        bool draw_script_fields(EditorContext& context, Simulation::IWorldEditor& world,
                                Simulation::ScriptComponent& script);

        /**
         * @brief Draws the "New Script" modal when the Inspector has requested one.
         *
         * Validates the type name, writes a stub source file into the project, registers
         * the definition, and opens the new file in the Text Editor — so a script created
         * here is immediately both editable and attachable.
         *
         * @param context Editor state; owns the modal's flag and the typed name.
         */
        void draw_new_script_modal(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
