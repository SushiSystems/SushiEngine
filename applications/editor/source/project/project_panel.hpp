/**************************************************************************/
/* project_panel.hpp                                                      */
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
 * @file project_panel.hpp
 * @brief The project's files: browsing them, and the documents they open into.
 *
 * The Project browser and the Text Editor share a translation unit because they are
 * two views of one thing — the browser decides what a double-click means (a scene, a
 * rigged character for the preview, a text document, or a hand-off to the shell) and
 * the editor hosts whatever it opened. The open/save document commands are exported
 * because the menu bar's Save and the new-script flow reach the same document store.
 *
 * This is also where the platform shell calls live (`ShellExecuteW`, Explorer), which
 * is why `<windows.h>` enters the editor here and nowhere else.
 */

#include <filesystem>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Project window: a filesystem browser rooted at the project.
         *
         * A folder tree beside a tile grid, with create/rename/delete, a name filter, and
         * the shell hand-offs. Double-clicking a directory descends; a file opens by kind.
         *
         * @param context Editor state; updates the browse directory, selection, documents.
         */
        void draw_project_panel(EditorContext& context);

        /**
         * @brief Draws the Cooking Override modal when @ref EditorContext::cooking_override_target
         * names an asset.
         *
         * §8.1's per-asset half of the import profile — the editor surface over
         * `ImportProfileOverride`. Called unconditionally every frame, matching
         * `draw_save_scene_as_modal`'s own convention: it is a no-op whenever the target is
         * empty.
         *
         * @param context Editor state; reads and clears @ref EditorContext::cooking_override_target.
         */
        void draw_cooking_override_modal(EditorContext& context);

        /**
         * @brief Draws the Text Editor window hosting the open documents as tabs.
         * @param context Editor state; edits document buffers and saves them to disk.
         */
        void draw_text_editor_panel(EditorContext& context);

        /**
         * @brief Opens a text file as a document tab, focusing it if it is already open.
         *
         * Exported because the new-script flow creates a file and wants it on screen
         * immediately, which must land in the same document list the Text Editor draws.
         *
         * @param context Editor state; appends to @ref EditorContext::documents.
         * @param path The file to read.
         */
        void open_document(EditorContext& context, const std::filesystem::path& path);

        /**
         * @brief Writes a document's buffer back to its path and marks it clean.
         *
         * Exported for the menu bar's File ▸ Save, so the menu and the Text Editor's own
         * save write the same bytes and agree on when the document stops being dirty.
         *
         * @param document The document to persist; its dirty flag is cleared on success.
         */
        void save_document(Document& document);
    } // namespace Editor
} // namespace SushiEngine
