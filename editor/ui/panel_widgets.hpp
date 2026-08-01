/**************************************************************************/
/* panel_widgets.hpp                                                      */
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
 * @file panel_widgets.hpp
 * @brief The vocabulary every editor panel shares: undo-aware fields and conversions.
 *
 * The panels are one-per-domain translation units that must still agree on how an
 * edit becomes an undo step, how a `Scalar` reaches a float widget, and how an
 * inline rename behaves. Without a shared home each of those was hand-written per
 * panel — the same three-line undo bracket around thirty widgets, the same
 * `static_cast` pair around ninety, three rename fields with three private buffers —
 * and each copy was free to drift. This header is that home: the mechanism lives
 * once, and a panel spells its intent instead of its implementation.
 */

#include <cstring>
#include <string>
#include <type_traits>

#include <SushiEngine/render/environment.hpp>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The ImGui drag-and-drop payload type carrying a project-relative asset path.
         *
         * Named once here because a payload type is a contract between two panels that do
         * not otherwise know about each other — the Project browser writes it, the Inspector
         * and the Hierarchy read it — and a mistyped string on either side is a drop that
         * silently never lands.
         */
        constexpr const char* ASSET_PATH_PAYLOAD = "SUSHI_ASSET_PATH";

        /**
         * @brief Makes the item just drawn a drag source carrying @p path.
         *
         * @param path Absolute path of the asset being dragged; sent as a null-terminated
         *             payload so the receiver can use it directly.
         * @param label Text shown under the cursor while dragging.
         */
        void set_asset_drag_source(const std::string& path, const std::string& label);

        /**
         * @brief Accepts an asset-path drop on the item just drawn.
         *
         * @param out_path Receives the dropped path when a drop lands this frame.
         * @return True on the frame a drop was accepted; false otherwise, including while a
         *         drag merely hovers.
         */
        bool accept_asset_drop(std::string& out_path);

        /**
         * @brief The world's editor surface, or nullptr before the simulation is injected.
         *
         * Every panel that edits the scene begins by asking for this and returning early
         * when it is null — the editor draws its full window set from the first frame,
         * which is before `main` has handed it a simulation.
         *
         * @param context Shared editor state holding the simulation pointer.
         * @return The world editor, or nullptr when no simulation is attached yet.
         */
        Simulation::IWorldEditor* world_of(EditorContext& context);

        /**
         * @brief Folds the item just drawn into exactly one undo step for its whole gesture.
         *
         * Call immediately after the widget. A drag opens a pending change when the item is
         * activated and commits it when the item is released with an edit, so a fifty-frame
         * drag records one step instead of fifty; a click-and-commit widget records the same
         * single step through the same two edges.
         *
         * @param context Shared editor state; drives `context.history`.
         * @param world The world the snapshot is taken from.
         */
        void track_item_undo(EditorContext& context, Simulation::IWorldEditor& world);

        /**
         * @brief A labelled drag-float row for a 3-component vector, matching Unity's
         * X/Y/Z inspector rows.
         *
         * Lays itself out according to where it finds itself: inside an ImGui table it emits
         * a row (label column, field column); outside one it draws a plain labelled drag.
         * It used to open a table row unconditionally, which made calling it from a panel
         * that draws no table an access violation rather than a layout glitch — so the
         * widget carries the test instead of every caller carrying a precondition.
         *
         * Brackets the drag as one undo step via @ref track_item_undo.
         *
         * @param context Shared editor state; drives the undo bracket.
         * @param world The world the undo snapshot is taken from.
         * @param label Row label, also the ImGui id scope.
         * @param values The three components, edited in place.
         * @param speed Drag sensitivity in units per pixel.
         * @param mixed Whether the selected entities disagree about this row, in which case
         *              a dash is displayed instead of one entity's numbers.
         * @param format printf-style display format, carrying the unit suffix.
         * @param tooltip Explanation shown on hover, or nullptr for none.
         * @return True on any frame a component changed.
         */
        bool vector3_field(EditorContext& context, Simulation::IWorldEditor& world,
                           const char* label, float values[3], float speed, bool mixed = false,
                           const char* format = "%.3f", const char* tooltip = nullptr);

        /**
         * @brief A single labelled scalar row, for values that are not a homogeneous vector.
         *
         * The geodetic latitude/longitude/altitude of a Surface-frame position is the
         * motivating case: three components with different units and ranges, which a
         * `DragFloat3` cannot express.
         *
         * @param context Shared editor state; drives the undo bracket.
         * @param world The world the undo snapshot is taken from.
         * @param label Row label, also the ImGui id scope.
         * @param value The value, edited in place.
         * @param speed Drag sensitivity in units per pixel.
         * @param min_value Lower clamp; equal to @p max_value to leave it unbounded.
         * @param max_value Upper clamp.
         * @param format printf-style display format, carrying the unit suffix.
         * @param mixed Whether the selected entities disagree about this row; see
         *              @ref vector3_field.
         * @param tooltip Explanation shown on hover, or nullptr for none.
         * @return True on any frame the value changed.
         */
        bool scalar_field(EditorContext& context, Simulation::IWorldEditor& world,
                          const char* label, float* value, float speed, float min_value,
                          float max_value, const char* format, bool mixed = false,
                          const char* tooltip = nullptr);

        /**
         * @brief Draws an autofocused inline rename field, seeded once per target.
         *
         * The Hierarchy's tree rows, its filtered search rows, and the Project tiles all
         * rename this way: a field that takes keyboard focus on the frame the target
         * changes and commits on Enter or focus loss. The seed and the target key live in
         * @ref EditorContext, so the three call sites share one buffer instead of three
         * private statics that could each hold a different half-typed name.
         *
         * @param context Shared editor state; owns the rename buffer and target key.
         * @param target_key Opaque identity of the row being renamed (entity id or path);
         *                   a change against the stored key re-seeds the buffer.
         * @param seed The current name, used to fill the field when the target changes.
         * @param width Field width in pixels; negative values follow ImGui's convention
         *              (`-FLT_MIN` fills the available space).
         * @param out_text Receives the entered text on the frame the edit commits.
         * @return True on the single frame the rename commits; the caller then applies it
         *         and clears its own "renaming" marker.
         */
        bool inline_rename_field(EditorContext& context, const std::string& target_key,
                                 const std::string& seed, float width, std::string& out_text);

        /**
         * @brief A menu item whose shortcut hint comes from the live binding.
         *
         * Every menu that advertises a shortcut reads it from the binding table rather than
         * spelling it, so a rebind is reflected wherever the shortcut is claimed and a menu
         * cannot promise a chord nothing is bound to. An unbound action simply shows no hint.
         *
         * @param context Editor state; the global input context is read for the binding.
         * @param label The item's text.
         * @param action_name The action in the EditorGlobal context whose binding to show.
         * @param enabled Whether the item is selectable.
         * @return True on the frame the item is activated.
         */
        bool menu_item_for_action(EditorContext& context, const char* label,
                                  const char* action_name, bool enabled);

        /**
         * @brief What a component header asked for this frame.
         *
         * A component section is not just a collapsing header: it is a header, a remove
         * affordance, and a menu of whole-component operations. Returning them together lets
         * the header be drawn once, in one place, for every component — the eleven sections
         * that each spelled out their own `bool keep = true` and their own remove branch now
         * read the same way, and a twelfth cannot forget the menu.
         */
        struct ComponentSection
        {
            bool open = false;  /**< Whether the body should be drawn. */
            bool remove = false; /**< The header's "x", or Remove from the menu. */
            bool reset = false;  /**< Reset: put the component's declared defaults back. */
            bool copy = false;   /**< Copy Values: remember this component's values. */
            bool paste = false;  /**< Paste Values: install the remembered values. */
        };

        /**
         * @brief Draws a component section's header and its right-click menu.
         *
         * @param context Editor state; the value clipboard decides whether Paste is offered.
         * @param label The component's display name, which is also the clipboard's key.
         * @param value_actions Whether Reset and Copy/Paste Values are offered. A component
         *                      with no authored fields — Surface Anchor is one — has nothing
         *                      for them to carry, and a menu item that does nothing is worse
         *                      than a menu without it.
         * @return What the user asked for; at most one of the actions is set.
         */
        ComponentSection component_header(EditorContext& context, const char* label,
                                          bool value_actions = true);

        /**
         * @brief The shapes the toolbar draws instead of spelling.
         *
         * Playback and the transform tools are the controls a user reaches for without
         * reading, which is exactly what a shape does better than a word — and a shape
         * survives being three characters wide, which "Rotate" does not. These are *drawn*,
         * not glyphs from a font: an icon font would be a binary asset to ship, license and
         * keep in step with the build, for eight shapes that are a triangle, two bars and a
         * few lines.
         */
        enum class ToolbarIcon
        {
            Play,
            Stop,
            Pause,
            Step,
            Move,
            Rotate,
            Scale
        };

        /**
         * @brief A square button whose face is a drawn @ref ToolbarIcon.
         *
         * @param id ImGui id for the button; never displayed.
         * @param icon Which shape to paint on it.
         * @param active Whether to hold the button in its pressed colour, for the tool
         *               selector's current tool.
         * @param tooltip Hover text naming the action, since the shape does not; shown even
         *                while the button is disabled, which is when a user most wants it.
         * @return True on the frame the button is activated.
         */
        bool icon_button(const char* id, ToolbarIcon icon, bool active, const char* tooltip);

        /**
         * @brief An action's bound chord as display text, e.g. "Ctrl+P".
         *
         * The same live-binding read behind @ref menu_item_for_action, exposed for the
         * surfaces that state a shortcut somewhere other than a menu item — a toolbar
         * tooltip, a panel's empty-state hint — so those cannot drift into promising a chord
         * nothing is bound to either.
         *
         * @param context Editor state; the global input context is read for the binding.
         * @param action_name The action in the EditorGlobal context to look up.
         * @return The chord, or an empty string when the action is unbound or unknown.
         */
        std::string shortcut_for_action(EditorContext& context, const char* action_name);

        /**
         * @brief Case-folds a string for case-insensitive name matching.
         *
         * Every panel with a search box compares folded copies of both sides; ASCII
         * folding is enough because it matches asset and entity names, which the editor
         * spells in ASCII.
         *
         * @param text The string to fold.
         * @return A lower-cased copy.
         */
        std::string to_lower(const std::string& text);

        /**
         * @brief Converts a world-space quaternion to the Euler degrees the Inspector edits.
         *
         * A presentation concern, not part of the engine's math seam: the world stores
         * orientation as a quaternion and only the editor wants degrees.
         *
         * @param q The orientation to convert.
         * @param out Receives roll, pitch, yaw in degrees.
         */
        void quaternion_to_euler_degrees(const Quaternion& q, float out[3]);

        /**
         * @brief The inverse of @ref quaternion_to_euler_degrees, for writing an edit back.
         * @param in Roll, pitch, yaw in degrees.
         * @return The equivalent orientation.
         */
        Quaternion euler_degrees_to_quat(const float in[3]);

        /**
         * @brief Narrows a simulation `Scalar` to the float an ImGui widget takes.
         *
         * Named rather than spelled as a cast at every call site: `Scalar` is a double
         * throughout the boundary, so a panel touches this conversion on nearly every
         * field, and a named narrowing states that the loss of precision is the display's,
         * not the simulation's.
         *
         * @param value The simulation-side value.
         * @return The same value at widget precision.
         */
        inline float to_float(Scalar value) { return static_cast<float>(value); }

        /**
         * @brief Widens an edited widget float back to the simulation's `Scalar`.
         * @param value The value the widget produced.
         * @return The same value at simulation precision.
         */
        inline Scalar to_scalar(float value) { return static_cast<Scalar>(value); }

        /**
         * @brief Raises the preferences-dirty flag when a settings block changed this frame.
         *
         * The settings panels are wide — the Rendering panel alone draws dozens of
         * widgets into one trivially-copyable struct — so rather than hook a dirty flag
         * into every slider, each panel snapshots the struct before its widgets run and
         * compares afterwards. Exhaustive by construction: a field added to the struct is
         * covered without touching the panel.
         *
         * @tparam T A trivially-copyable settings aggregate.
         * @param before The snapshot taken before the panel's widgets ran.
         * @param after The same object after them.
         * @param dirty The flag to raise; left untouched when nothing changed, so several
         *              panels can share one flag in a frame.
         */
        template <typename T>
        void push_if_changed(const T& before, const T& after, bool& dirty)
        {
            static_assert(std::is_trivially_copyable<T>::value,
                          "push_if_changed compares raw bytes and needs a trivially-copyable "
                          "aggregate; a type holding a pointer or a container must expose a "
                          "comparison of its own instead.");
            if (std::memcmp(&before, &after, sizeof(T)) != 0)
                dirty = true;
        }

        /**
         * @brief Writes an environment edit through as one undo step per gesture.
         *
         * Shared by every panel that authors the environment (Environment, Meteorology,
         * Lighting, Post Process), which is why it cannot live inside any of them. The
         * snapshot is taken *before* the write: a drag opens a pending change on its first
         * divergent frame and a discrete action records a single step, so undo restores the
         * pre-edit environment. These panels detect their edits by comparing the whole
         * struct rather than per widget, so the bracket is edge-triggered on
         * `ImGui::IsAnyItemActive` instead of a specific item's activation.
         *
         * @param context Shared editor state; owns the history and the open-transaction flag.
         * @param world The world to write the environment to.
         * @param environment The edited environment to install.
         */
        void commit_environment_edit(EditorContext& context, Simulation::IWorldEditor& world,
                                     const Render::Environment& environment);

        /**
         * @brief Commits an open environment transaction once no widget is active any more.
         *
         * Called once per frame by each panel that uses @ref commit_environment_edit. The
         * open-transaction flag is shared, so whichever of those panels draws after the
         * mouse release commits it — exactly once, whichever one that is.
         *
         * @param context Shared editor state; closes the pending change if one is open.
         */
        void finish_environment_edit(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
