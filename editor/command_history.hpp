/**************************************************************************/
/* command_history.hpp                                                    */
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

#ifndef SUSHIENGINE_EDITOR_COMMAND_HISTORY_HPP
#define SUSHIENGINE_EDITOR_COMMAND_HISTORY_HPP

#include <cstdint>
#include <optional>
#include <vector>

#include <nlohmann/json.hpp>

#include <SushiEngine/sim/simulation.hpp>

namespace sushi::editor
{
    /**
     * @brief Undo/redo over whole-world JSON snapshots (`scene_serializer`'s format).
     *
     * There is no per-field command hierarchy — every undo step is a full
     * `capture_scene`/`apply_scene` round-trip, which is simple and correct for a
     * single-user editor at this entity count, at the cost of coarser granularity
     * than a field-level command would give. Two recording modes cover the panels:
     * @ref record for a single discrete action (create, delete, reparent), and the
     * @ref begin_change / @ref end_change pair for a continuous drag (a slider held
     * across several frames), which must only cost one undo step.
     */
    class CommandHistory
    {
        public:
            /**
             * @brief Snapshots @p world now and pushes it as one undo step.
             *
             * Call this immediately before a discrete, single-frame mutation (an
             * entity create/delete, a rename commit, a reparent). Clears the redo
             * stack, since a new action invalidates any previously undone future.
             *
             * @param world The world to snapshot before the caller mutates it.
             */
            void record(SushiEngine::sim::IWorldEditor& world);

            /**
             * @brief Starts (or continues) tracking a multi-frame edit.
             *
             * Snapshots @p world only the first time this is called since the last
             * @ref end_change, so a widget held across several frames (a drag) is
             * captured once, at its pre-edit state — call this on the frame the
             * widget activates (e.g. `ImGui::IsItemActivated()`).
             *
             * @param world The world to snapshot at the start of the edit.
             */
            void begin_change(SushiEngine::sim::IWorldEditor& world);

            /**
             * @brief Commits the pending snapshot from @ref begin_change as one undo step.
             *
             * A no-op if no change is pending. Call this when the widget's edit
             * finishes (e.g. `ImGui::IsItemDeactivatedAfterEdit()`).
             */
            void end_change();

            /**
             * @brief Steps @p world back to the previous snapshot.
             * @param world The world to restore into.
             * @return True if there was a snapshot to undo to; false if the stack was empty.
             */
            bool undo(SushiEngine::sim::IWorldEditor& world);

            /**
             * @brief Re-applies the snapshot most recently undone.
             * @param world The world to restore into.
             * @return True if there was a snapshot to redo; false if the stack was empty.
             */
            bool redo(SushiEngine::sim::IWorldEditor& world);

            /** @brief Whether @ref undo would do anything. */
            bool can_undo() const noexcept { return !undo_stack_.empty(); }

            /** @brief Whether @ref redo would do anything. */
            bool can_redo() const noexcept { return !redo_stack_.empty(); }

            /**
             * @brief A counter bumped by every world-mutating operation this history
             * records or replays (@ref record, a committed @ref end_change, @ref undo,
             * @ref redo).
             *
             * Lets a host detect "the world changed since I last saved" without
             * comparing snapshots: stash this value at save time and compare it each
             * frame against the stashed one to know whether the scene is dirty.
             */
            std::uint64_t revision() const noexcept { return revision_; }

        private:
            std::vector<nlohmann::json> undo_stack_;
            std::vector<nlohmann::json> redo_stack_;
            std::optional<nlohmann::json> pending_;
            std::uint64_t revision_ = 0;
    };
} // namespace sushi::editor

#endif
