/**************************************************************************/
/* command_history.cpp                                                    */
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

#include "command_history.hpp"

#include "scene_serializer.hpp"

namespace sushi::editor
{
    namespace
    {
        constexpr std::size_t MAX_DEPTH = 50;

        void push_bounded(std::vector<nlohmann::json>& stack, nlohmann::json snapshot)
        {
            stack.push_back(std::move(snapshot));
            if (stack.size() > MAX_DEPTH)
                stack.erase(stack.begin());
        }
    } // namespace

    void CommandHistory::record(SushiEngine::sim::IWorldEditor& world)
    {
        pending_.reset();
        push_bounded(undo_stack_, capture_scene(world));
        redo_stack_.clear();
    }

    void CommandHistory::begin_change(SushiEngine::sim::IWorldEditor& world)
    {
        if (!pending_.has_value())
            pending_ = capture_scene(world);
    }

    void CommandHistory::end_change()
    {
        if (!pending_.has_value())
            return;
        push_bounded(undo_stack_, std::move(*pending_));
        pending_.reset();
        redo_stack_.clear();
    }

    bool CommandHistory::undo(SushiEngine::sim::IWorldEditor& world)
    {
        pending_.reset();
        if (undo_stack_.empty())
            return false;
        push_bounded(redo_stack_, capture_scene(world));
        const nlohmann::json snapshot = std::move(undo_stack_.back());
        undo_stack_.pop_back();
        apply_scene(world, snapshot);
        return true;
    }

    bool CommandHistory::redo(SushiEngine::sim::IWorldEditor& world)
    {
        pending_.reset();
        if (redo_stack_.empty())
            return false;
        push_bounded(undo_stack_, capture_scene(world));
        const nlohmann::json snapshot = std::move(redo_stack_.back());
        redo_stack_.pop_back();
        apply_scene(world, snapshot);
        return true;
    }
} // namespace sushi::editor
