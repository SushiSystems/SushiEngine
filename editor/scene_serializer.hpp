/**************************************************************************/
/* scene_serializer.hpp                                                   */
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

#ifndef SUSHIENGINE_EDITOR_SCENE_SERIALIZER_HPP
#define SUSHIENGINE_EDITOR_SCENE_SERIALIZER_HPP

#include <string>

#include <nlohmann/json.hpp>

#include <SushiEngine/sim/simulation.hpp>

namespace sushi::editor
{
    /**
     * @brief Captures every live entity in @p world as JSON, in `.sushiscene` shape.
     *
     * The in-memory counterpart of @ref save_scene, reused by `CommandHistory` to
     * snapshot the world for undo/redo without touching disk.
     *
     * @param world The world to snapshot.
     * @return The scene as a JSON array, one object per entity.
     */
    nlohmann::json capture_scene(SushiEngine::sim::IWorldEditor& world);

    /**
     * @brief Replaces every entity in @p world with the entities described by @p root.
     *
     * The in-memory counterpart of @ref load_scene; see its documentation for the
     * clear-then-recreate semantics and parent-index resolution.
     *
     * @param world The world to repopulate.
     * @param root A JSON array in the shape @ref capture_scene produces.
     */
    void apply_scene(SushiEngine::sim::IWorldEditor& world, const nlohmann::json& root);

    /**
     * @brief Writes every live entity in @p world to a `.sushiscene` JSON file.
     *
     * Reads the world purely through `IWorldEditor`'s query surface, so it names no
     * runtime or ECS type. Parent links are stored as indices into the written array
     * rather than raw `EntityId`s, since ids are not guaranteed stable across a
     * destroy-and-reload.
     *
     * @param world The world to snapshot.
     * @param path Destination file path; overwritten if it exists.
     * @return True on success; false if the file could not be written.
     */
    bool save_scene(SushiEngine::sim::IWorldEditor& world, const std::string& path);

    /**
     * @brief Replaces every entity in @p world with the contents of a `.sushiscene` file.
     *
     * Destroys all existing entities first, then recreates the file's entities in
     * order and reapplies parent links by the same index scheme `save_scene` writes —
     * the world becomes exactly what the file describes, not a merge with it.
     *
     * @param world The world to repopulate.
     * @param path Source file path.
     * @return True on success; false if the file could not be read or parsed, in
     *     which case the world is left cleared rather than partially restored.
     */
    bool load_scene(SushiEngine::sim::IWorldEditor& world, const std::string& path);
} // namespace sushi::editor

#endif
