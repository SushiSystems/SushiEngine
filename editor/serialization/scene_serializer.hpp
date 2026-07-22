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

#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/sim/simulation.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The astronomical-sky authoring state the Environment panel's "Solar
         * System" section edits (`EditorContext::sky_*`).
         *
         * Not part of @ref SushiEngine::Render::Environment: the ephemeris derives the
         * environment's sun direction and celestial bodies from this each frame, so it is
         * editor-authored input rather than renderer state. Persisted alongside the
         * environment in the scene file so the date/time/location an author dials in
         * survives a reload, the same as the rest of the environment.
         */
        struct SceneSkyState
        {
            bool enabled = true;
            SushiEngine::Astro::CalendarDate date{};
            double latitude_degrees = 0.0;
            double longitude_degrees = 0.0;
            bool astronomical_sun = true;
            bool animate = false;
            double days_per_second = 0.02;
            double accumulated_days = 0.0;
        };

        /**
         * @brief Captures every live entity in @p world as JSON, in `.sushiscene` shape.
         *
         * The in-memory counterpart of @ref save_scene, reused by `CommandHistory` to
         * snapshot the world for undo/redo without touching disk.
         *
         * @param world The world to snapshot.
         * @return The scene as a JSON array, one object per entity.
         */
        nlohmann::json capture_scene(SushiEngine::Simulation::IWorldEditor& world);

        /**
         * @brief Replaces every entity in @p world with the entities described by @p root.
         *
         * The in-memory counterpart of @ref load_scene; see its documentation for the
         * clear-then-recreate semantics and parent-index resolution.
         *
         * @param world The world to repopulate.
         * @param root A JSON array in the shape @ref capture_scene produces.
         */
        void apply_scene(SushiEngine::Simulation::IWorldEditor& world, const nlohmann::json& root);

        /**
         * @brief Writes every live entity in @p world, its environment, and (optionally)
         * the astronomical-sky authoring state to a `.sushiscene` JSON file.
         *
         * Reads the world purely through `IWorldEditor`'s query surface, so it names no
         * runtime or ECS type. Parent links are stored as indices into the written array
         * rather than raw `EntityId`s, since ids are not guaranteed stable across a
         * destroy-and-reload. The file is an object with an `entities` array (the shape
         * @ref capture_scene produces) plus an `environment` object and, when @p sky is
         * given, a `sky` object — so the lighting/sky/rendering-of-the-world authored in
         * the Environment panel survives a reload instead of resetting to defaults.
         *
         * @param world The world to snapshot.
         * @param path Destination file path; overwritten if it exists.
         * @param sky The astronomical-sky state to persist alongside the environment, or
         *     null to omit it (the environment and entities are still written).
         * @return True on success; false if the file could not be written.
         */
        bool save_scene(SushiEngine::Simulation::IWorldEditor& world, const std::string& path,
                         const SceneSkyState* sky = nullptr);

        /**
         * @brief Replaces every entity in @p world with the contents of a `.sushiscene` file.
         *
         * Destroys all existing entities first, then recreates the file's entities in
         * order and reapplies parent links by the same index scheme `save_scene` writes —
         * the world becomes exactly what the file describes, not a merge with it. Also
         * restores the environment (sun, atmosphere, clouds, stars, night lighting, IBL)
         * and, when @p sky is given, the astronomical-sky authoring state. A bare JSON
         * array (the pre-environment-persistence format) is still accepted for backward
         * compatibility; it yields the environment's and sky's existing defaults.
         *
         * @param world The world to repopulate.
         * @param path Source file path.
         * @param sky Filled with the persisted astronomical-sky state, if present in the
         *     file; left untouched otherwise. Null to skip.
         * @return True on success; false if the file could not be read or parsed, in
         *     which case the world is left cleared rather than partially restored.
         */
        bool load_scene(SushiEngine::Simulation::IWorldEditor& world, const std::string& path,
                         SceneSkyState* sky = nullptr);
    } // namespace Editor
} // namespace SushiEngine

#endif
