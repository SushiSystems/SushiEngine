/**************************************************************************/
/* environment_serializer.hpp                                             */
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

#ifndef SUSHIENGINE_SCENE_ENVIRONMENT_SERIALIZER_HPP
#define SUSHIENGINE_SCENE_ENVIRONMENT_SERIALIZER_HPP

/**
 * @file environment_serializer.hpp
 * @brief The one JSON shape for an authored `Render::Environment`.
 *
 * Shared by the scene serializer (the environment is scene content — every scene
 * file, undo snapshot, and play-mode snapshot carries it) and by the preferences
 * store (which persists a *default* environment applied only to new scenes). One
 * owner for the shape is what stops the two from diverging into a full scene copy
 * and a partial preferences copy that silently drops fog, GI, and the nest physics
 * — which is exactly the state this file replaced.
 *
 * What is deliberately not serialized: the runtime-owned channels. The borrowed
 * `AtmosphereForcing` pointers and the `WeatherField` view are installed by the
 * live simulation; `WeatherCoupling` is recomputed from scratch every extract;
 * `bodies`/`sky_stars`/`dominant_*`/the sun's position are repopulated every frame
 * by the ephemeris from the scene's sky state; `atmosphere_nest_size` is a
 * per-user machine budget the host injects per frame. `environment_from_json`
 * starts from a caller-provided base, so all of those ride through unchanged —
 * applying a loaded environment can never sever the live weather.
 */

#include <nlohmann/json.hpp>

#include <SushiEngine/environment/environment.hpp>

namespace SushiEngine
{
    namespace Scene
    {
        /**
         * @brief Serializes the authored fields of @p environment to JSON.
         *
         * Covers what an author edits in the Environment/Lighting/Meteorology panels:
         * sun, atmosphere, fog (including the local fog volumes), GI, ground, clouds,
         * the regional nest's full physics, stars, night lighting, ambient, exposure,
         * IBL, and the sky observer. Runtime-owned fields are never written (see the
         * file comment).
         *
         * @param environment The environment to serialize.
         * @return The environment as a JSON object.
         */
        nlohmann::json environment_to_json(const SushiEngine::Render::Environment& environment);

        /**
         * @brief Applies the fields present in @p json onto @p base and returns the result.
         *
         * Tolerant by construction: every read defaults to the value already in
         * @p base, so a document written before a field existed loads with that
         * field's current value rather than a zero — and the runtime-owned fields the
         * shape never writes (forcing pointers, weather field, ephemeris output) are
         * preserved from @p base, which is why callers pass the *live* environment as
         * the base when applying a loaded one.
         *
         * @param json The serialized environment (a non-object yields @p base verbatim).
         * @param base The environment the parsed fields are applied over.
         * @return @p base with every serialized field overwritten from @p json.
         */
        SushiEngine::Render::Environment environment_from_json(
            const nlohmann::json& json, SushiEngine::Render::Environment base);
    } // namespace Scene
} // namespace SushiEngine

#endif
