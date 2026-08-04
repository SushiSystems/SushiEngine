/**************************************************************************/
/* weather_field.hpp                                                      */
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
 * @file weather_field.hpp
 * @brief The spatial weather field the renderer reads instead of one global column.
 *
 * `docs/design/atmosphere_system.md` §1.1 names the defect this type exists to close. A
 * simulation that reaches the renderer as a *single* `WeatherColumn` sampled under the
 * observer compiles into a globally uniform deck stack, so everything actually visible in
 * the sky comes from tiled noise and no simulated structure — a front, a shower, a
 * clearing — can ever be seen. This is the field form of the same data: the simulation's
 * own horizontal grid, addressed in world space, which §7.3's coverage authority samples
 * per march step.
 *
 * The samples are **borrowed, not owned**: `Environment` is copied per frame into the
 * render snapshot, and copying tens of kilobytes of weather every frame to describe
 * something that changes on a multi-second cadence would be absurd. The producer (the
 * simulation) keeps the storage alive for as long as the environment referencing it can
 * be read, the same arrangement `ParticleView::compiled` already uses for sim-owned
 * emitters. @ref WeatherField::revision is what lets a consumer notice a change without
 * comparing the payload.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief One horizontal cell's weather at one vertical band.
         *
         * The narrow slice of the simulation's cell state that drives rendering, in the
         * same units the deck authoring path already speaks: a coverage fraction, an
         * opacity scale, a stratiform/convective mix, and the column's surface
         * precipitation. Four floats so the GPU texel is a straight `RGBA` encode.
         */
        struct WeatherFieldSample
        {
            float coverage = 0.0f;            /**< Fraction of sky this band covers here, [0, 1]. */
            float density_scale = 0.0f;       /**< Opacity/thickness scale, [0, 2]. */
            float convective_fraction = 0.0f; /**< 0 stratiform sheet -> 1 cellular/towering, [0, 1]. */
            float precipitation = 0.0f;       /**< Column surface precipitation rate at this cell, [0, 1]. */
        };

        /** @brief Vertical bands a @ref WeatherField carries; mirrors `Simulation::CloudLevel`. */
        constexpr int WEATHER_FIELD_LEVELS = 3;

        /**
         * @brief Upper bound on a field's horizontal cell count per axis.
         *
         * Sizes the renderer's texture once at startup rather than reallocating when a
         * scene loads a differently-sized simulation grid — the same reasoning that fixes
         * the cloud noise volumes and the atmosphere LUTs at construction. A producer with
         * a finer grid publishes a decimated field; it never overruns this.
         */
        constexpr int WEATHER_FIELD_MAX_CELLS = 64;

        /**
         * @brief A borrowed view of the simulation's weather grid, plus how to address it.
         *
         * Trivially copyable and pointer-sized, so it rides in `Environment` at no
         * meaningful per-frame cost. Addressing is a plain scale-and-offset from
         * **scene-absolute** world XZ to the field's `[0, 1]` UV, which is the same flat
         * tangent-plane approximation the producer's own grid and the baked cloud field
         * already accept at this scale; the renderer folds the camera position into the
         * offset when it packs its uniforms, exactly as it already does for the planet
         * centre.
         */
        struct WeatherField
        {
            const WeatherFieldSample* samples = nullptr; /**< `cells_x * cells_z * level_count`, level-major (level, z, x); borrowed. */
            std::int32_t cells_x = 0;      /**< Horizontal cell count, X axis, `<= WEATHER_FIELD_MAX_CELLS`. */
            std::int32_t cells_z = 0;      /**< Horizontal cell count, Z axis. */
            std::int32_t level_count = 0;  /**< Populated vertical bands, `<= WEATHER_FIELD_LEVELS`. */
            std::uint32_t revision = 0;    /**< Bumped whenever @ref samples' contents change; 0 means "never published". */

            float uv_scale_x = 0.0f;  /**< Scene-absolute world X metres -> U. */
            float uv_scale_z = 0.0f;  /**< Scene-absolute world Z metres -> V. */
            float uv_offset_x = 0.0f; /**< U at world X = 0. */
            float uv_offset_z = 0.0f; /**< V at world Z = 0. */

            /**
             * @brief Altitude each vertical band is centred on, metres above the surface.
             *
             * The renderer maps a march sample's altitude onto a continuous band
             * coordinate through these, so a cloud sitting between two bands reads a blend
             * of both rather than snapping at a bucket edge. Ascending.
             */
            float level_altitudes[WEATHER_FIELD_LEVELS]{};

            /**
             * @brief Whether the renderer should resolve cloud genus from this field.
             *
             * `docs/design/atmosphere_system.md` §7.4: with a spatial field the cloudscape bake
             * resolves a genus *per baked column* from the local band state, instead of
             * instantiating one globally compiled deck stack everywhere. That is right for a
             * provider whose column state is meteorology — it is what lets a stratus sheet, a
             * cumulus field and a cirrus deck coexist in one view, each where the simulation
             * actually put it — and wrong for `StaticWeather`, whose column state *is* an
             * authored deck stack decomposed into bands: re-deriving a genus from it would
             * quietly overrule the author's own choice of genus. So the producer answers the
             * question rather than the renderer guessing from the field's shape.
             *
             * False leaves the bake on the authored deck stack. The field is still uploaded
             * and still describes the sky truthfully; it simply says nothing the deck stack
             * does not already say, which for a uniform authored sky is the whole truth.
             */
            bool derives_genus = false;

            /**
             * @brief Altitude span the derived decks occupy across the whole field, metres.
             *
             * The march shell (`cloud_global.yz`) is the union of every deck that can appear.
             * When the decks are compiled once, globally, that union is a property of the deck
             * stack; when each column resolves its own genus it is a property of the *field*,
             * because a cumulonimbus growing 300 km away still has to be inside the shell the
             * march crosses or it is invisible. The producer takes the union as it fills the
             * cells — it is the only party that sees every column — and the renderer uses it
             * in place of the deck-derived shell whenever @ref derives_genus is set.
             *
             * Both zero when nothing was classified (no band reached the enable threshold),
             * which the renderer reads as "no cloud", exactly as an empty deck stack is read.
             */
            float union_base_m = 0.0f;
            float union_top_m = 0.0f;

            /** @brief Whether this field carries usable data this frame. */
            bool valid() const noexcept
            {
                return samples != nullptr && cells_x > 0 && cells_z > 0 && level_count > 0;
            }
        };
    } // namespace Render
} // namespace SushiEngine
