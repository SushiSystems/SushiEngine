/**************************************************************************/
/* emitter_descriptor.hpp                                                 */
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
 * @file emitter_descriptor.hpp
 * @brief One authored emitter: its module stack, capacity, and simulation domain.
 *
 * An @ref EmitterDescriptor is the artist-facing definition of a single emitter — the
 * aggregate of its spawn/shape/init/update/render modules plus the fixed budget it may never
 * exceed and which backend simulates it. A @ref ParticleEffect is a small list of these.
 * @ref EmitterCompiler bakes one descriptor into a POD CompiledEmitter for the runtime.
 */

#include <cstdint>
#include <string>

#include <SushiEngine/vfx/modules.hpp>

namespace SushiEngine
{
    namespace Vfx
    {
        /** @brief The largest per-emitter particle budget the compiler will honour. */
        constexpr std::uint32_t MAX_EMITTER_CAPACITY = 4u * 1024u * 1024u;

        /** @brief The per-emitter cap for the deterministic CPU backend (bounded state). */
        constexpr std::uint32_t MAX_DETERMINISTIC_PARTICLES = 1024u;

        /**
         * @brief One emitter's complete authored definition.
         *
         * Owns one module of each stage kind (each self-gated by its own @c enabled flag),
         * a capacity that bounds its live particle pool, a looping duration, and the domain
         * that selects its backend. The deterministic path clamps @ref capacity to
         * @ref MAX_DETERMINISTIC_PARTICLES; the cosmetic path clamps to @ref MAX_EMITTER_CAPACITY.
         */
        struct EmitterDescriptor
        {
            std::string name = "Emitter";                     /**< Editor-facing label. */
            SimulationDomain domain = SimulationDomain::Cosmetic; /**< Which backend simulates this. */
            std::uint32_t capacity = 4096;                    /**< Maximum simultaneously-live particles. */
            float duration = 5.0f;                            /**< Emitter cycle length, seconds. */
            bool looping = true;                              /**< Restart the cycle at @ref duration. */
            bool prewarm = false;                             /**< Start as if one full cycle already ran. */

            SpawnModule spawn;                                /**< Emission rate and bursts. */
            ShapeModule shape;                                /**< Birth volume and emit direction. */
            InitModule init;                                  /**< Per-particle birth attributes. */
            GravityModule gravity;                            /**< Constant acceleration. */
            DragModule drag;                                  /**< Velocity damping. */
            TurbulenceModule turbulence;                      /**< Curl-noise wind. */
            SizeOverLifeModule size_over_life;                /**< Size multiplier vs age. */
            ColorOverLifeModule color_over_life;              /**< Colour/alpha vs age. */
            RenderModule render;                              /**< Draw settings. */
        };
    } // namespace Vfx
} // namespace SushiEngine
