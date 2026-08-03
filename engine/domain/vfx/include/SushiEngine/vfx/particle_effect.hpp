/**************************************************************************/
/* particle_effect.hpp                                                    */
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
 * @file particle_effect.hpp
 * @brief A particle effect asset — one or more emitters authored as a unit.
 *
 * A @ref ParticleEffect is what an artist saves as a `.sushieffect` and an entity references
 * through a @ref ParticleEmitter component's AssetId: a small ordered list of
 * @ref EmitterDescriptor plus asset metadata. It carries no runtime or GPU state — the
 * database compiles it to a CompiledEffect and the backends simulate that.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/vfx/emitter_descriptor.hpp>

namespace SushiEngine
{
    namespace Vfx
    {
        /** @brief The largest number of emitters one effect may contain. */
        constexpr std::uint32_t MAX_EMITTERS_PER_EFFECT = 16;

        /**
         * @brief One authored particle effect: a named list of emitters.
         *
         * @ref emitters is drawn and simulated together as one effect instance; each entry
         * chooses its own simulation domain, so a single effect can mix a cosmetic GPU smoke
         * plume with a deterministic CPU spark shower.
         */
        struct ParticleEffect
        {
            std::string name = "Effect";                 /**< Asset name (editor + serialization). */
            std::vector<EmitterDescriptor> emitters;     /**< The emitters, drawn together. */

            /** @brief Whether the effect has at least one emitter. */
            bool empty() const noexcept
            {
                return emitters.empty();
            }
        };
    } // namespace Vfx
} // namespace SushiEngine
