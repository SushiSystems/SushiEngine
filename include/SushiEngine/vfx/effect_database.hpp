/**************************************************************************/
/* effect_database.hpp                                                    */
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
 * @file effect_database.hpp
 * @brief The particle-effect asset registry — the AssetId home the ECS component references.
 *
 * Mirrors @c Animation::AnimationDatabase: it owns the authored @ref ParticleEffect assets,
 * hands out stable @ref AssetId handles, and lazily compiles each effect to a
 * @ref CompiledEffect the first time it is requested, caching the result until the source is
 * edited. An entity's @ref ParticleEmitter component stores only an AssetId; the backends ask
 * the database for the compiled data. This keeps the heavy authoring types out of both the
 * component and the render seam.
 */

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <SushiEngine/vfx/asset_id.hpp>
#include <SushiEngine/vfx/compiled_emitter.hpp>
#include <SushiEngine/vfx/emitter_compiler.hpp>
#include <SushiEngine/vfx/particle_effect.hpp>

namespace SushiEngine
{
    namespace Vfx
    {
        /**
         * @brief Owns particle-effect assets and their cached compiled forms.
         *
         * Non-copyable: it is the single owner of the effect bytes for a session. Handles are
         * dense indices assigned in registration order and never reused, so an AssetId stays
         * valid for the database's lifetime.
         */
        class EffectDatabase
        {
            public:
                EffectDatabase() = default;
                EffectDatabase(const EffectDatabase&) = delete;
                EffectDatabase& operator=(const EffectDatabase&) = delete;

                /**
                 * @brief Registers an authored effect and returns its handle.
                 * @param effect The effect to take ownership of (moved in).
                 * @return The stable handle for later lookup.
                 */
                AssetId add(ParticleEffect effect)
                {
                    const AssetId id = static_cast<AssetId>(entries_.size());
                    entries_.push_back(Entry{std::move(effect), CompiledEffect{}, false});
                    return id;
                }

                /**
                 * @brief Whether a handle names a registered effect.
                 * @param id The handle to test.
                 * @return True when @p id is in range.
                 */
                bool valid(AssetId id) const noexcept
                {
                    return id < entries_.size();
                }

                /** @brief The number of registered effects. */
                std::size_t size() const noexcept
                {
                    return entries_.size();
                }

                /**
                 * @brief The authored effect for a handle (const).
                 * @param id A valid handle.
                 * @return The authored effect.
                 */
                const ParticleEffect& effect(AssetId id) const noexcept
                {
                    return entries_[id].source;
                }

                /**
                 * @brief The authored effect for a handle, for editing.
                 *
                 * Marks the compiled form stale so the next @ref compiled recompiles.
                 *
                 * @param id A valid handle.
                 * @return The mutable authored effect.
                 */
                ParticleEffect& effect_for_edit(AssetId id) noexcept
                {
                    entries_[id].compiled_valid = false;
                    return entries_[id].source;
                }

                /**
                 * @brief The compiled form for a handle, compiling on first use or after an edit.
                 * @param id A valid handle.
                 * @return The cached compiled effect.
                 */
                const CompiledEffect& compiled(AssetId id)
                {
                    Entry& entry = entries_[id];
                    if (!entry.compiled_valid)
                    {
                        entry.compiled = EmitterCompiler::compile(entry.source);
                        entry.compiled_valid = true;
                    }
                    return entry.compiled;
                }

            private:
                struct Entry
                {
                    ParticleEffect source;
                    CompiledEffect compiled;
                    bool compiled_valid = false;
                };

                std::vector<Entry> entries_;
        };
    } // namespace Vfx
} // namespace SushiEngine
