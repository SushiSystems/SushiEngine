/**************************************************************************/
/* effect_serializer.hpp                                                  */
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
 * @file effect_serializer.hpp
 * @brief Reading and writing `.sushieffect` files (design §8, VFX6a).
 *
 * A particle effect is authored as a `Vfx::ParticleEffect` — the descriptor tree, not the compiled
 * record — so that is what is persisted: JSON in the same shape and spirit as `.sushiscene`, one
 * object per emitter with a sub-object per module. The compiled form is a build product and is
 * never written; it is rebuilt from this on load, which is what lets a saved effect survive a
 * change to the compiled layout.
 *
 * Every field an author can reach is round-tripped, including the curves and gradients as their
 * authored key lists rather than as baked LUTs.
 */

#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include <SushiEngine/vfx/particle_effect.hpp>

namespace SushiEngine
{
    namespace Render
    {
        class IAssetLibrary;
    }

    namespace Editor
    {
        /** @brief The extension every particle-effect asset carries, including the dot. */
        extern const char* const EFFECT_FILE_EXTENSION;

        /**
         * @brief Captures @p effect as JSON in `.sushieffect` shape.
         * @param effect The authored effect.
         * @return The effect as a JSON object.
         */
        nlohmann::json capture_effect(const Vfx::ParticleEffect& effect);

        /**
         * @brief Rebuilds @p effect from JSON in the shape @ref capture_effect produces.
         *
         * Missing keys keep the corresponding module's default, so a file written by an older
         * build loads with the newer defaults rather than failing.
         *
         * @param root   The JSON object to read.
         * @param effect Replaced with the described effect.
         * @return True when @p root was a usable effect object.
         */
        bool apply_effect(const nlohmann::json& root, Vfx::ParticleEffect& effect);

        /**
         * @brief Turns every emitter's authored texture path into a live texture handle.
         *
         * The counterpart to writing the path rather than the handle: a deserialised effect names
         * its sprite textures by path and carries no id until this runs. Called once per load
         * rather than lazily, so a path that names no file costs one failed read instead of one
         * per frame for as long as the effect is selected.
         *
         * An emitter with no path keeps @c Vfx::NO_PARTICLE_TEXTURE and draws as the built-in dot;
         * so does one whose path could not be read.
         *
         * @param effect The effect to resolve, edited in place.
         * @param assets The library the paths are loaded through.
         */
        void resolve_effect_textures(Vfx::ParticleEffect& effect, Render::IAssetLibrary& assets);

        /**
         * @brief Writes @p effect to @p path.
         * @param effect The authored effect.
         * @param path   Destination file path; overwritten if it exists.
         * @return True on success; false if the file could not be written.
         */
        bool save_effect(const Vfx::ParticleEffect& effect, const std::string& path);

        /**
         * @brief Reads an effect from @p path.
         * @param path   Source file path.
         * @param effect Replaced with the loaded effect on success; untouched on failure.
         * @return True on success; false if the file could not be read or parsed.
         */
        bool load_effect(const std::string& path, Vfx::ParticleEffect& effect);

        /**
         * @brief Lists the effect assets in @p directory, newest listing order unspecified.
         *
         * Missing directories are not an error — an author who has saved nothing yet simply has an
         * empty library, and creating the directory is the business of whatever saves into it.
         *
         * @param directory The folder to scan, non-recursively.
         * @return Full paths of the files whose extension is @ref EFFECT_FILE_EXTENSION.
         */
        std::vector<std::string> list_effect_files(const std::string& directory);
    } // namespace Editor
} // namespace SushiEngine
