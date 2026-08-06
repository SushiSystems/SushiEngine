/**************************************************************************/
/* import_settings.hpp                                                    */
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
 * @file import_settings.hpp
 * @brief What one model asset says about how it is imported.
 *
 * The payload of the `.meta` file that sits beside the asset. Settings live beside the asset
 * rather than in a project-wide table keyed by path so that moving, renaming or copying the
 * asset carries them along instead of orphaning them.
 *
 * Every default is "change nothing". glTF fixes a right-handed coordinate system with +Y up and
 * metres for every linear distance, so a conformant file needs no correction and the two
 * transform fields exist for a file whose exporter ignored that.
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/cooking/import_profile.hpp>

namespace SushiEngine
{
    namespace Model
    {
        /** @brief 3-vector in import precision; the same per-namespace alias `Animation` uses. */
        using Vector3f = Vector3T<float>;

        /** @brief The per-asset import settings a `.meta` file holds. */
        struct ModelImportSettings
        {
            /** @brief Uniform scale applied at import; 1 leaves the file's own units alone. */
            float scale_factor = 1.0f;

            /**
             * @brief Rotation applied to the imported subtree's root, in degrees per axis.
             *
             * An escape hatch for a file whose exporter ignored glTF's orientation, not an
             * axis-convention picker. Zero, the default, is what a conformant file needs.
             */
            Vector3f root_rotation_degrees{0.0f, 0.0f, 0.0f};

            /** @brief Adopt the file's materials; off leaves each entity the engine default. */
            bool import_materials = true;

            /** @brief Turn `KHR_lights_punctual` lights into Light entities. */
            bool import_lights = true;

            /** @brief Turn glTF cameras into Camera entities, created inactive. */
            bool import_cameras = true;

            /** @brief Keep nodes that carry nothing, so authored pivots survive the import. */
            bool preserve_pivots = true;

            /** @brief Queue every mesh-carrying node for the physics cooking pipeline. */
            bool generate_colliders = false;

            /** @brief What this asset says differs from the project's cooking defaults. */
            Physics::Cooking::ImportProfileOverride cooking;
        };

        /**
         * @brief Whether two settings objects would produce the same import.
         *
         * Field by field, including each unset optional in @ref ModelImportSettings::cooking:
         * an override that is absent and one that happens to hold the project's own value are
         * different settings, and comparing them as equal would hide that.
         *
         * @param a First operand.
         * @param b Second operand.
         * @return True when every field matches.
         */
        bool operator==(const ModelImportSettings& a, const ModelImportSettings& b) noexcept;

        /**
         * @brief The negation of @ref operator==.
         *
         * @param a First operand.
         * @param b Second operand.
         * @return True when any field differs.
         */
        bool operator!=(const ModelImportSettings& a, const ModelImportSettings& b) noexcept;
    } // namespace Model
} // namespace SushiEngine
