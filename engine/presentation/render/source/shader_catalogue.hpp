/**************************************************************************/
/* shader_catalogue.hpp                                                   */
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
 * @file shader_catalogue.hpp
 * @brief The one list of every shader the renderer ships.
 *
 * Naming the shaders in one place rather than at the point of use is what keeps the
 * build-time SPIR-V and the hot-reload source paths from drifting apart: a shader
 * added here is compiled, embedded, and watched, and a shader missing from here is
 * simply not reachable.
 */

#include <cstddef>

#include "resources/shader_library.hpp"

#ifndef SUSHI_SHADER_SOURCE_DIR
#define SUSHI_SHADER_SOURCE_DIR ""
#endif

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief Every shader, with its stage, embedded SPIR-V, and source file.
         * @return A static array valid for the process lifetime.
         */
        const Resources::ShaderSource* shader_catalogue() noexcept;

        /**
         * @brief Number of entries in shader_catalogue().
         * @return The catalogue's length.
         */
        std::size_t shader_catalogue_count() noexcept;
    } // namespace Render
} // namespace SushiEngine
