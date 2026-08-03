/**************************************************************************/
/* mesh_vertex.hpp                                                        */
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
 * @file mesh_vertex.hpp
 * @brief The engine's drawable vertex format.
 *
 * Position, normal, tangent, two UV sets and a vertex colour — enough for normal
 * mapping, parallax and a detail set, which is what makes the material system
 * possible. The renderer uploads it verbatim and every pass describes its vertex
 * input from these offsets, so the layout is a contract: changing a field changes
 * the pipelines.
 *
 * It lives here rather than with the renderer's mesh registry because it is a plain
 * memory layout, and the things that read one — meshlet clustering, the distance
 * field bake, an importer — are CPU work that must not need a device to compile.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Geometry
    {
        /**
         * @brief One drawable vertex, 60 bytes.
         *
         * @c tangent's w is the bitangent handedness (+1 or -1). A zero tangent is
         * legal and means "none authored": the shader then derives a tangent frame
         * from screen-space derivatives, so a mesh without tangents still normal-maps.
         */
        struct MeshVertex
        {
            float position[3];
            float normal[3];
            float tangent[4];
            float uv0[2];
            float uv1[2];
            std::uint8_t color[4];
        };

        static_assert(sizeof(MeshVertex) == 60, "MeshVertex must stay the 60-byte draw layout");
    } // namespace Geometry
} // namespace SushiEngine
