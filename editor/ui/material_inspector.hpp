/**************************************************************************/
/* material_inspector.hpp                                                 */
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
 * @file material_inspector.hpp
 * @brief The material editor, laid out like Unity's Standard shader inspector.
 *
 * Kept out of editor_panels.cpp because the material is the widest authoring
 * surface in the editor and would otherwise dominate the inspector file. The
 * ordering follows the one an artist expects: tiling and offset at the top, then
 * each map with the scalar or tint that goes with it, then the detail fold-out,
 * then the advanced lobes and the rendering state.
 */

#include <SushiEngine/render/material.hpp>

namespace SushiEngine
{
    namespace Render
    {
        class IAssetLibrary;
    }

    namespace Editor
    {
        /**
         * @brief Draws the material editor for one surface.
         *
         * Texture slots load through @p assets when a path is entered, so the panel
         * owns no asset state of its own.
         *
         * @param material Edited in place.
         * @param assets   Library texture paths are loaded through.
         * @return true when any field changed this frame.
         */
        bool draw_material_editor(SushiEngine::Render::Material& material,
                                  SushiEngine::Render::IAssetLibrary& assets);
    } // namespace Editor
} // namespace SushiEngine
