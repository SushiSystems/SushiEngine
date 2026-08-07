/**************************************************************************/
/* prefab_thumbnail_renderer.hpp                                          */
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
 * @file prefab_thumbnail_renderer.hpp
 * @brief Every entity a prefab actually resolves to today, drawn flat/unlit, read back once.
 *
 * A prefab is a captured entity subtree, not a single mesh — each of its entities can carry
 * its own independent mesh, material, and transform, potentially from entirely different
 * source files (see docs/superpowers/specs/2026-08-07-project-panel-prefab-thumbnails-design.md).
 * @c IMeshThumbnailRenderer's single-glTF-file contract cannot represent that, so this is a
 * separate, parallel interface: instantiate the prefab into a throwaway world, resolve its
 * assets, frame a camera around the union of every entity's bounds, and draw each one with its
 * own real model matrix.
 */

#include <cstdint>

#include "scene_view.hpp"

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief Renders one .sushiprefab's actual, current, resolved entities into an image.
         *
         * Owns an asset stack isolated from the renderer's main scene assets, the same way
         * IMeshThumbnailRenderer does — loading or discarding a prefab thumbnail never touches
         * what the live scene has loaded for the same file.
         */
        class IPrefabThumbnailRenderer
        {
            public:
                virtual ~IPrefabThumbnailRenderer() = default;

                /**
                 * @brief Loads @p path, instantiates it, and renders every resulting entity.
                 *
                 * Internally: creates a throwaway simulation, applies the prefab document into
                 * it, resolves its assets against this renderer's own isolated asset stack,
                 * frames a fixed three-quarter camera around the union of every entity's
                 * world-space bounds, and draws each entity flat/unlit with its own real model
                 * matrix. The throwaway simulation is discarded before this call returns.
                 *
                 * This method submits to the same graphics queue the main renderer uses for its
                 * own frame submission (externally synchronized in Vulkan — do not call this
                 * concurrently with the main renderer's own frame submission from a different
                 * thread without external synchronization). It blocks the calling thread
                 * synchronously until the GPU finishes rendering and the readback completes.
                 *
                 * @param path   A `.sushiprefab` file path.
                 * @param width  Output image width in pixels.
                 * @param height Output image height in pixels.
                 * @param out_image Receives the rendered result on success.
                 * @return @c true on success; @c false on any load or render failure (an
                 *   unsupported/corrupt document, a document with no entities, more entities or
                 *   primitives than this renderer's fixed capacity, or a Vulkan error).
                 */
                virtual bool render_thumbnail(const char* path, std::uint32_t width,
                                              std::uint32_t height, FrameImage& out_image) = 0;
        };
    } // namespace Render
} // namespace SushiEngine
