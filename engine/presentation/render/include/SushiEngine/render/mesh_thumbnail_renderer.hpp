/**************************************************************************/
/* mesh_thumbnail_renderer.hpp                                            */
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
 * @file mesh_thumbnail_renderer.hpp
 * @brief One mesh, one flat/unlit draw, one readback — nothing else.
 *
 * @c ISceneView::render is the production forward path: lighting, environment, TAA, picking,
 * decals, particles. A thumbnail needs none of that, and standing up a full @c ISceneView per
 * resident thumbnail would be both wasteful and awkward to pool. @c IMeshThumbnailRenderer is
 * the deliberately small alternative: load one glTF, frame it with a fixed camera, draw it flat,
 * read the pixels back. See
 * docs/superpowers/specs/2026-08-07-project-panel-model-thumbnails-design.md.
 */

#include <cstdint>

#include "scene_view.hpp"

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief Renders one glTF/GLB model, flat/unlit, into a small offscreen image.
         *
         * Owns an asset stack isolated from the renderer's main scene assets (its own mesh
         * registry, texture library, and bindless heap) — loading or discarding a thumbnail
         * never touches what the live scene has loaded for the same file.
         */
        class IMeshThumbnailRenderer
        {
            public:
                virtual ~IMeshThumbnailRenderer() = default;

                /**
                 * @brief Loads @p path and renders it into a @p width x @p height RGBA8 image.
                 *
                 * The camera is a fixed three-quarter angle auto-framing the model's bounding
                 * box; shading is flat headlight-plus-ambient sampling each primitive's
                 * base-color texture. @p out_image is left untouched on failure.
                 *
                 * @param path   A `.gltf`/`.glb` file path.
                 * @param width  Output image width in pixels.
                 * @param height Output image height in pixels.
                 * @param out_image Receives the rendered result on success.
                 * @return @c true on success; @c false on any load or render failure (an
                 *   unsupported/corrupt file, a model with no position data, more primitives
                 *   than this renderer's fixed capacity, or a Vulkan error).
                 */
                virtual bool render_thumbnail(const char* path, std::uint32_t width,
                                              std::uint32_t height, FrameImage& out_image) = 0;
        };
    } // namespace Render
} // namespace SushiEngine
