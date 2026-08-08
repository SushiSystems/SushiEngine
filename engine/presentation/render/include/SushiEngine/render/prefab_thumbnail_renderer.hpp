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
 * separate, parallel interface: draw a caller-resolved array of entities, each with its own
 * real model matrix; this renderer computes its own framing camera from the union of every
 * draw's mesh bounding sphere, scaled by that draw's model matrix, so the caller never has to
 * reach into this renderer's private mesh data to produce one.
 *
 * This class is deliberately ignorant of prefabs, JSON, and `ISimulation`/`IWorldEditor` --
 * `engine/presentation/render` (the `presentation` tier) is forbidden from depending on
 * `engine/world/simulation`/`engine/world/serialization` (the `world` tier): `world` sits
 * above `presentation` in this repository's tier order (`cmake/EngineLayers.cmake`), and the
 * `render`-depends-on-`simulation` edge is separately listed in `SUSHIENGINE_FORBIDDEN_EDGES`
 * as one no tier arrangement may ever make legal. Instantiating a `.sushiprefab` into a
 * throwaway world, applying it, and resolving its assets against @ref asset_library are all
 * therefore the caller's job -- the `application` tier (the editor), which is free to depend
 * on both `world` and `presentation`. That caller (Phase 4b's `PrefabThumbnailCache`, a
 * separate, not-yet-written plan) walks the resolved world itself and hands this class the
 * plain `PrefabThumbnailDraw` array below.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/material/material.hpp>

#include "asset_library_interface.hpp"
#include "scene_view.hpp"

namespace SushiEngine
{
    namespace Render
    {
        /** @brief One already-resolved entity's worth of draw data for a prefab thumbnail. */
        struct PrefabThumbnailDraw
        {
            MeshId mesh = INVALID_MESH;
            Material material;
            Matrix4 model; /**< This entity's own composed world transform. */
        };

        /**
         * @brief Renders a caller-resolved set of entities into an image.
         *
         * Owns an asset stack isolated from the renderer's main scene assets, the same way
         * IMeshThumbnailRenderer does -- resolving a prefab thumbnail's assets never touches
         * what the live scene has loaded for the same file.
         */
        class IPrefabThumbnailRenderer
        {
            public:
                virtual ~IPrefabThumbnailRenderer() = default;

                /**
                 * @brief The isolated asset library this renderer's meshes/materials live in.
                 *
                 * A caller resolves a prefab's mesh_path/material-path references against
                 * this library (e.g. via Scene::resolve_scene_assets, from world-tier code
                 * this render-tier class must never call directly) before building the
                 * PrefabThumbnailDraw array render_thumbnail() below expects.
                 */
                virtual IAssetLibrary& asset_library() noexcept = 0;

                /**
                 * @brief The fixed maximum number of entries @ref render_thumbnail accepts.
                 *
                 * A caller building a `PrefabThumbnailDraw` array (walking a resolved prefab's
                 * entities) must compare its running count against this before ever calling
                 * @ref render_thumbnail, and treat exceeding it as a load failure -- the same
                 * way it already must treat a prefab producing zero draws. Exists so that cap
                 * lives in exactly one place (the concrete renderer's own fixed capacity)
                 * instead of being duplicated as a magic number in every caller.
                 */
                virtual std::size_t max_draws() const noexcept = 0;

                /**
                 * @brief Renders exactly the given resolved draws into a width x height RGBA8 image.
                 *
                 * Shading is flat headlight-plus-ambient sampling each draw's base-color
                 * texture. The framing camera is computed internally from the union of every
                 * draw's mesh bounding sphere, scaled by that draw's model matrix -- the caller
                 * supplies no bounds of its own. @p out_image is left untouched on failure.
                 * This method submits to the same graphics queue the main renderer uses for its
                 * own frame submission (externally synchronized in Vulkan) and blocks the
                 * calling thread synchronously until the GPU finishes rendering and the readback
                 * completes.
                 *
                 * @param draws  The entities to draw, already resolved to live mesh/material
                 *   handles in this renderer's own asset_library(). Must not be null when
                 *   @p count is greater than zero; a null pointer with a nonzero count is a
                 *   defined @c false return, not undefined behavior.
                 * @param count  Number of entries in @p draws. Must not exceed a fixed
                 *   implementation capacity (a caller exceeding it should treat that as a
                 *   load failure before ever calling this method). Zero is a defined @c false
                 *   return: an empty prefab is a load failure, not a silently valid thumbnail.
                 * @param width  Output image width in pixels.
                 * @param height Output image height in pixels.
                 * @param out_image Receives the rendered result on success.
                 * @return @c true on success; @c false on any render failure (a Vulkan error,
                 *   a null @p draws with nonzero @p count, a zero @p count, or more draws than
                 *   this renderer's fixed capacity).
                 */
                virtual bool render_thumbnail(const PrefabThumbnailDraw* draws, std::size_t count,
                                              std::uint32_t width, std::uint32_t height,
                                              FrameImage& out_image) = 0;
        };
    } // namespace Render
} // namespace SushiEngine
