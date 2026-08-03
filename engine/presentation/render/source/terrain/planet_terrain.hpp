/**************************************************************************/
/* planet_terrain.hpp                                                     */
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
 * @file planet_terrain.hpp
 * @brief One body's terrain, from its asset to the buffer a draw reads.
 *
 * The composition root of the terrain path: it owns the pack, the height source over it,
 * the layer stack, and the slot cache, and each frame it turns a camera position into the
 * node array `terrain.vert` indexes (`docs/slop/solar_system_overhaul.md` §7, §8).
 *
 * It owns no Vulkan of its own beyond the cache — the buffers it fills are graph
 * transients the pass declares, so nothing here outlives a frame or needs a barrier.
 *
 * **Frames.** Everything the selection and the cube-sphere map do happens in the body's
 * own fixed frame, which is where the elevations are defined and where the ellipsoid sits
 * at the origin. The camera arrives already expressed in it, and the rotation back into
 * the scene frame travels to the shader in the body block rather than being applied here:
 * rotating a few thousand node origins on the host would cost more than rotating each
 * vertex once on the device, and it would put the scene frame into data that has no
 * business knowing about it.
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/terrain/layer_stack.hpp>
#include <SushiEngine/terrain/pack_format.hpp>
#include <SushiEngine/terrain/quadtree.hpp>

#include "terrain/terrain_frame.hpp"
#include "terrain/tile_cache.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Terrain
        {
            /**
             * @brief Where a body's baked terrain is looked for.
             *
             * One convention in one place, rather than a path spelled out at each call
             * site: `se planet bake` writes here and this reads from here, and a body with
             * nothing baked simply has no file, which @ref PlanetTerrain::set_body treats
             * as "no terrain" rather than as an error.
             *
             * @param body Ephemeris body index; negative yields an empty path.
             * @return The asset path, or empty when there can be none.
             */
            std::string default_pack_path(int body);

            /** @brief How a body's terrain is configured at bring-up. */
            struct PlanetTerrainDesc
            {
                /** @brief Tiles that may be resident at once. */
                std::uint32_t slot_count = 2048;

                /**
                 * @brief Screen-space error a node's cell may project to, pixels.
                 *
                 * Four on the baseline: §17's measured table puts that where the triangle
                 * count meets the budget, and two is a tier for hardware with the fill rate
                 * for five million triangles.
                 */
                double screen_error_pixels = 4.0;

                /** @brief Deepest level the selection may reach. */
                std::uint8_t maximum_depth = 12;

                /** @brief Most nodes one frame may draw; sizes the node buffer. */
                std::size_t maximum_nodes = 4096;
            };

            /**
             * @brief One selected node, exactly as `terrain_common.glsl` declares it.
             *
             * Eighty bytes of std430. Kept beside the GLSL struct it mirrors, and pinned by
             * a static assertion, because a silent layout drift here draws a planet made of
             * garbage rather than failing.
             */
            struct TerrainNodeRecord
            {
                float origin[4];     /**< xyz = camera-relative surface point, w = grid span. */
                float centre[4];     /**< xyz = node centre's cube point, w = cube face. */
                float grid_morph[4]; /**< xy = grid origin, z/w = morph start/end metres. */
                float decode[4];     /**< x/y = slot elevation range, z = slot layer. */
                float uv_rect[4];    /**< xy = uv offset, zw = uv scale. */
            };

            static_assert(sizeof(TerrainNodeRecord) == 80,
                          "the node record must match terrain_common.glsl's std430 layout");

            /** @brief The body block: the ellipsoid and the rotation out of its frame. */
            struct TerrainBodyRecord
            {
                float semi_axes[4];
                float body_to_scene[16];
            };

            /**
             * @brief A body's terrain: its asset, its edits, its cache, and its frame data.
             *
             * Non-copyable: it owns the cache's device resources.
             */
            class PlanetTerrain
            {
                public:
                    /**
                     * @brief Prepares the selector; claims no device memory yet.
                     *
                     * Deliberately does not take a body. A view is built long before anyone
                     * knows which world it will look at, and the slot pool is tens of
                     * megabytes — a scene with no terrain in it should not pay for one.
                     * @ref set_body is what turns this on.
                     *
                     * @param device The live Vulkan device.
                     * @param desc   What to select for, and how large the pool may grow.
                     */
                    PlanetTerrain(Vulkan::VulkanDevice& device, const PlanetTerrainDesc& desc);

                    PlanetTerrain(const PlanetTerrain&) = delete;
                    PlanetTerrain& operator=(const PlanetTerrain&) = delete;

                    /**
                     * @brief Points this at a body, loading its pack and sizing its pool.
                     *
                     * Idempotent in effect but not in cost, so the caller compares against
                     * @ref body() first. A body with no pack still becomes the current
                     * body: that is what stops a missing file from being re-opened every
                     * frame for the rest of the session.
                     *
                     * @warning The device must be idle when the body actually changes. The
                     *          slot pool survives — the slots are anonymous storage and
                     *          only their index is body-specific — but re-pointing it drops
                     *          bindings that frames in flight are still drawing from.
                     *
                     * @param body      Ephemeris body index, or negative for none.
                     * @param pack_path Its asset; see @ref default_pack_path.
                     */
                    void set_body(int body, const std::string& pack_path);

                    /** @brief The body this is currently pointed at, or negative for none. */
                    int body() const noexcept { return body_index_; }

                    /**
                     * @brief Whether a pack was found and accepted.
                     *
                     * False is not an error: a body with no baked terrain falls back to the
                     * analytic ground the sky pass already draws, which is what shipped
                     * before terrain existed.
                     */
                    bool loaded() const noexcept { return pack_.loaded(); }

                    /**
                     * @brief Whether this frame's selection produced something to draw.
                     *
                     * The one question the pass and the sky's analytic ground both need
                     * answered, and the only honest place to answer it is after
                     * @ref prepare has run.
                     */
                    bool drawing() const noexcept { return cache_ && !records_.empty(); }

                    /**
                     * @brief Selects this frame's nodes, binds their slots, and packs the buffer.
                     *
                     * Missing tiles are staged in nearest-first order up to the cache's
                     * per-frame budget; whatever did not fit is inherited from an ancestor
                     * and drawn coarser, so a frame is never delayed by a tile.
                     *
                     * @param view Where the body is being looked at from.
                     */
                    void prepare(const TerrainFrameView& view);

                    /** @brief Nodes packed by the last @ref prepare. */
                    std::uint32_t node_count() const noexcept
                    {
                        return static_cast<std::uint32_t>(records_.size());
                    }

                    /** @brief The packed node array; @ref node_count records. */
                    const TerrainNodeRecord* node_records() const noexcept
                    {
                        return records_.data();
                    }

                    /** @brief Bytes the node array occupies. */
                    std::size_t node_bytes() const noexcept
                    {
                        return records_.size() * sizeof(TerrainNodeRecord);
                    }

                    /** @brief The body block for this frame. */
                    const TerrainBodyRecord& body_record() const noexcept { return body_; }

                    /**
                     * @brief The slot cache, for the upload pass and the descriptor write.
                     * @pre @ref drawing is true; there is no pool before a body is set.
                     */
                    TileCache& cache() noexcept { return *cache_; }

                    /** @brief What the last selection did, for the editor's readout. */
                    const SushiEngine::Terrain::QuadtreeStatistics& statistics() const noexcept
                    {
                        return statistics_;
                    }

                    /** @brief Tiles staged by the last @ref prepare. */
                    std::uint32_t uploads() const noexcept { return uploads_; }

                    /** @brief Nodes drawing from an ancestor rather than their own tile. */
                    std::uint32_t inherited() const noexcept { return inherited_; }

                    /** @brief The layer stack, so the builder can edit this body's ground. */
                    SushiEngine::Terrain::LayerStack& layers() noexcept { return layers_; }

                private:
                    /** @brief One node that wanted its own tile and did not have one. */
                    struct Miss
                    {
                        std::size_t node = 0;
                        double distance = 0.0;
                    };

                    Vulkan::VulkanDevice& device_;
                    SushiEngine::Terrain::PlanetPack pack_;
                    SushiEngine::Terrain::PackHeightSource source_;
                    SushiEngine::Terrain::LayerStack layers_;
                    /**
                     * @brief The pool, created with the first pack and kept across bodies.
                     *
                     * A pointer only so that it need not exist: its image is sized from
                     * @ref PlanetTerrainDesc::slot_count and a view that never looks at a
                     * planet should never allocate it.
                     */
                    std::unique_ptr<TileCache> cache_;
                    PlanetTerrainDesc desc_;
                    int body_index_ = -1;

                    std::vector<SushiEngine::Terrain::TerrainNode> nodes_;
                    std::vector<TerrainNodeRecord> records_;
                    std::vector<Miss> misses_;
                    std::vector<float> scratch_;
                    TerrainBodyRecord body_{};
                    SushiEngine::Terrain::QuadtreeStatistics statistics_{};
                    std::uint32_t uploads_ = 0;
                    std::uint32_t inherited_ = 0;
            };
        } // namespace Terrain
    } // namespace Render
} // namespace SushiEngine
