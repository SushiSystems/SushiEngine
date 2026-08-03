/**************************************************************************/
/* terrain_authoring.hpp                                                  */
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
 * @file terrain_authoring.hpp
 * @brief One body's ground as something an author can change, and watch.
 *
 * A @ref SushiEngine::Terrain::LayerStack is a container: it will hold whatever records
 * are put in it and has no opinion about who is drawing the ground those records
 * describe. Whoever *is* drawing it holds a compiled copy of every tile the camera can
 * see, and an edit that only reaches the container leaves that copy stale — the ground
 * keeps the shape it had before the edit until the tile happens to be evicted.
 *
 * This interface is the difference. Every mutator here means "change the stack **and**
 * invalidate the ground that depended on it", which is the only edit an author ever
 * wants; the read side reports what the implementation's last frame did with the result.
 * Held in the terrain module rather than beside an implementation because more than one
 * thing legitimately implements it — the renderer's near-field body today, a headless
 * authority applying replicated edits after it — and they must offer one vocabulary.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/terrain/layer_stack.hpp>
#include <SushiEngine/terrain/quadtree.hpp>

namespace SushiEngine
{
    namespace Terrain
    {
        /**
         * @brief The editable ground of the body a view is currently looking at.
         *
         * Ordering is the vocabulary, not position: a layer is addressed by its
         * @ref TerrainLayer::order, because that is the identity composition depends on
         * and an index would silently mean something else after the layer before it was
         * removed. @ref layer walks the stack in composition order for display only.
         */
        class ITerrainAuthoring
        {
            public:
                virtual ~ITerrainAuthoring() = default;

                /** @brief The body being authored, as an ephemeris index; negative for none. */
                virtual int body() const noexcept = 0;

                /**
                 * @brief Whether that body has baked terrain behind it.
                 *
                 * False is not an error — a body with no pack falls back to analytic
                 * ground — but it does mean an edit has nothing to reshape, so an
                 * authoring surface should say so rather than accept layers into a void.
                 */
                virtual bool loaded() const noexcept = 0;

                /**
                 * @brief The body's mean radius, metres; zero when nothing is loaded.
                 *
                 * A footprint is angular so that it is body-independent (@ref
                 * LayerFootprint), but nobody authors a crater in radians. This is the one
                 * number that turns the record's angle into the metres an author means,
                 * and it comes from the implementation because the loaded pack is what
                 * knows the body's shape.
                 */
                virtual double mean_radius_metres() const noexcept = 0;

                /**
                 * @brief The ground the last frame was looking down at, as a unit direction.
                 *
                 * What "here" means to an author. Without it a footprint can only be typed
                 * in coordinates, and the one thing a builder always wants is to edit the
                 * ground in front of them.
                 *
                 * @return The direction from the body centre towards the last frame's
                 *         camera, or the polar axis when no frame has been prepared.
                 */
                virtual Vector3 view_direction() const noexcept = 0;

                /** @brief How many layers the stack holds. */
                virtual std::size_t layer_count() const noexcept = 0;

                /**
                 * @brief The layer at a position in composition order.
                 * @param index Position, below @ref layer_count.
                 * @return A copy of the record, so it cannot be edited behind the
                 *         implementation's back; an out-of-range index yields a default
                 *         layer rather than reading past the stack.
                 */
                virtual TerrainLayer layer(std::size_t index) const = 0;

                /**
                 * @brief Adds a layer and invalidates the ground under its footprint.
                 * @param layer The layer to add; its order must not already be present.
                 * @return true when it was added; false when that order is taken.
                 */
                virtual bool insert_layer(const TerrainLayer& layer) = 0;

                /**
                 * @brief Rewrites the layer holding an order, footprint and order included.
                 *
                 * Invalidates the ground under both footprints when the edit moved one,
                 * because the ground the layer *left* is as wrong as the ground it reached.
                 *
                 * @param order The order identifying the layer to rewrite.
                 * @param layer Its new value; @ref TerrainLayer::order may differ from
                 *              @p order, which moves the layer in composition order.
                 * @return true when it was rewritten; false when no layer held @p order, or
                 *         when the new order is a *different* one that is already taken.
                 */
                virtual bool update_layer(std::uint32_t order, const TerrainLayer& layer) = 0;

                /**
                 * @brief Removes the layer holding an order and invalidates its ground.
                 * @param order The order to remove.
                 * @return true when a layer was removed, false when none held that order.
                 */
                virtual bool remove_layer(std::uint32_t order) = 0;

                /**
                 * @brief Exchanges two layers' places in composition order.
                 *
                 * The move an author actually makes — "this crater goes on top of that
                 * flatten" — expressed so it cannot fail on a taken order, which a pair of
                 * @ref update_layer calls through an intermediate order can.
                 *
                 * @param first  One layer's order.
                 * @param second The other's.
                 * @return true when both existed and were exchanged; false otherwise, in
                 *         which case the stack is unchanged.
                 */
                virtual bool swap_layer_order(std::uint32_t first, std::uint32_t second) = 0;

                /** @brief Drops every layer, invalidating the ground all of them touched. */
                virtual void clear_layers() = 0;

                /**
                 * @brief What the implementation's last frame selected, for a readout.
                 *
                 * A copy of a per-frame value rather than a reference into one: the frame
                 * that produced it has already moved on, and a reader that held a reference
                 * would be reading a field being rewritten under it.
                 */
                virtual QuadtreeStatistics selection_statistics() const noexcept = 0;

                /**
                 * @brief Tiles still carrying pre-edit ground, waiting to be recompiled.
                 *
                 * Recompilation is bounded per frame like every other tile operation, so an
                 * edit over a wide footprint resolves over several frames. Non-zero means
                 * what is on screen is not yet what the stack says, which is worth showing
                 * rather than leaving an author to wonder whether the edit took.
                 */
                virtual std::size_t pending_recompile_count() const noexcept = 0;
        };
    } // namespace Terrain
} // namespace SushiEngine
