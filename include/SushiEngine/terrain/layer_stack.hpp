/**************************************************************************/
/* layer_stack.hpp                                                        */
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
 * @file layer_stack.hpp
 * @brief What reshapes measured ground: an ordered stack of small authored records.
 *
 * A layer is a *record*, not a raster (`docs/slop/solar_system_overhaul.md` §6.1). A
 * crater is a direction, a radius, and a profile; a building pad is a flatten. The
 * record is bytes rather than megabytes, which is what lets an edit replicate over a
 * network, serialise into a scene, and be undone — none of which storing edited height
 * rasters would allow.
 *
 * Composition order is the load-bearing detail. @ref TerrainLayer::order is explicit and
 * unique within a stack, so the composed ground is a pure function of the *set* of
 * layers and not of the sequence they arrived in. That is what makes the authoritative
 * height reproducible on a server and a client that received the same edits in different
 * orders, and it is why @ref LayerStack::insert refuses a duplicate order rather than
 * quietly breaking the tie by arrival.
 *
 * The stack is a sorted vector. A loose quadtree over the sphere is the right index once
 * there are enough layers for the linear scan to matter; building one before there is a
 * consumer would be a structure guessing at its own access pattern.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Terrain
    {
        /** @brief What a layer does to the ground inside its footprint. */
        enum class LayerOperation : std::uint8_t
        {
            /** @brief Pulls the ground toward a level: a building pad, a road bed. */
            Flatten = 0,
            /** @brief Adds to the ground: a berm, a spoil heap, a dam wall. */
            Raise,
            /** @brief A bowl with a raised rim and a decaying ejecta blanket. */
            Crater,
        };

        /**
         * @brief Where on a body a layer acts: a disc, in angle from a direction.
         *
         * Angular rather than metric so a footprint is body-independent and needs no
         * projection to test. The disc is P0's one shape; polylines and polygons are the
         * builder's shapes and land with it, behind this same struct's role in the stack.
         */
        struct LayerFootprint
        {
            /** @brief Unit direction from the body centre to the footprint's centre. */
            Vector3 direction{Vector3{0.0, 0.0, 1.0}};
            /** @brief Angular radius the operation acts at full strength within. */
            double inner_radians = 0.0;
            /** @brief Angular radius the operation has faded to nothing by. */
            double outer_radians = 0.0;
        };

        /** @brief The numbers an operation reads; each uses the fields it needs. */
        struct LayerProfile
        {
            /** @brief Flatten: the elevation the ground is pulled to, metres. */
            double target_metres = 0.0;
            /** @brief Raise: the elevation added at full strength, metres. */
            double amount_metres = 0.0;
            /** @brief Crater: how far the bowl's floor sits below the ground, metres. */
            double depth_metres = 0.0;
            /** @brief Crater: how far the rim stands above the ground, metres. */
            double rim_metres = 0.0;
        };

        /** @brief One authored edit to a body's ground. */
        struct TerrainLayer
        {
            /**
             * @brief Composition order; unique within a stack, low applied first.
             *
             * Explicit because composition that depended on the order a container
             * happened to return would make the authoritative height non-reproducible,
             * and the whole determinism posture rests on it being reproducible.
             */
            std::uint32_t order = 0;
            LayerOperation operation = LayerOperation::Flatten;
            LayerFootprint footprint{};
            LayerProfile profile{};
        };

        /**
         * @brief Angle between two unit directions, radians.
         * @param a First unit direction.
         * @param b Second unit direction.
         * @return The angle, within [0, pi]; the dot product is clamped first so a
         *         rounding excursion past one cannot produce a NaN.
         */
        inline double angular_distance(const Vector3& a, const Vector3& b) noexcept
        {
            double cosine = dot(a, b);
            cosine = cosine < -1.0 ? -1.0 : (cosine > 1.0 ? 1.0 : cosine);
            return std::acos(cosine);
        }

        /**
         * @brief A layer's strength at an angular distance from its centre.
         *
         * One inside the inner radius, zero past the outer, and a smoothstep between, so
         * the edit meets the surrounding ground with a continuous slope rather than a
         * step a normal would catch on.
         *
         * @param footprint The layer's footprint.
         * @param angle     Angular distance from its centre, radians.
         * @return Strength within [0, 1].
         */
        inline double layer_strength(const LayerFootprint& footprint, double angle) noexcept
        {
            if (angle <= footprint.inner_radians)
                return 1.0;
            if (angle >= footprint.outer_radians ||
                footprint.outer_radians <= footprint.inner_radians)
                return 0.0;
            const double span = footprint.outer_radians - footprint.inner_radians;
            const double x = (angle - footprint.inner_radians) / span;
            return 1.0 - x * x * (3.0 - 2.0 * x);
        }

        /**
         * @brief One layer applied to one elevation.
         *
         * Separated from the stack so a shader port has a single function to mirror and a
         * test has a single function to pin.
         *
         * @param layer            The layer.
         * @param direction        Unit direction of the point being evaluated.
         * @param elevation_metres The elevation before this layer.
         * @return The elevation after it; unchanged outside the footprint.
         */
        inline double apply_layer(const TerrainLayer& layer, const Vector3& direction,
                                  double elevation_metres) noexcept
        {
            const double angle = angular_distance(direction, layer.footprint.direction);
            if (angle >= layer.footprint.outer_radians)
                return elevation_metres;

            switch (layer.operation)
            {
                case LayerOperation::Flatten:
                {
                    const double strength = layer_strength(layer.footprint, angle);
                    return elevation_metres +
                           (layer.profile.target_metres - elevation_metres) * strength;
                }
                case LayerOperation::Raise:
                {
                    const double strength = layer_strength(layer.footprint, angle);
                    return elevation_metres + layer.profile.amount_metres * strength;
                }
                case LayerOperation::Crater:
                {
                    // A parabolic bowl inside the rim radius and a decaying ejecta
                    // blanket outside it, meeting at the rim height. Continuous at both
                    // joins by construction, which is what a normal derived from the
                    // result needs and what a piecewise profile most easily gets wrong.
                    const double rim_radius = layer.footprint.inner_radians;
                    if (rim_radius <= 0.0 || layer.footprint.outer_radians <= rim_radius)
                        return elevation_metres;
                    if (angle <= rim_radius)
                    {
                        const double x = angle / rim_radius;
                        const double bowl = x * x;
                        return elevation_metres - layer.profile.depth_metres * (1.0 - bowl) +
                               layer.profile.rim_metres * bowl;
                    }
                    const double outside = (layer.footprint.outer_radians - angle) /
                                           (layer.footprint.outer_radians - rim_radius);
                    return elevation_metres + layer.profile.rim_metres * outside * outside;
                }
            }
            return elevation_metres;
        }

        /**
         * @brief A body's ordered edits, and the ground they compose.
         *
         * Held sorted by @ref TerrainLayer::order, so iteration order is composition
         * order and neither depends on how the layers arrived.
         */
        class LayerStack
        {
            public:
                /**
                 * @brief Adds a layer, keeping the stack ordered.
                 * @param layer The layer to add; its order must not already be present.
                 * @return true when it was added; false when a layer already holds that
                 *         order, which is a caller error rather than a race — orders are
                 *         assigned by whoever authors the edit.
                 */
                bool insert(const TerrainLayer& layer)
                {
                    const auto position =
                        std::lower_bound(layers_.begin(), layers_.end(), layer.order,
                                         [](const TerrainLayer& entry, std::uint32_t order)
                                         { return entry.order < order; });
                    if (position != layers_.end() && position->order == layer.order)
                        return false;
                    layers_.insert(position, layer);
                    return true;
                }

                /**
                 * @brief Removes the layer holding an order.
                 * @param order The order to remove.
                 * @return true when a layer was removed, false when none held that order.
                 */
                bool remove(std::uint32_t order)
                {
                    const auto position =
                        std::lower_bound(layers_.begin(), layers_.end(), order,
                                         [](const TerrainLayer& entry, std::uint32_t value)
                                         { return entry.order < value; });
                    if (position == layers_.end() || position->order != order)
                        return false;
                    layers_.erase(position);
                    return true;
                }

                /** @brief Drops every layer. */
                void clear() { layers_.clear(); }

                /** @brief How many layers the stack holds. */
                std::size_t size() const noexcept { return layers_.size(); }

                /**
                 * @brief The layer at a position in composition order.
                 * @param index Position, below @ref size().
                 * @return The layer.
                 */
                const TerrainLayer& at(std::size_t index) const { return layers_[index]; }

                /**
                 * @brief Every layer applied to one elevation, in order.
                 * @param direction        Unit direction of the point being evaluated.
                 * @param elevation_metres The measured elevation before any edit.
                 * @return The authoritative elevation after the stack.
                 */
                double apply(const Vector3& direction, double elevation_metres) const
                {
                    double elevation = elevation_metres;
                    for (const TerrainLayer& layer : layers_)
                        elevation = apply_layer(layer, direction, elevation);
                    return elevation;
                }

                /**
                 * @brief Whether any layer reaches into a spherical cap.
                 *
                 * The question a tile compile asks before walking its samples: a tile no
                 * layer touches keeps its measured heights untouched and skips the walk.
                 *
                 * @param direction      Unit direction of the cap's centre.
                 * @param radius_radians Angular radius of the cap.
                 * @return true when at least one layer's footprint overlaps it.
                 */
                bool overlaps(const Vector3& direction, double radius_radians) const
                {
                    for (const TerrainLayer& layer : layers_)
                    {
                        const double separation =
                            angular_distance(direction, layer.footprint.direction);
                        if (separation < radius_radians + layer.footprint.outer_radians)
                            return true;
                    }
                    return false;
                }

            private:
                std::vector<TerrainLayer> layers_;
        };
    } // namespace Terrain
} // namespace SushiEngine
