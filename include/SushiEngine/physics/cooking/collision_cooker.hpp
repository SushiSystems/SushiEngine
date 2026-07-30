/**************************************************************************/
/* collision_cooker.hpp                                                   */
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
 * @file collision_cooker.hpp
 * @brief §8.4: a mesh in, a `.sushicollision` out, with the error measured.
 *
 * Five stages, each an `ICookingStage` in a list, so a sixth is an object inserted
 * rather than a case added (§4.2): repair, mass properties, geometry, distance field,
 * serialize. Only the third differs between the two shapes an asset can take — convex
 * pieces for a dynamic body, a triangle hierarchy for authored-static geometry, which is
 * both cheaper and exact (§8.4 item 4).
 *
 * The cache check is the cooker's, not the caller's, so "an unchanged mesh with unchanged
 * parameters is never re-cooked" is a property of the pipeline rather than of every call
 * site remembering to ask.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/physics/cooking/collision_asset.hpp>
#include <SushiEngine/physics/cooking/cooker_interface.hpp>
#include <SushiEngine/physics/cooking/convex_decomposition.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /**
             * @brief The `CollisionCooker`'s output-format version.
             *
             * Bumped whenever the produced bytes would differ for the same input, which is
             * what invalidates every cached asset it ever wrote. It is the one component of
             * the cache key that no amount of hashing the inputs can derive.
             */
            constexpr std::uint32_t COLLISION_COOKER_VERSION = 1;

            /**
             * @brief Turns an imported mesh into the shape a body collides as.
             *
             * Stateless apart from the thresholds, and safe to run on a background thread:
             * §8.1 puts cooking off the main thread while the editor keeps drawing. Two
             * cooks on two threads must use two cookers, since @ref cook is not reentrant.
             */
            class CollisionCooker final : public IMeshCooker
            {
            public:
                const char* name() const noexcept override { return "CollisionCooker"; }
                std::uint32_t version() const noexcept override
                {
                    return COLLISION_COOKER_VERSION;
                }
                CookedAssetKind kind() const noexcept override
                {
                    return CookedAssetKind::Collision;
                }

                CookedAssetKey cache_key(const Geometry::TriangleMeshView& mesh,
                                         const CookingParameters& parameters)
                    const noexcept override;

                CookingReport cook(const Geometry::TriangleMeshView& mesh,
                                   const CookingParameters& parameters, ICookedAssetStore* store,
                                   ICookingProgressSink* progress,
                                   std::vector<std::byte>& out) override;

                /**
                 * @brief Sets the limits a produced asset is judged against (§8.5).
                 *
                 * @param thresholds The project's limits; defaults reject what is
                 *                   unambiguously broken and pass what is merely imperfect.
                 */
                void set_thresholds(const CookingThresholds& thresholds) noexcept
                {
                    thresholds_ = thresholds;
                }

                /** @brief The limits currently applied. */
                const CookingThresholds& thresholds() const noexcept { return thresholds_; }

            private:
                CookingThresholds thresholds_;
            };

            /**
             * @brief The state a collision cook's stages pass between them.
             *
             * Exposed rather than private to the translation unit, because a project adding
             * a sixth stage needs to name what that stage reads — which is §4.2's whole
             * point, and a context hidden in a `.cpp` closes the extension point it exists
             * to open.
             */
            struct CollisionCookContext
            {
                /** @brief The source geometry as handed in. */
                Geometry::TriangleMeshView source;

                /** @brief The dial, already resolved. */
                DerivedCookingParameters parameters;

                /** @brief The repaired source; every later stage reads this, not @ref source. */
                Geometry::TriangleMesh repaired;

                /** @brief A distance hierarchy over @ref repaired, for every accuracy measure. */
                Geometry::MeshDistanceQuery surface;

                /** @brief The convex pieces, empty for a static cook. */
                std::vector<ConvexPiece> pieces;

                /** @brief The asset being assembled. */
                CollisionAsset asset;

                /** @brief Filled as the stages learn things; returned to the caller. */
                CookingReport report;
            };
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
