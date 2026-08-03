/**************************************************************************/
/* soft_body_cooker.hpp                                                   */
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
 * @file soft_body_cooker.hpp
 * @brief §8.3: a mesh in, a `.sushisoft` out — the user's headline feature.
 *
 * *"mesh'i projeye attıktan sonra otomatik olarak soft body optimized bake eden bir
 * motor, ve bunun hassasiyetini ayarlayabiliyoruz."* This is the bake half of that
 * sentence; the fidelity dial is the other half and lives in `cooking_parameters.hpp`.
 *
 * §8.3 numbers ten stages and this cooker exposes six `ICookingStage` objects, which is
 * a grouping rather than an omission — the mapping is stated once here so nobody has to
 * count:
 *
 * | §8.3 | Stage object |
 * |---|---|
 * | 1 Repair | `Repair` |
 * | 2 Voxelize, 3 Tetrahedralize, 4 Optimize, 5 Rest state, 7 Surface set | `Tetrahedralize` |
 * | 6 Embed the render mesh | `Embed` |
 * | 7's hierarchy, 8 Bake the rest distance field | `BakeRestShape` |
 * | 9 Build levels of detail | `LevelsOfDetail` |
 * | 10 Serialize | `Serialize` |
 *
 * Stages 2 through 5 and 7 are one object because they share the voxel grid, and
 * splitting them across the seam would mean publishing that grid as an interface — which
 * is a worse violation of §3.2 than the grouping is of §8.3's numbering.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/physics/cooking/cooker_interface.hpp>
#include <SushiEngine/physics/cooking/soft_body_asset.hpp>
#include <SushiEngine/physics/cooking/tetrahedral_mesh.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /** @brief The `SoftBodyCooker`'s output-format version (see §8.1's cache key). */
            constexpr std::uint32_t SOFT_BODY_COOKER_VERSION = 1;

            /**
             * @brief Turns an imported mesh into a simulable tetrahedral body.
             *
             * Stateless apart from the thresholds, and safe on a background thread; two
             * concurrent cooks want two cookers, since @ref cook is not reentrant.
             */
            class SoftBodyCooker final : public IMeshCooker
            {
            public:
                const char* name() const noexcept override { return "SoftBodyCooker"; }
                std::uint32_t version() const noexcept override
                {
                    return SOFT_BODY_COOKER_VERSION;
                }
                CookedAssetKind kind() const noexcept override
                {
                    return CookedAssetKind::SoftBody;
                }

                CookedAssetKey cache_key(const Geometry::TriangleMeshView& mesh,
                                         const CookingParameters& parameters)
                    const noexcept override;

                CookingReport cook(const Geometry::TriangleMeshView& mesh,
                                   const CookingParameters& parameters, ICookedAssetStore* store,
                                   ICookingProgressSink* progress,
                                   std::vector<std::byte>& out) override;

                /** @brief Sets the limits a produced asset is judged against (§8.5). */
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
             * @brief The state a soft-body cook's stages pass between them.
             *
             * Public for the same reason the collision cook's context is: a project adding a
             * stage has to be able to name what that stage reads, and a context hidden in a
             * translation unit closes the extension point §4.2 exists to open.
             */
            struct SoftBodyCookContext
            {
                /** @brief The source geometry as handed in. */
                Geometry::TriangleMeshView source;

                /** @brief The dial, already resolved. */
                DerivedCookingParameters parameters;

                /** @brief The parameters as authored, for the copy the blob carries. */
                CookingParameters authored;

                /** @brief The repaired source; every later stage reads this. */
                Geometry::TriangleMesh repaired;

                /** @brief A distance hierarchy over @ref repaired. */
                Geometry::MeshDistanceQuery surface;

                /** @brief One simulation mesh per level, finest first. */
                std::vector<TetrahedralMesh> levels;

                /**
                 * @brief The resolution the finest level actually used.
                 *
                 * The coarser levels halve *this* rather than the authored number, because the
                 * element-count target may already have moved it — see `coarser_options`.
                 */
                std::int32_t base_resolution = 0;

                /** @brief The asset being assembled. */
                SoftBodyAsset asset;

                /** @brief Filled as the stages learn things; returned to the caller. */
                CookingReport report;
            };
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
