/**************************************************************************/
/* skin_vertex.hpp                                                        */
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
 * @file skin_vertex.hpp
 * @brief The per-vertex skinning stream — the parallel buffer that never touches MeshVertex.
 *
 * Skinning attributes ride in their own stream (design §6.1), not as a `MeshVertex`
 * change, so the fifteen passes bound to the base 60-byte format are untouched. One
 * @ref SkinVertex per base vertex names up to four influencing joints and their weights;
 * the skinning dispatch reads it beside the base positions. Joint indices are in the
 * *cooked skeleton's* order, not the glTF skin's — @ref remap_from_order does that
 * remap at import, because the skeleton cook topologically re-sorts joints.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Influences blended per skinned vertex in A1 (Ultra's eight lands with A7). */
        constexpr std::uint32_t SKIN_INFLUENCES = 4;

        /**
         * @brief One base vertex's skinning weights: four joints and their normalized weights.
         *
         * 12 bytes, its own stream. @c joints index the cooked skeleton (`SkeletonView`
         * order); @c weights are unorm8 (0..255) and are authored to sum to ~255, so the
         * dispatch can divide by 255 and trust the total. A zero-weight influence names
         * joint 0 by convention and contributes nothing.
         */
        struct SkinVertex
        {
            std::uint16_t joints[SKIN_INFLUENCES] = {0, 0, 0, 0};
            std::uint8_t weights[SKIN_INFLUENCES] = {0, 0, 0, 0};
        };

        static_assert(sizeof(SkinVertex) == 12, "SkinVertex must be 12 tightly packed bytes");

        /**
         * @brief Inverts a skeleton cook's sort order into a joint-index remap table.
         *
         * @ref build_skeleton_blob returns `order`, where `order[new_index]` is the original
         * (glTF-skin) joint index. Skin vertices carry original indices, so they must be
         * remapped to the new order before they address the cooked skeleton's palette. This
         * builds `remap[original] = new_index`.
         *
         * @param order The sort order from @ref build_skeleton_blob (new index to original).
         * @return The inverse table: original joint index to its index in the cooked skeleton.
         */
        inline std::vector<std::uint16_t> remap_from_order(const std::vector<int>& order)
        {
            std::vector<std::uint16_t> remap(order.size());
            for (std::size_t new_index = 0; new_index < order.size(); ++new_index)
                remap[static_cast<std::size_t>(order[new_index])] =
                    static_cast<std::uint16_t>(new_index);
            return remap;
        }
    } // namespace Animation
} // namespace SushiEngine
