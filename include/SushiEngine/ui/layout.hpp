/**************************************************************************/
/* layout.hpp                                                            */
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
 * @file layout.hpp
 * @brief The UI layout resolution: a `RectTransform` against a parent rect.
 *
 * `resolve_rect` is the whole of the anchor model, kept a pure function of a parent
 * rectangle and a `RectTransform` so it is trivially testable and free of any ECS
 * dependency. The `UI` façade calls it once per element, parents before children,
 * to fill every `ComputedRect`.
 */

#include <SushiEngine/ui/components.hpp>
#include <SushiEngine/ui/rect.hpp>

namespace SushiEngine
{
    namespace UI
    {
        /**
         * @brief Resolves one axis of a `RectTransform` against its parent extent.
         *
         * The 1D core of the UGUI model: given the parent's origin and length on an
         * axis and the transform's anchors/pivot/offset/size for that axis, returns the
         * element's origin (min) and length on that axis.
         *
         * @param parent_min   Parent rectangle origin on this axis.
         * @param parent_size  Parent rectangle length on this axis.
         * @param anchor_min   Lower anchor, a fraction of the parent length.
         * @param anchor_max   Upper anchor, a fraction of the parent length.
         * @param pivot        The element's pivot on this axis, a fraction of its length.
         * @param anchored     Offset of the pivot from the anchor, in pixels.
         * @param size_delta   Size added to the anchor span on this axis.
         * @param out_min      Receives the element origin on this axis.
         * @param out_size     Receives the element length on this axis.
         */
        inline void resolve_axis(Scalar parent_min, Scalar parent_size, Scalar anchor_min,
                                 Scalar anchor_max, Scalar pivot, Scalar anchored,
                                 Scalar size_delta, Scalar& out_min, Scalar& out_size) noexcept
        {
            const Scalar anchor_ref_min = parent_min + anchor_min * parent_size;
            const Scalar anchor_ref_max = parent_min + anchor_max * parent_size;
            const Scalar span = anchor_ref_max - anchor_ref_min;
            out_size = span + size_delta;
            const Scalar pivot_position = anchor_ref_min + pivot * span + anchored;
            out_min = pivot_position - pivot * out_size;
        }

        /**
         * @brief Resolves a `RectTransform` into an absolute screen rectangle.
         *
         * Applies @ref resolve_axis on x and y independently, exactly reproducing
         * Unity `RectTransform`: with a point anchor the element's size is `size_delta`
         * and its pivot sits at the anchor plus `anchored_position`; with a stretched
         * anchor `size_delta` insets or grows the stretched span.
         *
         * @param parent    The parent element's already-resolved rectangle.
         * @param transform The element's layout parameters.
         * @return The element's rectangle in the same (screen) space as @p parent.
         */
        inline Rect resolve_rect(const Rect& parent, const RectTransform& transform) noexcept
        {
            Rect result;
            resolve_axis(parent.min.x, parent.size.x, transform.anchor_min.x,
                         transform.anchor_max.x, transform.pivot.x, transform.anchored_position.x,
                         transform.size_delta.x, result.min.x, result.size.x);
            resolve_axis(parent.min.y, parent.size.y, transform.anchor_min.y,
                         transform.anchor_max.y, transform.pivot.y, transform.anchored_position.y,
                         transform.size_delta.y, result.min.y, result.size.y);
            return result;
        }
    } // namespace UI
} // namespace SushiEngine
