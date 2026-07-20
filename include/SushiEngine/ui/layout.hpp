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

#include <cmath>

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

        /**
         * @brief The inverse of @ref resolve_axis: rewrites a transform's axis from a target rect.
         *
         * Given the pixel rect an axis should now occupy, backs out the `anchored`
         * offset and `size_delta` that make @ref resolve_axis reproduce it — the
         * formula a screen-space drag/resize edit uses to write back into a
         * `RectTransform` without disturbing its anchors/pivot.
         *
         * @param parent_min    Parent rectangle origin on this axis.
         * @param parent_size   Parent rectangle length on this axis.
         * @param anchor_min    Lower anchor, a fraction of the parent length.
         * @param anchor_max    Upper anchor, a fraction of the parent length.
         * @param pivot         The element's pivot on this axis, a fraction of its length.
         * @param target_min    The desired element origin on this axis.
         * @param target_size   The desired element length on this axis.
         * @param out_anchored  Receives the anchored offset reproducing @p target_min/@p target_size.
         * @param out_size_delta Receives the size delta reproducing @p target_size.
         */
        inline void resolve_axis_inverse(Scalar parent_min, Scalar parent_size, Scalar anchor_min,
                                          Scalar anchor_max, Scalar pivot, Scalar target_min,
                                          Scalar target_size, Scalar& out_anchored,
                                          Scalar& out_size_delta) noexcept
        {
            const Scalar anchor_ref_min = parent_min + anchor_min * parent_size;
            const Scalar anchor_ref_max = parent_min + anchor_max * parent_size;
            const Scalar span = anchor_ref_max - anchor_ref_min;
            out_size_delta = target_size - span;
            const Scalar pivot_position = target_min + pivot * target_size;
            out_anchored = pivot_position - anchor_ref_min - pivot * span;
        }

        /**
         * @brief Rewrites @p transform's `anchored_position`/`size_delta` so it resolves to @p target.
         *
         * The inverse of @ref resolve_rect: keeps the transform's anchors and pivot as
         * authored and backs out the pixel offsets that reproduce @p target against
         * @p parent — the seam a screen-space drag or resize handle writes through.
         *
         * @param parent Parent rectangle @p target is expressed against.
         * @param target The desired resolved rectangle.
         * @param transform The transform to rewrite in place.
         */
        inline void apply_screen_rect(const Rect& parent, const Rect& target,
                                       RectTransform& transform) noexcept
        {
            resolve_axis_inverse(parent.min.x, parent.size.x, transform.anchor_min.x,
                                  transform.anchor_max.x, transform.pivot.x, target.min.x,
                                  target.size.x, transform.anchored_position.x,
                                  transform.size_delta.x);
            resolve_axis_inverse(parent.min.y, parent.size.y, transform.anchor_min.y,
                                  transform.anchor_max.y, transform.pivot.y, target.min.y,
                                  target.size.y, transform.anchored_position.y,
                                  transform.size_delta.y);
        }

        /**
         * @brief Resolves a Canvas's own rect against the actual screen size.
         *
         * `ConstantPixelSize` simply fills the screen (today's behaviour). In
         * `ScaleWithScreenSize`, children are laid out against a rect scaled by
         * Unity `CanvasScaler`'s formula: the log-average of the width ratio and the
         * height ratio (actual / reference), weighted by `match_width_or_height`, so
         * the canvas presents a consistent apparent size across resolutions.
         *
         * @param screen_size The actual screen (or panel) size in pixels.
         * @param canvas      The canvas's scale settings.
         * @return The rect child elements resolve their anchors against.
         */
        inline Rect resolve_canvas_rect(const Vector2& screen_size, const Canvas& canvas) noexcept
        {
            if (canvas.scale_mode == CanvasScaleMode::ConstantPixelSize ||
                canvas.reference_size.x <= 0 || canvas.reference_size.y <= 0 ||
                screen_size.x <= 0 || screen_size.y <= 0)
                return Rect{Vector2{0, 0}, screen_size};

            const Scalar width_ratio = screen_size.x / canvas.reference_size.x;
            const Scalar height_ratio = screen_size.y / canvas.reference_size.y;
            const Scalar log_width = std::log(static_cast<double>(width_ratio));
            const Scalar log_height = std::log(static_cast<double>(height_ratio));
            const Scalar blended_log =
                log_width + (log_height - log_width) * canvas.match_width_or_height;
            const Scalar scale = static_cast<Scalar>(std::exp(static_cast<double>(blended_log)));
            const Scalar effective_width = scale > 0 ? screen_size.x / scale : canvas.reference_size.x;
            const Scalar effective_height = scale > 0 ? screen_size.y / scale : canvas.reference_size.y;
            return Rect{Vector2{0, 0}, Vector2{effective_width, effective_height}};
        }
    } // namespace UI
} // namespace SushiEngine
