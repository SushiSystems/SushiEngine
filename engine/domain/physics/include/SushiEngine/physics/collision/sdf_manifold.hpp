/**************************************************************************/
/* sdf_manifold.hpp                                                       */
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
 * @file sdf_manifold.hpp
 * @brief Colliding a convex shape with a cooked signed-distance field (§7.5).
 *
 * A signed-distance field is not a bounded convex set, so GJK/EPA does not
 * apply to it — there is no "furthest point along a direction" for an implicit
 * volume. What it offers instead is the thing a triangle mesh's narrowphase
 * (`mesh_manifold.hpp`) has to reconstruct with a heuristic: an unambiguous,
 * global inside/outside answer and a gradient that is the outward surface
 * normal *by construction* (the eikonal property `|grad d| = 1`), valid
 * whether the query point is outside the solid or already deep inside it.
 * `mesh_manifold.hpp`'s own comment names the case this does not have to
 * guess at — "which side of the surface the shape is on" — because the field
 * already encodes it everywhere, not just near whichever triangle a query
 * happened to land closest to.
 *
 * The routine here is deliberately a single-point manifold, the same scope
 * `generate_sphere_sphere_manifold` and `generate_obb_sphere_manifold` keep:
 * two passes of the field, no clipping. The first pass samples at the shape's
 * own centre to get a rough outward direction; the second refines at the
 * shape's support point in that direction, which is the point actually likely
 * to be touching. A multi-point patch (a box resting flat on a field-backed
 * surface) is future work behind the same seam, not a correctness gap in what
 * ships here — a single point still pushes the shape out correctly, exactly as
 * `generate_obb_sphere_manifold` already does for a shape resting on a sphere.
 */

#include <cstdint>
#include <limits>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Manifold between a convex shape and a signed-distance field.
         *
         * @param shape           The convex shape; any type with a `support()`
         *                        overload (`physics/geometry/shapes.hpp`).
         * @param field           The field, placed in the world.
         * @param center          The shape's body centre, for the anchor frame.
         * @param orientation     The shape's body orientation.
         * @param contact_offset  Contacts are generated out to this separation (§7.6).
         * @return A manifold with `point_count == 0` when the shape is further
         *         from the field's surface than @p contact_offset, or when the
         *         field is empty.
         */
        template <typename T, typename Shape>
        inline ContactManifold<T> generate_convex_sdf_manifold(const Shape& shape,
                                                               const SDFCollider<T>& field,
                                                               const Vector3T<T>& center,
                                                               const QuaternionT<T>& orientation,
                                                               T contact_offset = T(0)) noexcept
        {
            if (field.distances == nullptr || field.resolution <= 0)
                return ContactManifold<T>{};

            // Pass 1: a rough outward direction from the shape's own centre. The
            // shape's point most likely touching the surface is its support point
            // toward the solid — away from the outward gradient.
            const Vector3T<T> rough_gradient = sdf_gradient_world(field, center);
            const Vector3T<T> probe = support(shape, rough_gradient * T(-1));

            // Pass 2: refine at the actual candidate point rather than trusting the
            // centre's sample, which can be a body-width away from the surface.
            const T distance = sdf_sample_world(field, probe);
            if (distance == std::numeric_limits<T>::max() || distance > contact_offset)
                return ContactManifold<T>{};

            const Vector3T<T> gradient = sdf_gradient_world(field, probe);
            // `probe` sits `distance` along `gradient` from the surface (negative
            // when already inside), so stepping back by exactly that reaches it.
            const Vector3T<T> point_on_field = probe - gradient * distance;
            // Pair convention: normal runs from the shape (a) toward the field's
            // solid (b), which is the inward direction, i.e. the negated gradient.
            const Vector3T<T> normal = gradient * T(-1);

            return make_point_manifold(normal, probe, point_on_field, distance, center,
                                       orientation, field.center, field.orientation,
                                       make_feature_id(0, 0, 0, false));
        }
    } // namespace Physics
} // namespace SushiEngine
