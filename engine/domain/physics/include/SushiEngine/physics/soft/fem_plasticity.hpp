/**************************************************************************/
/* fem_plasticity.hpp                                                     */
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
 * @file fem_plasticity.hpp
 * @brief §9.4's permanent dent: relaxing an element's rest state toward its
 *        current shape once it yields.
 *
 * `fem_element.hpp` already separates the *elastic* rest inverse
 * (`rest_inverse_column_0/1/2`, cooked once and never touched again) from
 * the *plastic* one (`plastic_inverse_column_0/1/2`, what every projection
 * in `fem_projection.hpp` actually reads) — which is the multiplicative
 * decomposition `F = F_elastic * F_plastic` stated as "which matrix do the
 * constraints measure against" rather than as two matrices multiplied
 * together every tick. Yielding is then exactly what §9.4 says it is:
 * writing a new plastic rest matrix, nothing else.
 *
 * **The update, derived rather than guessed.** Write `Ds` for the element's
 * current shape matrix (the three current edge vectors as columns) and `Dp`
 * for the plastic rest matrix (`plastic_inverse_column_*`'s own inverse), so
 * `F_elastic = Ds * Dp^-1`. Creeping a fraction `c` of the way from
 * `F_elastic` toward the identity means choosing a new `Dp'` such that
 * `Ds * Dp'^-1 = (1-c) * F_elastic + c * I`. Substituting `Ds = F_elastic *
 * Dp` and solving:
 *
 * ```
 *   Dp'^-1 = Dp^-1 * F_elastic^-1 * [(1-c) F_elastic + c I]
 *          = (1-c) * Dp^-1 + c * (Dp^-1 * F_elastic^-1)
 *          = (1-c) * Dp^-1 + c * Ds^-1              (since F_elastic * Dp = Ds)
 * ```
 *
 * a plain per-column blend of the current plastic inverse toward the current
 * shape's own inverse — which is why `invert_fem_matrix3` exists: nothing
 * cooks `Ds^-1` in advance, because `Ds` is a different matrix every tick.
 * At `c = 1` this sets `Dp'^-1 = Ds^-1` exactly, i.e. the current shape
 * *becomes* the rest shape and the elastic strain the constraints see drops
 * to zero in one step — full, instant creep, the sensible limit.
 *
 * **Scope.** This runs once per tick, alongside the stress readout it reads
 * from, not once per sub-step the way §9.4's pseudocode is written — a
 * deliberate simplification for this first implementation (creep is a slow
 * process relative to a sub-step, and running it there would mean inverting
 * a live shape matrix for every yielding element up to `substeps` times a
 * tick for no behavioural difference a reasonable creep rate would show).
 * `plastic_creep` is read as the fraction of the current excess strain that
 * becomes permanent *per tick* it stays over yield, not as a rate with units
 * of inverse time; scaling it by the tick length is future refinement, not a
 * correctness gap this file's tests depend on being absent.
 */

#include <cmath>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/soft/fem_element.hpp>
#include <SushiEngine/physics/soft/fem_projection.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Applies one tick's worth of plastic creep to an element, if it yielded.
         *
         * A no-op when the material never yields (`maximum_plastic_strain <= 0` or
         * `plastic_creep <= 0`), when the element has already hardened
         * (`accumulated_plastic_strain` at its ceiling), when this tick's von Mises
         * stress (already computed by @ref tetrahedron_von_mises_stress) is under
         * `yield_stress`, or when the element has collapsed and has no shape matrix
         * to invert.
         *
         * @param bodies   The owning solver's particle array.
         * @param element  The element; its plastic rest inverse, rest volume, and
         *                 accumulated strain are updated in place.
         * @param material The constitutive parameters `yield_stress`,
         *                 `plastic_creep`, and `maximum_plastic_strain` come from.
         */
        template <typename T>
        inline void apply_fem_plasticity(const RigidBodyT<T>* bodies, FEMTetrahedronT<T>& element,
                                         const SoftBodyMaterialT<T>& material) noexcept
        {
            if (!(material.plastic_creep > T(0)) || !(material.maximum_plastic_strain > T(0)))
                return;
            if (element.accumulated_plastic_strain >= material.maximum_plastic_strain)
                return;
            if (!(element.von_mises_stress > material.yield_stress))
                return;

            const Vector3T<T>& x0 = bodies[element.vertex[0]].position;
            const Vector3T<T> edge1 = bodies[element.vertex[1]].position - x0;
            const Vector3T<T> edge2 = bodies[element.vertex[2]].position - x0;
            const Vector3T<T> edge3 = bodies[element.vertex[3]].position - x0;

            FEMMatrix3<T> current_shape;
            current_shape.column0 = edge1;
            current_shape.column1 = edge2;
            current_shape.column2 = edge3;
            FEMMatrix3<T> current_shape_inverse;
            if (!invert_fem_matrix3(current_shape, current_shape_inverse))
                return; // a collapsed element: nothing safe to creep toward

            const FEMMatrix3<T> elastic = tetrahedron_deformation_gradient(
                edge1, edge2, edge3, element.plastic_inverse_column_0,
                element.plastic_inverse_column_1, element.plastic_inverse_column_2);
            const Vector3T<T> deviation0 = elastic.column0 - Vector3T<T>{T(1), T(0), T(0)};
            const Vector3T<T> deviation1 = elastic.column1 - Vector3T<T>{T(0), T(1), T(0)};
            const Vector3T<T> deviation2 = elastic.column2 - Vector3T<T>{T(0), T(0), T(1)};
            const T strain_magnitude = std::sqrt(
                dot(deviation0, deviation0) + dot(deviation1, deviation1) + dot(deviation2, deviation2));
            if (!(strain_magnitude > T(0)))
                return;

            // Clamped so the accumulator never overshoots `maximum_plastic_strain`,
            // whatever the creep rate alone would have taken this tick.
            T creep = material.plastic_creep;
            const T remaining = material.maximum_plastic_strain - element.accumulated_plastic_strain;
            if (strain_magnitude * creep > remaining)
                creep = remaining / strain_magnitude;
            if (!(creep > T(0)))
                return;

            element.plastic_inverse_column_0 = element.plastic_inverse_column_0 * (T(1) - creep) +
                                               current_shape_inverse.column0 * creep;
            element.plastic_inverse_column_1 = element.plastic_inverse_column_1 * (T(1) - creep) +
                                               current_shape_inverse.column1 * creep;
            element.plastic_inverse_column_2 = element.plastic_inverse_column_2 * (T(1) - creep) +
                                               current_shape_inverse.column2 * creep;

            element.accumulated_plastic_strain += strain_magnitude * creep;
            if (element.accumulated_plastic_strain > material.maximum_plastic_strain)
                element.accumulated_plastic_strain = material.maximum_plastic_strain;

            // The rest volume follows the rest shape: recomputed from the new
            // plastic rest matrix rather than left at the original, or a
            // permanently dented element would keep computing its hydrostatic
            // compliance against a volume it no longer actually rests at.
            FEMMatrix3<T> new_rest_shape;
            const FEMMatrix3<T> new_plastic_inverse{element.plastic_inverse_column_0,
                                                    element.plastic_inverse_column_1,
                                                    element.plastic_inverse_column_2};
            if (invert_fem_matrix3(new_plastic_inverse, new_rest_shape))
            {
                const T volume = determinant(new_rest_shape) / T(6);
                if (volume > T(0))
                    element.rest_volume = volume;
            }
        }
    } // namespace Physics
} // namespace SushiEngine
