/**************************************************************************/
/* fem_element.hpp                                                       */
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
 * @file fem_element.hpp
 * @brief One tetrahedron's rest state and per-tick solver bookkeeping.
 *
 * The FEM analogue of `XpbdDistanceConstraintT` (`physics/constraints/xpbd_constraint.hpp`):
 * a trivially-copyable descriptor holding exactly the per-element constants the
 * projection needs. Where a distance constraint names two bodies, an element
 * names four — the tetrahedron's own vertices — which is the one respect in
 * which this cannot yet be registered as a constraint *kind* in the shared
 * solver (`physics/solver/`): `ConstraintStore`, `IncrementalColoring`, and
 * `color_constraints` are written for exactly two body indices per constraint
 * end to end, and generalizing that to N bodies is real, separate work this
 * element deliberately does not take on. §16's P6-A entry runs this element
 * through a small host-only reference solver instead
 * (`finite_element_model.hpp`) — the same relationship `HostXpbdSolver` has to
 * `RuntimeGraphBuilder`, but for now the only implementation, not a mirror of
 * one.
 */

#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief One tetrahedron: its four particles, its rest state, and its
         *        two XPBD constraints' accumulated Lagrange multipliers.
         *
         * @ref rest_inverse_column_0/1/2 are the three columns of `Dm^-1` — the
         * inverse rest-shape matrix `physics/cooking/tetrahedral_mesh.hpp` already
         * cooks, in exactly the layout `TetrahedralMesh::rest_inverse` and
         * `Cooking::SoftBodyAssetView::rest_inverse` store it in (three `Vector3`s
         * per element). Carrying it as three named columns rather than as an
         * opaque array is what lets `fem_projection.hpp`'s formulas be written
         * against `.x`/`.y`/`.z` the way every other shape and constraint in this
         * codebase is, instead of introducing a general matrix type for the one
         * place that would use it.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct FemTetrahedronT
        {
            using Real = T;

            /** @brief The tetrahedron's four particle indices, into the owning solver's array. */
            std::uint32_t vertex[4] = {0, 0, 0, 0};

            /** @brief Column 0 of `Dm^-1`. */
            Vector3T<T> rest_inverse_column_0{Vector3T<T>{T(0), T(0), T(0)}};
            /** @brief Column 1 of `Dm^-1`. */
            Vector3T<T> rest_inverse_column_1{Vector3T<T>{T(0), T(0), T(0)}};
            /** @brief Column 2 of `Dm^-1`. */
            Vector3T<T> rest_inverse_column_2{Vector3T<T>{T(0), T(0), T(0)}};

            /** @brief The tetrahedron's rest volume; must be positive (§9.1's compliance divides by it). */
            T rest_volume = 0;

            /**
             * @brief The deviatoric constraint's accumulated Lagrange multiplier.
             *
             * Reset to zero at the first substep of every tick, exactly like a
             * contact's or a joint's — an element's internal stress is a
             * per-tick quantity, not one that should compound tick over tick.
             */
            T deviatoric_lambda = 0;

            /** @brief The hydrostatic constraint's accumulated Lagrange multiplier. */
            T hydrostatic_lambda = 0;

            /**
             * @brief The element's plastic part of `Dm^-1`, read instead of the rest
             *        state once §9.4 (P6-C) begins writing it.
             *
             * Kept at the identity-equivalent (the same as @ref rest_inverse_column_0/1/2)
             * until plasticity is implemented; separated from the elastic rest state
             * now rather than folded in later, because a plastic update overwrites
             * *this*, never the original rest matrix a fracture or an LOD remap
             * still needs.
             */
            Vector3T<T> plastic_inverse_column_0{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> plastic_inverse_column_1{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> plastic_inverse_column_2{Vector3T<T>{T(0), T(0), T(0)}};

            /** @brief The von Mises equivalent stress from this element's last projection (§9.3). */
            T von_mises_stress = 0;

            /**
             * @brief How much permanent (plastic) strain this element has accumulated (§9.4).
             *
             * A dimensionless magnitude — `||F_elastic - I||`, folded in a little at a
             * time as the element yields — not a stress. Clamped to
             * `SoftBodyMaterialT::maximum_plastic_strain`; once it reaches that ceiling
             * the element has hardened and stops accumulating further, though it can
             * still deform *elastically* around whatever permanent shape it is left in.
             */
            T accumulated_plastic_strain = 0;
        };

        /** @brief The boundary FEM element: `FemTetrahedronT` fixed to `Scalar`. */
        using FemTetrahedron = FemTetrahedronT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
