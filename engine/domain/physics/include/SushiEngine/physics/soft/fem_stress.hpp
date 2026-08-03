/**************************************************************************/
/* fem_stress.hpp                                                        */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
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
 * @file fem_stress.hpp
 * @brief §9.3's readout: the Green strain, the Cauchy stress, and one von
 *        Mises scalar per element — all falling out of the deformation
 *        gradient `fem_projection.hpp` already computes.
 *
 * **Why St. Venant-Kirchhoff and not the exact stress conjugate to the two
 * XPBD constraints.** `fem_projection.hpp`'s two constraints are a positional
 * formulation — a constraint *function* and its gradient, not a strain-energy
 * density with a well-defined stress tensor of its own (recovering one would
 * mean inverting the XPBD compliance/substep relationship back into a stress
 * unit, which depends on the substep count and is not what §9.3 is asking
 * for). §9.3 wants a **material** reading — "is this beam strong enough" —
 * and the standard, textbook way to get one from a deformation gradient and a
 * pair of Lame parameters is the St. Venant-Kirchhoff relation: linear in the
 * Green strain, using the *same* `mu`/`lambda` the constraints already read.
 * It agrees closely with the exact neo-Hookean stress for the small-to-moderate
 * strains a stress *readout* is meant to characterize, and it is exact in the
 * well-known linear-elasticity limit this file's own tests check it against.
 *
 * **Why six scalars instead of a general 3x3 matrix type.** Strain and stress
 * are symmetric by construction (Green strain by definition; Cauchy stress
 * because `F S F^T` with symmetric `S` is symmetric), so a general 3x3 type
 * would carry three redundant, always-equal fields. `FemSymmetricMatrix3`
 * names the six independent components once, the way an engine that already
 * avoids a matrix library for `fem_projection.hpp`'s column arithmetic should.
 */

#include <cmath>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/soft/fem_projection.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief A symmetric 3x3 tensor: strain or stress, by its six independent components. */
        template <typename T>
        struct FemSymmetricMatrix3
        {
            T xx = 0, yy = 0, zz = 0;
            T xy = 0, yz = 0, zx = 0;
        };

        /** @brief One column of `FemMatrix3`, as a row — needed to form `F*S*F^T`. */
        template <typename T>
        inline Vector3T<T> fem_matrix_row(const FemMatrix3<T>& m, int row) noexcept
        {
            switch (row)
            {
                case 0:
                    return Vector3T<T>{m.column0.x, m.column1.x, m.column2.x};
                case 1:
                    return Vector3T<T>{m.column0.y, m.column1.y, m.column2.y};
                default:
                    return Vector3T<T>{m.column0.z, m.column1.z, m.column2.z};
            }
        }

        /**
         * @brief The Green-Lagrange strain `E = (F^T F - I) / 2`.
         *
         * Zero exactly at `F = I` (an undeformed element) and invariant under a
         * pure rotation of `F`, the same rotation-invariance property the
         * deviatoric constraint has and for the same reason: `F^T F` depends
         * only on `F`'s columns' lengths and mutual angles, not on how the
         * whole frame is oriented in world space.
         */
        template <typename T>
        inline FemSymmetricMatrix3<T> green_lagrange_strain(const FemMatrix3<T>& f) noexcept
        {
            FemSymmetricMatrix3<T> e;
            e.xx = (dot(f.column0, f.column0) - T(1)) * T(0.5);
            e.yy = (dot(f.column1, f.column1) - T(1)) * T(0.5);
            e.zz = (dot(f.column2, f.column2) - T(1)) * T(0.5);
            e.xy = dot(f.column0, f.column1) * T(0.5);
            e.yz = dot(f.column1, f.column2) * T(0.5);
            e.zx = dot(f.column2, f.column0) * T(0.5);
            return e;
        }

        /**
         * @brief The second Piola-Kirchhoff stress, St. Venant-Kirchhoff: `S = lambda*tr(E)*I + 2*mu*E`.
         *
         * @param strain The Green-Lagrange strain.
         * @param mu     The material's Lame `mu`.
         * @param lambda The material's Lame `lambda`.
         */
        template <typename T>
        inline FemSymmetricMatrix3<T> second_piola_kirchhoff_stress(
            const FemSymmetricMatrix3<T>& strain, T mu, T lambda) noexcept
        {
            const T trace = strain.xx + strain.yy + strain.zz;
            FemSymmetricMatrix3<T> s;
            s.xx = lambda * trace + T(2) * mu * strain.xx;
            s.yy = lambda * trace + T(2) * mu * strain.yy;
            s.zz = lambda * trace + T(2) * mu * strain.zz;
            s.xy = T(2) * mu * strain.xy;
            s.yz = T(2) * mu * strain.yz;
            s.zx = T(2) * mu * strain.zx;
            return s;
        }

        /**
         * @brief The Cauchy (true) stress, pulled forward from the second
         *        Piola-Kirchhoff stress: `sigma = (1/det(F)) * F * S * F^T`.
         *
         * Zero when `det(F)` is degenerate (a collapsed or inverted element)
         * rather than a division by zero or a not-a-number — a collapsed
         * element is a fracture-budget or cooking failure elsewhere in the
         * pipeline (§9.1's own guard makes the same call), and reporting no
         * stress there is safer than reporting an enormous, meaningless one
         * that would immediately trip the fracture threshold on a value that
         * was never a real material reading.
         */
        template <typename T>
        inline FemSymmetricMatrix3<T> cauchy_stress(const FemMatrix3<T>& f,
                                                     const FemSymmetricMatrix3<T>& second_pk) noexcept
        {
            const T jacobian = determinant(f);
            FemSymmetricMatrix3<T> sigma;
            if (!(jacobian > T(1e-9)))
                return sigma;

            // F * S, via S's own columns (S is symmetric, so its column j is
            // (S_0j, S_1j, S_2j), the same values whichever index is read first).
            const Vector3T<T> s_column0{second_pk.xx, second_pk.xy, second_pk.zx};
            const Vector3T<T> s_column1{second_pk.xy, second_pk.yy, second_pk.yz};
            const Vector3T<T> s_column2{second_pk.zx, second_pk.yz, second_pk.zz};
            FemMatrix3<T> fs;
            fs.column0 = f.column0 * s_column0.x + f.column1 * s_column0.y + f.column2 * s_column0.z;
            fs.column1 = f.column0 * s_column1.x + f.column1 * s_column1.y + f.column2 * s_column1.z;
            fs.column2 = f.column0 * s_column2.x + f.column1 * s_column2.y + f.column2 * s_column2.z;

            const T inverse_jacobian = T(1) / jacobian;
            const Vector3T<T> fs_row0 = fem_matrix_row(fs, 0);
            const Vector3T<T> fs_row1 = fem_matrix_row(fs, 1);
            const Vector3T<T> fs_row2 = fem_matrix_row(fs, 2);
            const Vector3T<T> f_row0 = fem_matrix_row(f, 0);
            const Vector3T<T> f_row1 = fem_matrix_row(f, 1);
            const Vector3T<T> f_row2 = fem_matrix_row(f, 2);

            sigma.xx = dot(fs_row0, f_row0) * inverse_jacobian;
            sigma.yy = dot(fs_row1, f_row1) * inverse_jacobian;
            sigma.zz = dot(fs_row2, f_row2) * inverse_jacobian;
            sigma.xy = dot(fs_row0, f_row1) * inverse_jacobian;
            sigma.yz = dot(fs_row1, f_row2) * inverse_jacobian;
            sigma.zx = dot(fs_row2, f_row0) * inverse_jacobian;
            return sigma;
        }

        /**
         * @brief The von Mises equivalent stress: one scalar summarizing how far
         *        past a uniaxial yield point this stress state sits.
         */
        template <typename T>
        inline T von_mises_stress(const FemSymmetricMatrix3<T>& sigma) noexcept
        {
            const T a = sigma.xx - sigma.yy;
            const T b = sigma.yy - sigma.zz;
            const T c = sigma.zz - sigma.xx;
            const T shear = sigma.xy * sigma.xy + sigma.yz * sigma.yz + sigma.zx * sigma.zx;
            const T sum_of_squares = T(0.5) * (a * a + b * b + c * c) + T(3) * shear;
            return sum_of_squares > T(0) ? std::sqrt(sum_of_squares) : T(0);
        }

        /**
         * @brief The von Mises stress of one tetrahedron, from its current pose.
         *
         * Computed once — not per constraint, not per sub-step — reusing
         * exactly the `F` the deviatoric and hydrostatic projections already
         * built, which is the "costs almost nothing" §9.3 promises.
         *
         * @param bodies   The owning solver's particle array.
         * @param element  The element to measure.
         * @param mu       The material's Lame `mu`.
         * @param lambda   The material's Lame `lambda`.
         */
        template <typename T>
        inline T tetrahedron_von_mises_stress(const RigidBodyT<T>* bodies,
                                              const FemTetrahedronT<T>& element, T mu,
                                              T lambda) noexcept
        {
            const Vector3T<T>& x0 = bodies[element.vertex[0]].position;
            const Vector3T<T> edge1 = bodies[element.vertex[1]].position - x0;
            const Vector3T<T> edge2 = bodies[element.vertex[2]].position - x0;
            const Vector3T<T> edge3 = bodies[element.vertex[3]].position - x0;
            const FemMatrix3<T> f = tetrahedron_deformation_gradient(
                edge1, edge2, edge3, element.plastic_inverse_column_0,
                element.plastic_inverse_column_1, element.plastic_inverse_column_2);
            const FemSymmetricMatrix3<T> strain = green_lagrange_strain(f);
            const FemSymmetricMatrix3<T> stress = second_piola_kirchhoff_stress(strain, mu, lambda);
            const FemSymmetricMatrix3<T> cauchy = cauchy_stress(f, stress);
            return von_mises_stress(cauchy);
        }
    } // namespace Physics
} // namespace SushiEngine
