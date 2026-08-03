/**************************************************************************/
/* mesh_mass_properties.hpp                                               */
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
 * @file mesh_mass_properties.hpp
 * @brief Mass, centre and inertia integrated over an arbitrary closed mesh.
 *
 * `mass_properties.hpp` has the closed forms for the primitives and says explicitly
 * that a cooked hull has none: its mass properties are integrated over its faces, and
 * the cooker is what produces faces. This is that integration, and it completes the
 * §1.2 item 6 story — an author no longer types a diagonal inverse inertia tensor by
 * hand for *any* shape, primitive or cooked.
 *
 * **The mechanism.** A closed surface's volume integrals reduce to a signed sum over
 * the tetrahedra spanned from the origin to each triangle, which is the divergence
 * theorem doing the work: no interior sampling, no voxel grid, exact to floating point
 * for any closed polyhedron regardless of whether the origin is inside it, because the
 * pieces outside cancel. The second-moment integral over one such tetrahedron has a
 * closed form (`det/120 * (a a^T + b b^T + c c^T + s s^T)`, with `s = a + b + c`),
 * obtained by mapping the reference simplex through the tetrahedron's edge matrix.
 *
 * **Why the eigendecomposition is here and not later.** `RigidBodyT::inv_inertia` is a
 * *diagonal* in the body-local frame, which is only correct in the frame where the
 * tensor is diagonal. A cooked hull's tensor is not diagonal in the frame the artist
 * modelled it in, so keeping only its diagonal would silently discard the products of
 * inertia and produce a body that tumbles plausibly and wrongly. So the cooker rotates
 * into the principal axes and reports the rotation, which §5.1 says bodies store.
 *
 * Not templated, unlike its neighbours in this directory: this runs once per asset, on
 * the host, at cook time, and reusing @ref quaternion_from_matrix rather than
 * duplicating Shepperd's method in a template is worth more than a precision the
 * caller has no way to want.
 */

#include <cmath>
#include <cstddef>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/geometry/mesh_utilities.hpp>
#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/geometry/mass_properties.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief What integrating a closed mesh produced, in its principal frame.
         *
         * @ref properties holds the mass, the centre of mass in the *mesh's* frame, and
         * the inertia as a diagonal in the **principal** frame. The two frames are
         * related by @ref principal_rotation, and using one without the other is the
         * mistake this type exists to make visible.
         */
        struct MeshMassProperties
        {
            /** @brief Mass, centre of mass, and the principal moments as a diagonal. */
            MassProperties<Scalar> properties;

            /**
             * @brief Rotation taking a vector from the principal frame to the mesh frame.
             *
             * Identity whenever the mesh's own axes already diagonalize the tensor,
             * which is deliberate: the axis assignment picks, for each coordinate axis,
             * the eigenvector most aligned with it. A box modelled axis-aligned therefore
             * reports no rotation instead of an arbitrary permutation of its own axes,
             * and an editor showing the principal frame shows something recognisable.
             */
            Quaternion principal_rotation{Quaternion{0, 0, 0, 1}};

            /** @brief The enclosed volume, in cubic local units; negative for an inverted mesh. */
            Scalar volume = 0;

            /**
             * @brief False when the mesh was not something these integrals apply to.
             *
             * An open shell, a non-manifold surface, a zero-volume sheet, a mesh wound
             * inside out, or an empty one. The caller must not fall back to the values in
             * @ref properties, which are zero — a body of zero mass and zero inertia is a
             * body the solver treats as infinitely heavy and unable to rotate, which is the
             * least debuggable possible outcome. Zero mass does, however, already mean
             * "keep the authored value" at the extract, which is the right fallback for a
             * collider that genuinely is a single-sided sheet.
             */
            bool valid = false;
        };

        namespace detail
        {
            /**
             * @brief Diagonalizes a symmetric 3x3 by cyclic Jacobi rotations.
             *
             * Jacobi rather than a closed-form cubic root: three-by-three eigenvalues do
             * have an analytic form, and it loses most of its digits on the nearly-degenerate
             * tensors that ordinary content produces constantly — a symmetric box, a
             * cylinder, anything with two equal moments. Jacobi converges quadratically
             * and is exact on a matrix that is already diagonal, which is the common case.
             *
             * @param tensor      The symmetric matrix, row-major; overwritten.
             * @param eigenvectors Receives the eigenvectors as *columns*, row-major.
             */
            inline void jacobi_symmetric_3x3(Scalar tensor[3][3], Scalar eigenvectors[3][3]) noexcept
            {
                for (int row = 0; row < 3; ++row)
                    for (int column = 0; column < 3; ++column)
                        eigenvectors[row][column] = row == column ? Scalar(1) : Scalar(0);

                for (int sweep = 0; sweep < 32; ++sweep)
                {
                    const Scalar off_diagonal = std::abs(tensor[0][1]) + std::abs(tensor[0][2]) +
                                                std::abs(tensor[1][2]);
                    if (off_diagonal <= Scalar(1e-18))
                        return;

                    for (int p = 0; p < 2; ++p)
                    {
                        for (int q = p + 1; q < 3; ++q)
                        {
                            if (std::abs(tensor[p][q]) <= Scalar(1e-20))
                                continue;

                            const Scalar theta = (tensor[q][q] - tensor[p][p]) /
                                                 (Scalar(2) * tensor[p][q]);
                            const Scalar sign = theta >= Scalar(0) ? Scalar(1) : Scalar(-1);
                            const Scalar tangent =
                                sign / (std::abs(theta) + std::sqrt(theta * theta + Scalar(1)));
                            const Scalar cosine = Scalar(1) / std::sqrt(tangent * tangent + Scalar(1));
                            const Scalar sine = tangent * cosine;
                            const Scalar tau = sine / (Scalar(1) + cosine);

                            const Scalar pivot = tensor[p][q];
                            tensor[p][p] -= tangent * pivot;
                            tensor[q][q] += tangent * pivot;
                            tensor[p][q] = 0;
                            tensor[q][p] = 0;

                            // The third row and column mix, and both updates read the
                            // pre-rotation values, so they are captured before either is
                            // written. Getting this wrong is the classic Jacobi bug: it
                            // still converges, to the wrong basis.
                            for (int r = 0; r < 3; ++r)
                            {
                                if (r == p || r == q)
                                    continue;
                                const Scalar rp = tensor[r][p];
                                const Scalar rq = tensor[r][q];
                                tensor[r][p] = rp - sine * (rq + tau * rp);
                                tensor[p][r] = tensor[r][p];
                                tensor[r][q] = rq + sine * (rp - tau * rq);
                                tensor[q][r] = tensor[r][q];
                            }
                            for (int r = 0; r < 3; ++r)
                            {
                                const Scalar rp = eigenvectors[r][p];
                                const Scalar rq = eigenvectors[r][q];
                                eigenvectors[r][p] = rp - sine * (rq + tau * rp);
                                eigenvectors[r][q] = rq + sine * (rp - tau * rq);
                            }
                        }
                    }
                }
            }
        } // namespace detail

        /**
         * @brief Mass, centre of mass and principal inertia of a closed triangle mesh.
         *
         * **Closure is checked, not assumed**, and that check is the difference between a
         * refusal and a wrong answer. The divergence theorem needs a closed surface; given
         * an open one it still returns a number, and the number is the volume of the cone
         * fan from the origin to whatever surface it was handed. A box missing one face
         * integrates to five sixths of its volume — entirely plausible, entirely wrong, and
         * impossible to trace back later. So a surface with a boundary or a non-manifold
         * edge is reported invalid instead.
         *
         * An inverted mesh integrates to a negative volume, which is likewise reported
         * rather than silently absolute-valued, because the caller that gets it has a
         * winding bug and needs to know. `Geometry::repair_mesh` is what makes an arbitrary
         * import both closed and outward-wound.
         *
         * A **self-intersecting** mesh is closed and does integrate, and its overlapping
         * region is counted twice. That is inherent to the input rather than to the method,
         * and the honest place to catch it is the repair report's component count.
         *
         * @param mesh    The closed surface to integrate; only positions and indices read.
         * @param density Mass per unit volume; zero or less yields an invalid result.
         * @return The integrated properties, with @c valid false when the surface was not
         *         closed, was inverted, or enclosed no volume.
         */
        inline MeshMassProperties mesh_mass_properties(const Geometry::TriangleMeshView& mesh,
                                                       Scalar density)
        {
            MeshMassProperties result;
            if (!mesh.has_triangles() || !(density > Scalar(0)))
                return result;

            const Geometry::MeshTopologyReport topology = Geometry::analyze_mesh_topology(mesh);
            if (!topology.watertight())
                return result;

            Scalar volume_times_six = 0;
            Vector3 first_moment{Vector3{0, 0, 0}};
            Scalar covariance[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

            const std::size_t triangle_count = mesh.triangle_count();
            for (std::size_t t = 0; t < triangle_count; ++t)
            {
                const std::uint32_t i0 = mesh.indices[t * 3 + 0];
                const std::uint32_t i1 = mesh.indices[t * 3 + 1];
                const std::uint32_t i2 = mesh.indices[t * 3 + 2];
                if (i0 >= mesh.vertex_count || i1 >= mesh.vertex_count ||
                    i2 >= mesh.vertex_count)
                    continue;

                float raw[3][3];
                mesh.read_position(i0, raw[0]);
                mesh.read_position(i1, raw[1]);
                mesh.read_position(i2, raw[2]);
                const Vector3 a{Scalar(raw[0][0]), Scalar(raw[0][1]), Scalar(raw[0][2])};
                const Vector3 b{Scalar(raw[1][0]), Scalar(raw[1][1]), Scalar(raw[1][2])};
                const Vector3 c{Scalar(raw[2][0]), Scalar(raw[2][1]), Scalar(raw[2][2])};

                // Six times the signed volume of the tetrahedron from the origin. The
                // signs are what make the origin's position irrelevant: whatever lies
                // outside the surface is spanned twice with opposite orientation.
                const Scalar determinant = dot(a, cross(b, c));
                volume_times_six += determinant;

                const Vector3 sum = a + b + c;
                first_moment = first_moment + sum * (determinant / Scalar(24));

                const Scalar scale = determinant / Scalar(120);
                for (int row = 0; row < 3; ++row)
                {
                    const Scalar ar = row == 0 ? a.x : (row == 1 ? a.y : a.z);
                    const Scalar br = row == 0 ? b.x : (row == 1 ? b.y : b.z);
                    const Scalar cr = row == 0 ? c.x : (row == 1 ? c.y : c.z);
                    const Scalar sr = row == 0 ? sum.x : (row == 1 ? sum.y : sum.z);
                    for (int column = 0; column < 3; ++column)
                    {
                        const Scalar ac = column == 0 ? a.x : (column == 1 ? a.y : a.z);
                        const Scalar bc = column == 0 ? b.x : (column == 1 ? b.y : b.z);
                        const Scalar cc = column == 0 ? c.x : (column == 1 ? c.y : c.z);
                        const Scalar sc = column == 0 ? sum.x : (column == 1 ? sum.y : sum.z);
                        covariance[row][column] +=
                            scale * (ar * ac + br * bc + cr * cc + sr * sc);
                    }
                }
            }

            result.volume = volume_times_six / Scalar(6);
            if (!(result.volume > Scalar(0)))
                return result;   // open, degenerate, or wound inside out

            const Scalar mass = result.volume * density;
            const Vector3 center = first_moment * (Scalar(1) / result.volume);

            // The covariance is a volume integral; density turns it into a mass one.
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    covariance[row][column] *= density;

            // Shift the second moment to the centre of mass before forming the tensor,
            // rather than forming the tensor and shifting it: the two are equivalent and
            // this way the parallel-axis term is one outer product instead of three
            // special cases.
            const Scalar center_component[3] = {center.x, center.y, center.z};
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    covariance[row][column] -=
                        mass * center_component[row] * center_component[column];

            // I = trace(C) * Identity - C, the standard relation between the second
            // moment of mass and the inertia tensor.
            const Scalar trace = covariance[0][0] + covariance[1][1] + covariance[2][2];
            Scalar tensor[3][3];
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    tensor[row][column] = (row == column ? trace : Scalar(0)) -
                                          covariance[row][column];

            Scalar eigenvectors[3][3];
            detail::jacobi_symmetric_3x3(tensor, eigenvectors);
            const Scalar moment[3] = {tensor[0][0], tensor[1][1], tensor[2][2]};

            // Assign each coordinate axis the eigenvector most aligned with it, greedily
            // and largest alignment first, so an axis-aligned shape keeps its own axes.
            int assignment[3] = {-1, -1, -1};
            bool taken[3] = {false, false, false};
            for (int pass = 0; pass < 3; ++pass)
            {
                int best_axis = -1;
                int best_column = -1;
                Scalar best_alignment = Scalar(-1);
                for (int axis = 0; axis < 3; ++axis)
                {
                    if (assignment[axis] >= 0)
                        continue;
                    for (int column = 0; column < 3; ++column)
                    {
                        if (taken[column])
                            continue;
                        const Scalar alignment = std::abs(eigenvectors[axis][column]);
                        if (alignment > best_alignment)
                        {
                            best_alignment = alignment;
                            best_axis = axis;
                            best_column = column;
                        }
                    }
                }
                if (best_axis < 0 || best_column < 0)
                    break;
                assignment[best_axis] = best_column;
                taken[best_column] = true;
            }

            Scalar basis[3][3];
            Vector3 principal{Vector3{0, 0, 0}};
            for (int axis = 0; axis < 3; ++axis)
            {
                const int column = assignment[axis] >= 0 ? assignment[axis] : axis;
                // Point the axis the same way as the coordinate axis it stands in for, so
                // the rotation is the small one rather than its 180-degree twin.
                const Scalar sign = eigenvectors[axis][column] < Scalar(0) ? Scalar(-1) : Scalar(1);
                for (int row = 0; row < 3; ++row)
                    basis[row][axis] = eigenvectors[row][column] * sign;
                const Scalar value = moment[column];
                if (axis == 0)
                    principal.x = value;
                else if (axis == 1)
                    principal.y = value;
                else
                    principal.z = value;
            }

            // A reflection is not a rotation: flipping one axis costs nothing physically,
            // because an inertia axis has no preferred direction, and leaves a quaternion
            // that means what it says.
            const Scalar determinant =
                basis[0][0] * (basis[1][1] * basis[2][2] - basis[1][2] * basis[2][1]) -
                basis[0][1] * (basis[1][0] * basis[2][2] - basis[1][2] * basis[2][0]) +
                basis[0][2] * (basis[1][0] * basis[2][1] - basis[1][1] * basis[2][0]);
            if (determinant < Scalar(0))
            {
                for (int row = 0; row < 3; ++row)
                    basis[row][2] = -basis[row][2];
            }

            Matrix4 rotation{};
            for (int i = 0; i < 16; ++i)
                rotation.m[i] = Scalar(0);
            for (int column = 0; column < 3; ++column)
                for (int row = 0; row < 3; ++row)
                    rotation.m[std::size_t(column * 4 + row)] = basis[row][column];
            rotation.m[15] = Scalar(1);

            result.properties.mass = mass;
            result.properties.center_of_mass = center;
            result.properties.inertia = principal;
            result.principal_rotation = normalize(quaternion_from_matrix(rotation));
            result.valid = true;
            return result;
        }
    } // namespace Physics
} // namespace SushiEngine
