/**************************************************************************/
/* test_mesh_mass_properties.cpp                                          */
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

// Polyhedral integration, checked against the closed forms that already exist for the
// primitives. That is the strongest oracle available: a box mesh and `box_mass_properties`
// must agree to floating point, and if they do not, exactly one of them is wrong and
// both are used by shipping code.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/mesh_utilities.hpp>
#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/geometry/mass_properties.hpp>
#include <SushiEngine/physics/geometry/mesh_mass_properties.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief An outward-wound box of the given half-extents, centred on @p offset. */
    Geometry::TriangleMesh box_mesh(float half_x, float half_y, float half_z,
                                    float offset_x = 0.0f, float offset_y = 0.0f,
                                    float offset_z = 0.0f)
    {
        Geometry::TriangleMesh mesh;
        const float corners[8][3] = {
            {-half_x, -half_y, -half_z}, {half_x, -half_y, -half_z},
            {half_x, half_y, -half_z},   {-half_x, half_y, -half_z},
            {-half_x, -half_y, half_z},  {half_x, -half_y, half_z},
            {half_x, half_y, half_z},    {-half_x, half_y, half_z}};
        for (const auto& corner : corners)
        {
            mesh.positions.push_back(corner[0] + offset_x);
            mesh.positions.push_back(corner[1] + offset_y);
            mesh.positions.push_back(corner[2] + offset_z);
        }
        const std::uint32_t faces[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                           {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                           {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};
        for (const auto& face : faces)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /**
     * @brief A fine outward-wound sphere, welded closed.
     *
     * The weld is not decoration. A latitude-longitude grid gives every slice its own copy
     * of each pole, so the caps are a ring of triangles that share no apex and the surface
     * has a hole at both ends — which is a perfectly ordinary dirty mesh and exactly what
     * `repair_mesh` is for. Integrating it unwelded would be asking for the volume of an
     * open shell, which is a question with a plausible wrong answer.
     */
    Geometry::TriangleMesh sphere_mesh(float radius, std::uint32_t stacks, std::uint32_t slices)
    {
        Geometry::TriangleMesh mesh;
        const float pi = 3.14159265358979323846f;
        for (std::uint32_t stack = 0; stack <= stacks; ++stack)
        {
            const float phi = pi * float(stack) / float(stacks);
            for (std::uint32_t slice = 0; slice < slices; ++slice)
            {
                const float theta = 2.0f * pi * float(slice) / float(slices);
                mesh.positions.push_back(radius * std::sin(phi) * std::cos(theta));
                mesh.positions.push_back(radius * std::cos(phi));
                mesh.positions.push_back(radius * std::sin(phi) * std::sin(theta));
            }
        }
        for (std::uint32_t stack = 0; stack < stacks; ++stack)
        {
            for (std::uint32_t slice = 0; slice < slices; ++slice)
            {
                const std::uint32_t next = (slice + 1) % slices;
                const std::uint32_t a = stack * slices + slice;
                const std::uint32_t b = stack * slices + next;
                const std::uint32_t c = (stack + 1) * slices + slice;
                const std::uint32_t d = (stack + 1) * slices + next;
                mesh.indices.push_back(a);
                mesh.indices.push_back(b);
                mesh.indices.push_back(c);
                mesh.indices.push_back(b);
                mesh.indices.push_back(d);
                mesh.indices.push_back(c);
            }
        }

        Geometry::TriangleMesh welded;
        Geometry::MeshRepairOptions options;
        options.weld_tolerance = radius * 1.0e-4f;
        Geometry::repair_mesh(mesh.view(), options, welded);
        return welded;
    }
} // namespace

TEST(Unit_MeshMassProperties,MatchesTheClosedFormForABox)
{
    const Scalar density = 800;
    const Geometry::TriangleMesh mesh = box_mesh(0.5f, 1.5f, 0.25f);
    const MeshMassProperties integrated = mesh_mass_properties(mesh.view(), density);
    ASSERT_TRUE(integrated.valid);

    const MassProperties<Scalar> closed_form =
        box_mass_properties(Vector3{Scalar(0.5), Scalar(1.5), Scalar(0.25)}, density);

    // Half-extents in, so the volume is the product of the full extents.
    EXPECT_NEAR(integrated.volume, 1.0 * 3.0 * 0.5, 1e-9);
    EXPECT_NEAR(integrated.properties.mass, closed_form.mass, 1e-6);
    EXPECT_NEAR(integrated.properties.inertia.x, closed_form.inertia.x, 1e-6);
    EXPECT_NEAR(integrated.properties.inertia.y, closed_form.inertia.y, 1e-6);
    EXPECT_NEAR(integrated.properties.inertia.z, closed_form.inertia.z, 1e-6);

    EXPECT_NEAR(integrated.properties.center_of_mass.x, 0.0, 1e-9);
    EXPECT_NEAR(integrated.properties.center_of_mass.y, 0.0, 1e-9);
    EXPECT_NEAR(integrated.properties.center_of_mass.z, 0.0, 1e-9);

    // A box modelled axis-aligned already diagonalizes its own tensor, so the principal
    // rotation must be the identity rather than an arbitrary permutation of the axes.
    // An editor drawing the principal frame is what makes this worth pinning down.
    EXPECT_NEAR(std::abs(integrated.principal_rotation.w), 1.0, 1e-6);
    EXPECT_NEAR(integrated.principal_rotation.x, 0.0, 1e-6);
    EXPECT_NEAR(integrated.principal_rotation.y, 0.0, 1e-6);
    EXPECT_NEAR(integrated.principal_rotation.z, 0.0, 1e-6);
}

TEST(Unit_MeshMassProperties,FindsTheCentreOfAnOffsetBox)
{
    // The whole reason the centre is integrated rather than assumed: a mesh authored
    // away from its origin has a centre of mass that is not the origin, and a body that
    // rotates about the wrong point tumbles plausibly and wrongly.
    const Geometry::TriangleMesh mesh = box_mesh(0.5f, 0.5f, 0.5f, 3.0f, -2.0f, 1.0f);
    const MeshMassProperties integrated = mesh_mass_properties(mesh.view(), Scalar(1000));
    ASSERT_TRUE(integrated.valid);

    EXPECT_NEAR(integrated.properties.center_of_mass.x, 3.0, 1e-6);
    EXPECT_NEAR(integrated.properties.center_of_mass.y, -2.0, 1e-6);
    EXPECT_NEAR(integrated.properties.center_of_mass.z, 1.0, 1e-6);

    // And the inertia is about that centre, so it matches the unit cube's despite the
    // offset. Getting this wrong is the classic parallel-axis mistake, and its symptom
    // is a distant object that is inexplicably hard to spin.
    const MassProperties<Scalar> closed_form =
        box_mass_properties(Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)}, Scalar(1000));
    EXPECT_NEAR(integrated.properties.inertia.x, closed_form.inertia.x, 1e-6);
    EXPECT_NEAR(integrated.properties.inertia.y, closed_form.inertia.y, 1e-6);
    EXPECT_NEAR(integrated.properties.inertia.z, closed_form.inertia.z, 1e-6);
}

TEST(Unit_MeshMassProperties,MatchesTheClosedFormForASphere)
{
    const Scalar density = 1200;
    const float radius = 0.75f;
    const Geometry::TriangleMesh mesh = sphere_mesh(radius, 48, 96);
    const MeshMassProperties integrated = mesh_mass_properties(mesh.view(), density);
    ASSERT_TRUE(integrated.valid);

    const MassProperties<Scalar> closed_form = sphere_mass_properties(Scalar(radius), density);

    // A tessellated sphere is inscribed in the true one, so it is slightly lighter: the
    // tolerance is that chord deficit and not an admission of imprecision.
    EXPECT_NEAR(integrated.properties.mass, closed_form.mass, closed_form.mass * 0.002);
    EXPECT_NEAR(integrated.properties.inertia.x, closed_form.inertia.x,
                closed_form.inertia.x * 0.004);
    EXPECT_NEAR(integrated.properties.inertia.y, closed_form.inertia.y,
                closed_form.inertia.y * 0.004);

    // A sphere's three moments are equal, which is the degenerate case a closed-form
    // cubic eigensolver loses its digits on and Jacobi does not.
    EXPECT_NEAR(integrated.properties.inertia.x, integrated.properties.inertia.y,
                integrated.properties.inertia.x * 0.002);
    EXPECT_NEAR(integrated.properties.inertia.y, integrated.properties.inertia.z,
                integrated.properties.inertia.y * 0.002);
}

TEST(Unit_MeshMassProperties,DiagonalizesARotatedBox)
{
    // The reason the eigendecomposition is in the cooker at all. A box turned about Z
    // has a tensor with real products of inertia; keeping only its diagonal would
    // silently discard them.
    //
    // Thirty degrees and not forty-five, deliberately. At forty-five each eigenvector is
    // equally aligned with two coordinate axes, so which axis it is assigned to is a tie
    // — a real ambiguity in the shape rather than a defect, and asserting one arm of a
    // coin flip would make this test lie about what the code guarantees.
    const float angle = 3.14159265358979323846f / 6.0f;
    const float cosine = std::cos(angle);
    const float sine = std::sin(angle);
    Geometry::TriangleMesh rotated = box_mesh(0.5f, 1.5f, 0.25f);
    for (std::size_t i = 0; i < rotated.vertex_count(); ++i)
    {
        const float x = rotated.positions[i * 3 + 0];
        const float y = rotated.positions[i * 3 + 1];
        rotated.positions[i * 3 + 0] = x * cosine - y * sine;
        rotated.positions[i * 3 + 1] = x * sine + y * cosine;
    }

    const Scalar density = 800;
    const MeshMassProperties integrated = mesh_mass_properties(rotated.view(), density);
    ASSERT_TRUE(integrated.valid);

    // The principal moments are the *unrotated* box's: rotating a body does not change
    // what it weighs or how hard it is to spin about its own axes.
    const MassProperties<Scalar> closed_form =
        box_mass_properties(Vector3{Scalar(0.5), Scalar(1.5), Scalar(0.25)}, density);
    //
    // The tolerances are *relative*, and which side of the seam the error is on was
    // measured rather than assumed: integrating these same float32 corners in long double
    // with an independent tetra-fan formula reproduces the cooker's answer to its last two
    // digits (1199.9999839774744 against 1199.9999839774746). So the shortfall is in the
    // stored vertices — a rotated corner is not representable in float32 where an
    // axis-aligned one is, which is exactly why the unrotated box above holds absolute
    // bounds and this one cannot. The bound is ~75x the observed residual and still orders
    // of magnitude tighter than any algorithmic error could hide in.
    constexpr Scalar FLOAT_INPUT_TOLERANCE = Scalar(1e-6);
    EXPECT_NEAR(integrated.properties.mass, closed_form.mass,
                closed_form.mass * FLOAT_INPUT_TOLERANCE);
    EXPECT_NEAR(integrated.properties.inertia.x, closed_form.inertia.x,
                closed_form.inertia.x * FLOAT_INPUT_TOLERANCE);
    EXPECT_NEAR(integrated.properties.inertia.y, closed_form.inertia.y,
                closed_form.inertia.y * FLOAT_INPUT_TOLERANCE);
    EXPECT_NEAR(integrated.properties.inertia.z, closed_form.inertia.z,
                closed_form.inertia.z * FLOAT_INPUT_TOLERANCE);

    // And the rotation is the one that gets you back, which is the half that would be
    // useless if it were not reported: rotating the principal x axis into the mesh frame
    // must land on the box's own long-since-rotated x axis.
    const Vector3 principal_x =
        rotate(integrated.principal_rotation, Vector3{Scalar(1), Scalar(0), Scalar(0)});
    EXPECT_NEAR(principal_x.x, Scalar(cosine), 1e-4);
    EXPECT_NEAR(principal_x.y, Scalar(sine), 1e-4);
    EXPECT_NEAR(principal_x.z, 0.0, 1e-4);
}

TEST(Unit_MeshMassProperties,RefusesWhatItCannotIntegrate)
{
    // Each of these must report invalid rather than return zeroes that look like an
    // answer: a body of zero mass and zero inertia is one the solver treats as
    // infinitely heavy and unable to rotate, which is the least debuggable outcome
    // available.
    Geometry::TriangleMesh empty;
    EXPECT_FALSE(mesh_mass_properties(empty.view(), Scalar(1000)).valid);

    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    EXPECT_FALSE(mesh_mass_properties(box.view(), Scalar(0)).valid);
    EXPECT_FALSE(mesh_mass_properties(box.view(), Scalar(-1)).valid);

    // A flat sheet encloses nothing.
    Geometry::TriangleMesh sheet;
    sheet.positions = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f};
    sheet.indices = {0, 1, 2, 0, 2, 3};
    EXPECT_FALSE(mesh_mass_properties(sheet.view(), Scalar(1000)).valid);

    // An open shell is the case worth pinning down, because it is the one that returns a
    // *plausible* wrong answer rather than an obviously broken one: the divergence theorem
    // happily reports the cone fan from the origin, and a unit box missing one face comes
    // out at five sixths of its true volume. Refusing is the only honest response.
    Geometry::TriangleMesh open = box;
    open.indices.resize(open.indices.size() - 6);
    const MeshMassProperties from_open = mesh_mass_properties(open.view(), Scalar(1000));
    EXPECT_FALSE(from_open.valid);
    EXPECT_EQ(from_open.properties.mass, 0.0);

    // Non-manifold for the same reason: a doubled triangle makes the enclosed volume a
    // question about which copy counts.
    Geometry::TriangleMesh doubled = box;
    for (int i = 0; i < 3; ++i)
        doubled.indices.push_back(doubled.indices[i]);
    EXPECT_FALSE(mesh_mass_properties(doubled.view(), Scalar(1000)).valid);

    // A mesh wound inside out integrates to a negative volume, which is reported rather
    // than absolute-valued: the caller has a winding bug and needs to know, instead of
    // receiving plausible numbers.
    Geometry::TriangleMesh inverted = box;
    for (std::size_t face = 0; face < inverted.triangle_count(); ++face)
        std::swap(inverted.indices[face * 3 + 1], inverted.indices[face * 3 + 2]);
    const MeshMassProperties result = mesh_mass_properties(inverted.view(), Scalar(1000));
    EXPECT_FALSE(result.valid);
    EXPECT_LT(result.volume, 0.0);
}

TEST(Unit_MeshMassProperties,IsIndifferentToWhereTheOriginSits)
{
    // The divergence theorem's payoff, and the property that makes an interior sampling
    // pass unnecessary: whatever lies outside the surface is spanned twice with opposite
    // orientation and cancels, so a mesh far from its origin integrates as exactly as
    // one around it.
    const Geometry::TriangleMesh near_origin = box_mesh(0.5f, 1.5f, 0.25f);
    const Geometry::TriangleMesh far_away = box_mesh(0.5f, 1.5f, 0.25f, 500.0f, 0.0f, 0.0f);

    const MeshMassProperties here = mesh_mass_properties(near_origin.view(), Scalar(800));
    const MeshMassProperties there = mesh_mass_properties(far_away.view(), Scalar(800));
    ASSERT_TRUE(here.valid);
    ASSERT_TRUE(there.valid);

    EXPECT_NEAR(there.properties.mass, here.properties.mass, here.properties.mass * 1e-6);
    EXPECT_NEAR(there.properties.center_of_mass.x, 500.0, 1e-3);
    EXPECT_NEAR(there.properties.inertia.x, here.properties.inertia.x,
                here.properties.inertia.x * 1e-3);
    EXPECT_NEAR(there.properties.inertia.y, here.properties.inertia.y,
                here.properties.inertia.y * 1e-3);
}
