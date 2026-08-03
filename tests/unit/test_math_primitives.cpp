/**************************************************************************/
/* test_math_primitives.cpp                                             */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_MathPrimitives: the vector/quaternion/matrix seam every camera and transform is
// built on (core/types.hpp -> blas_placeholder). Only the floating-origin split was
// covered before; this pins the rest: quaternion rotation matches its matrix form and is
// an isometry, the reverse-Z infinite-far projection maps the near plane to depth 1 and
// distance to 0, and look_at/compose place the eye and the model origin where they must.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/core/types.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    // Column-major Matrix4 * (v, w): element [row, col] lives at m[col * 4 + row].
    struct Vector4
    {
        Scalar x, y, z, w;
    };

    Vector4 transform(const Matrix4& m, const Vector4& v)
    {
        Vector4 r;
        r.x = m.m[0] * v.x + m.m[4] * v.y + m.m[8] * v.z + m.m[12] * v.w;
        r.y = m.m[1] * v.x + m.m[5] * v.y + m.m[9] * v.z + m.m[13] * v.w;
        r.z = m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z + m.m[14] * v.w;
        r.w = m.m[3] * v.x + m.m[7] * v.y + m.m[11] * v.z + m.m[15] * v.w;
        return r;
    }
}

TEST(Unit_MathPrimitives, VectorAlgebraBasics)
{
    const Vector3 x{1.0, 0.0, 0.0};
    const Vector3 y{0.0, 1.0, 0.0};
    EXPECT_NEAR(dot(x, y), 0.0, 1e-15);
    EXPECT_NEAR(dot(x, x), 1.0, 1e-15);
    // Right-handed: x cross y = z.
    EXPECT_TRUE(Harness::approx_equal(cross(x, y), Vector3{0.0, 0.0, 1.0}, 1e-15));
    EXPECT_NEAR(length(Vector3{3.0, 4.0, 0.0}), 5.0, 1e-12);
    EXPECT_TRUE(Harness::is_unit(normalize(Vector3{0.0, 2.0, -2.0}), 1e-12));
    // Degenerate normalize returns the zero vector unchanged (no divide by zero).
    EXPECT_TRUE(Harness::approx_equal(normalize(Vector3{0.0, 0.0, 0.0}), Vector3{0.0, 0.0, 0.0},
                                      1e-15));
}

TEST(Unit_MathPrimitives, QuaternionRotatesAndIsAnIsometry)
{
    const Quaternion qz = quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, 1.5707963267948966);
    // 90 deg about +Z sends +X to +Y.
    EXPECT_TRUE(Harness::approx_equal(rotate(qz, Vector3{1.0, 0.0, 0.0}), Vector3{0.0, 1.0, 0.0},
                                      1e-12));
    // Rotation preserves length.
    const Vector3 v{0.3, -0.7, 0.5};
    EXPECT_NEAR(length(rotate(qz, v)), length(v), 1e-12);
}

TEST(Unit_MathPrimitives, QuaternionTimesConjugateIsIdentity)
{
    const Quaternion q =
        normalize(quaternion_axis_angle(normalize(Vector3{1.0, 2.0, 3.0}), 1.1));
    const Quaternion identity = mul(q, conjugate(q));
    const Vector3 v{0.4, 0.5, -0.6};
    EXPECT_TRUE(Harness::approx_equal(rotate(identity, v), v, 1e-12));
}

TEST(Unit_MathPrimitives, QuaternionMatrixMatchesDirectRotation)
{
    const Quaternion q =
        normalize(quaternion_axis_angle(normalize(Vector3{0.2, -1.0, 0.5}), 2.3));
    const Matrix4 m = matrix4_from_quaternion(q);
    const Vector3 v{0.7, 0.1, -0.4};
    const Vector4 by_matrix = transform(m, Vector4{v.x, v.y, v.z, 0.0});
    const Vector3 by_rotate = rotate(q, v);
    EXPECT_TRUE(Harness::approx_equal(Vector3{by_matrix.x, by_matrix.y, by_matrix.z}, by_rotate,
                                      1e-12));
}

TEST(Unit_MathPrimitives, ReverseZProjectionMapsNearToOneAndInfinityToZero)
{
    const Matrix4 p = perspective(1.5707963267948966, 1.0, 0.1, 1000.0);

    // Structural entries of the reverse-Z, infinite-far, Y-flipped projection.
    EXPECT_NEAR(p.m[5], -1.0, 1e-9);  // Y flip (f/aspect with f=1)
    EXPECT_NEAR(p.m[10], 0.0, 1e-12); // no linear depth term
    EXPECT_NEAR(p.m[11], -1.0, 1e-12);
    EXPECT_NEAR(p.m[14], 0.1, 1e-12); // near plane

    // A point on the near plane (view z = -near) lands at NDC depth 1.
    const Vector4 near_clip = transform(p, Vector4{0.0, 0.0, -0.1, 1.0});
    EXPECT_NEAR(near_clip.z / near_clip.w, 1.0, 1e-9);

    // A very distant point approaches NDC depth 0.
    const Vector4 far_clip = transform(p, Vector4{0.0, 0.0, -1.0e6, 1.0});
    EXPECT_NEAR(far_clip.z / far_clip.w, 0.0, 1e-6);
}

TEST(Unit_MathPrimitives, LookAtPlacesEyeAtOriginAndTargetDownNegativeZ)
{
    const Matrix4 view = look_at(Vector3{0.0, 0.0, 5.0}, Vector3{0.0, 0.0, 0.0},
                                 Vector3{0.0, 1.0, 0.0});
    // The eye maps to the view-space origin.
    const Vector4 eye = transform(view, Vector4{0.0, 0.0, 5.0, 1.0});
    EXPECT_NEAR(eye.x, 0.0, 1e-12);
    EXPECT_NEAR(eye.y, 0.0, 1e-12);
    EXPECT_NEAR(eye.z, 0.0, 1e-12);
    // The look-at target is in front of the camera: negative view z.
    const Vector4 target = transform(view, Vector4{0.0, 0.0, 0.0, 1.0});
    EXPECT_NEAR(target.z, -5.0, 1e-12);
}

TEST(Unit_MathPrimitives, ComposeTransformPlacesTheModelOrigin)
{
    const Vector3 position{10.0, -4.0, 2.0};
    const Matrix4 model = compose_transform(position, Quaternion{}, Vector3{1.0, 1.0, 1.0});
    // With identity rotation and unit scale, the model origin lands at the position.
    const Vector4 origin = transform(model, Vector4{0.0, 0.0, 0.0, 1.0});
    EXPECT_NEAR(origin.x, position.x, 1e-12);
    EXPECT_NEAR(origin.y, position.y, 1e-12);
    EXPECT_NEAR(origin.z, position.z, 1e-12);

    // Scale then rotate then translate: a unit +X point scales by 3 along X.
    const Matrix4 scaled = compose_transform(position, Quaternion{}, Vector3{3.0, 1.0, 1.0});
    const Vector4 x = transform(scaled, Vector4{1.0, 0.0, 0.0, 1.0});
    EXPECT_NEAR(x.x, position.x + 3.0, 1e-12);
}
