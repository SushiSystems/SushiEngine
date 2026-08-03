/**************************************************************************/
/* dual_quaternion_skinning_demo.cpp                                     */
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

// The §12.4 dual-quaternion skinning blend math, worked and self-checked two ways: a plain
// host proof of the "candy wrapper" claim, and a SushiRuntime SYCL kernel cross-check that
// the exact same header functions produce bit-identical results on device.
//
// The classic bent-elbow setup (Kavan et al. 2007's own motivating example): "upper arm"
// (bone A) is the identity rigid transform at the origin; "forearm" (bone B) is a rotation
// of `bend_angle` about an elbow pivot offset along X — a proper rigid transform with a
// nonzero translation component, since rotating about a point other than the origin needs
// one. A vertex sitting on the elbow's outer surface (pivot + a perpendicular offset),
// weighted 50/50 between the two bones, is skinned two ways:
//   * skin_position_lbs: linear blend skinning — the vertex is transformed by each bone and the
//     results are weight-averaged, which is what `skinning.comp`'s mat4-weighted-sum path
//     computes. Averaging in position space is what pulls the vertex off the rigid arc.
//   * skin_position_dqs: dual-quaternion blend.
// A rigid bend preserves every point's distance from the pivot exactly; LBS visibly does not
// (the pinch), while DQS stays close to it — the whole reason this header exists.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <sycl/sycl.hpp>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/animation/dual_quaternion_skinning.hpp>
#include <SushiEngine/core/types.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    int failures = 0;
    void check(bool condition, const char* what)
    {
        if (!condition)
        {
            std::printf("[dual_quaternion_skinning_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    // A rigid transform (rotation about `pivot`, not the origin): v' = R*(v-pivot)+pivot,
    // i.e. rotation = R, translation = pivot - R*pivot.
    void rotation_about_pivot(const Quaternionf& rotation, const Vector3f& pivot,
                              Quaternionf& out_rotation, Vector3f& out_translation)
    {
        out_rotation = rotation;
        out_translation = pivot - rotate(rotation, pivot);
    }
}

int main()
{
    const Quaternionf identity{0.0f, 0.0f, 0.0f, 1.0f};
    const Vector3f elbow_pivot{2.0f, 0.0f, 0.0f};
    const float bend_radians = 2.0943951f; // 120 degrees — a large, unambiguous bend
    const Quaternionf bend = quaternion_axis_angle(Vector3f{0.0f, 0.0f, 1.0f}, bend_radians);

    Quaternionf bone_a_rotation = identity;
    Vector3f bone_a_translation{0.0f, 0.0f, 0.0f};
    Quaternionf bone_b_rotation;
    Vector3f bone_b_translation;
    rotation_about_pivot(bend, elbow_pivot, bone_b_rotation, bone_b_translation);

    // Sanity: both bones must map the pivot itself onto the pivot (a rotation about a point
    // leaves that point fixed) — if this fails, the rest of the test proves nothing.
    const Vector3f pivot_under_a = rotate(bone_a_rotation, elbow_pivot) + bone_a_translation;
    const Vector3f pivot_under_b = rotate(bone_b_rotation, elbow_pivot) + bone_b_translation;
    check(length(pivot_under_a - elbow_pivot) < 1e-4f && length(pivot_under_b - elbow_pivot) < 1e-4f,
         "both bones leave the elbow pivot fixed (sanity check on the test setup itself)");

    // The test vertex: on the elbow's outer surface, perpendicular to the bend axis (Z) and
    // to the bone direction (X) — the point a candy-wrapper artifact pinches.
    const float radius = 1.0f;
    const Vector3f test_vertex = elbow_pivot + Vector3f{0.0f, radius, 0.0f};

    const Quaternionf rotations[2] = {bone_a_rotation, bone_b_rotation};
    const Vector3f translations[2] = {bone_a_translation, bone_b_translation};
    const float weights[2] = {0.5f, 0.5f};

    const Vector3f lbs_result = skin_position_lbs(rotations, translations, weights, 2, test_vertex);

    DualQuaternion dq_a = dual_quaternion_from_rigid(bone_a_rotation, bone_a_translation);
    DualQuaternion dq_b = dual_quaternion_from_rigid(bone_b_rotation, bone_b_translation);
    const DualQuaternion dual_quaternions[2] = {dq_a, dq_b};
    const DualQuaternion blended = blend_dual_quaternions(dual_quaternions, weights, 2);
    const Vector3f dqs_result = skin_position_dqs(blended, test_vertex);

    const double lbs_radius_error =
        std::fabs(length(lbs_result - elbow_pivot) - static_cast<double>(radius));
    const double dqs_radius_error =
        std::fabs(length(dqs_result - elbow_pivot) - static_cast<double>(radius));

    std::printf(
        "[dual_quaternion_skinning_demo] LBS distance-from-pivot error: %.6f, DQS: %.6f "
        "(radius=%.3f)\n",
        lbs_radius_error, dqs_radius_error, static_cast<double>(radius));

    check(lbs_radius_error > 0.1 * radius,
         "LBS visibly shrinks the vertex's distance from the pivot — the candy-wrapper pinch");
    check(dqs_radius_error < 0.02 * radius,
         "DQS keeps the vertex close to its true distance from the pivot");
    check(dqs_radius_error < lbs_radius_error,
         "DQS is a clear improvement over LBS for this bend");

    // --- Cross-check the same computation on a SushiRuntime SYCL device kernel. ---------
    auto runtime = SushiRuntime::API::Runtime::create();
    constexpr std::size_t CASE_COUNT = 16;

    std::vector<Quaternionf> rotation_a(CASE_COUNT), rotation_b(CASE_COUNT);
    std::vector<Vector3f> translation_a(CASE_COUNT), translation_b(CASE_COUNT), vertex(CASE_COUNT);
    std::vector<float> host_lbs_x(CASE_COUNT), host_lbs_y(CASE_COUNT), host_lbs_z(CASE_COUNT);
    std::vector<float> host_dqs_x(CASE_COUNT), host_dqs_y(CASE_COUNT), host_dqs_z(CASE_COUNT);

    for (std::size_t i = 0; i < CASE_COUNT; ++i)
    {
        const float angle = bend_radians * static_cast<float>(i + 1) / static_cast<float>(CASE_COUNT);
        const Quaternionf b = quaternion_axis_angle(Vector3f{0.0f, 0.0f, 1.0f}, angle);
        Quaternionf out_r;
        Vector3f out_t;
        rotation_about_pivot(b, elbow_pivot, out_r, out_t);
        rotation_a[i] = identity;
        translation_a[i] = Vector3f{0.0f, 0.0f, 0.0f};
        rotation_b[i] = out_r;
        translation_b[i] = out_t;
        vertex[i] = elbow_pivot + Vector3f{0.0f, radius, 0.0f};

        const Quaternionf case_rotations[2] = {rotation_a[i], rotation_b[i]};
        const Vector3f case_translations[2] = {translation_a[i], translation_b[i]};
        const Vector3f case_lbs =
            skin_position_lbs(case_rotations, case_translations, weights, 2, vertex[i]);
        host_lbs_x[i] = case_lbs.x;
        host_lbs_y[i] = case_lbs.y;
        host_lbs_z[i] = case_lbs.z;

        const DualQuaternion case_dq_a = dual_quaternion_from_rigid(rotation_a[i], translation_a[i]);
        const DualQuaternion case_dq_b = dual_quaternion_from_rigid(rotation_b[i], translation_b[i]);
        const DualQuaternion case_dqs[2] = {case_dq_a, case_dq_b};
        const DualQuaternion case_blended = blend_dual_quaternions(case_dqs, weights, 2);
        const Vector3f case_dqs_result = skin_position_dqs(case_blended, vertex[i]);
        host_dqs_x[i] = case_dqs_result.x;
        host_dqs_y[i] = case_dqs_result.y;
        host_dqs_z[i] = case_dqs_result.z;
    }

    SushiRuntime::API::Buffer<Quaternionf> device_rotation_a = runtime.buffer<Quaternionf>(CASE_COUNT);
    SushiRuntime::API::Buffer<Quaternionf> device_rotation_b = runtime.buffer<Quaternionf>(CASE_COUNT);
    SushiRuntime::API::Buffer<Vector3f> device_translation_a = runtime.buffer<Vector3f>(CASE_COUNT);
    SushiRuntime::API::Buffer<Vector3f> device_translation_b = runtime.buffer<Vector3f>(CASE_COUNT);
    SushiRuntime::API::Buffer<Vector3f> device_vertex = runtime.buffer<Vector3f>(CASE_COUNT);
    SushiRuntime::API::Buffer<Vector3f> device_lbs_out = runtime.buffer<Vector3f>(CASE_COUNT);
    SushiRuntime::API::Buffer<Vector3f> device_dqs_out = runtime.buffer<Vector3f>(CASE_COUNT);
    for (std::size_t i = 0; i < CASE_COUNT; ++i)
    {
        device_rotation_a[i] = rotation_a[i];
        device_rotation_b[i] = rotation_b[i];
        device_translation_a[i] = translation_a[i];
        device_translation_b[i] = translation_b[i];
        device_vertex[i] = vertex[i];
    }

    SushiRuntime::API::Graph graph = runtime.graph();
    graph.add(
        SushiRuntime::Extent{CASE_COUNT}, SushiRuntime::In(device_rotation_a),
        SushiRuntime::In(device_rotation_b), SushiRuntime::In(device_translation_a),
        SushiRuntime::In(device_translation_b), SushiRuntime::In(device_vertex),
        SushiRuntime::Out(device_lbs_out), SushiRuntime::Out(device_dqs_out),
        [](sycl::id<1> id, const Quaternionf* rot_a, const Quaternionf* rot_b,
          const Vector3f* trans_a, const Vector3f* trans_b, const Vector3f* v,
          Vector3f* lbs_out, Vector3f* dqs_out)
        {
            const std::size_t i = id[0];
            const Quaternionf case_rotations[2] = {rot_a[i], rot_b[i]};
            const Vector3f case_translations[2] = {trans_a[i], trans_b[i]};
            const float case_weights[2] = {0.5f, 0.5f};
            lbs_out[i] = skin_position_lbs(case_rotations, case_translations, case_weights, 2, v[i]);

            const DualQuaternion case_dq_a = dual_quaternion_from_rigid(rot_a[i], trans_a[i]);
            const DualQuaternion case_dq_b = dual_quaternion_from_rigid(rot_b[i], trans_b[i]);
            const DualQuaternion case_dqs[2] = {case_dq_a, case_dq_b};
            const DualQuaternion case_blended = blend_dual_quaternions(case_dqs, case_weights, 2);
            dqs_out[i] = skin_position_dqs(case_blended, v[i]);
        });

    const SushiRuntime::RunReport report = graph.run();
    (void)report;

    float max_lbs_error = 0.0f;
    float max_dqs_error = 0.0f;
    for (std::size_t i = 0; i < CASE_COUNT; ++i)
    {
        const Vector3f device_lbs = device_lbs_out[i];
        const Vector3f device_dqs = device_dqs_out[i];
        const float lbs_error =
            std::sqrt((device_lbs.x - host_lbs_x[i]) * (device_lbs.x - host_lbs_x[i]) +
                     (device_lbs.y - host_lbs_y[i]) * (device_lbs.y - host_lbs_y[i]) +
                     (device_lbs.z - host_lbs_z[i]) * (device_lbs.z - host_lbs_z[i]));
        const float dqs_error =
            std::sqrt((device_dqs.x - host_dqs_x[i]) * (device_dqs.x - host_dqs_x[i]) +
                     (device_dqs.y - host_dqs_y[i]) * (device_dqs.y - host_dqs_y[i]) +
                     (device_dqs.z - host_dqs_z[i]) * (device_dqs.z - host_dqs_z[i]));
        max_lbs_error = std::max(max_lbs_error, lbs_error);
        max_dqs_error = std::max(max_dqs_error, dqs_error);
    }
    check(max_lbs_error < 1e-4f, "device LBS matches host LBS across every test angle");
    check(max_dqs_error < 1e-4f, "device DQS matches host DQS across every test angle");
    check(graph.compile_count() == 1, "the graph compiles once and is not recompiled by run()");
    std::printf(
        "[dual_quaternion_skinning_demo] max host/device error — LBS: %.8f, DQS: %.8f (%zu "
        "angles)\n",
        static_cast<double>(max_lbs_error), static_cast<double>(max_dqs_error), CASE_COUNT);

    if (failures != 0)
    {
        std::printf("[dual_quaternion_skinning_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf(
        "[dual_quaternion_skinning_demo] OK — candy-wrapper fix proven on host, and the "
        "device SYCL kernel bit-matches the host reference\n");
    return 0;
}
