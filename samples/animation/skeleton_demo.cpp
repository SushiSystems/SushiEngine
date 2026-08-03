/**************************************************************************/
/* skeleton_demo.cpp                                                     */
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
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// The phase-A0 animation foundations, worked and self-checked headlessly:
//   * The math seam additions — TRS decompose/recompose round-trips, an affine inverse
//     that reconstructs the identity, and slerp/nlerp endpoint and unit-length behaviour.
//   * The skeleton cook/load — a chain authored *out* of topological order is cooked to a
//     `.sushiskel` blob, then loaded back through IAnimationDatabase and checked to satisfy
//     parent[i] < i, to resolve joints by name hash, and — the end-to-end proof — to yield
//     an identity skin matrix at the bind pose (model-space compose x inverse-bind == I).

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    int failures = 0;

    void check(bool condition, const char* what)
    {
        if (!condition)
        {
            std::printf("[skeleton_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    bool nearly(double a, double b, double eps = 1e-5) { return std::fabs(a - b) <= eps; }

    bool matrices_equal(const Mat4& a, const Mat4& b, double eps = 1e-5)
    {
        for (int i = 0; i < 16; ++i)
            if (!nearly(a.m[i], b.m[i], eps))
                return false;
        return true;
    }

    Mat4 identity() { return Mat4{}; }

    // --- Math seam ------------------------------------------------------------------
    void test_math()
    {
        // TRS decompose is the inverse of compose for a well-formed matrix.
        const Vector3 t{1.5, -2.0, 3.25};
        const Quaternion r =
            quaternion_axis_angle(normalize(Vector3{0.3, 1.0, -0.6}), Scalar(0.7));
        const Vector3 s{2.0, 0.5, 1.25};
        const Mat4 composed = compose_transform(t, r, s);

        Vector3 dt{}, ds{};
        Quaternion dr{};
        decompose_transform(composed, dt, dr, ds);
        check(nearly(dt.x, t.x) && nearly(dt.y, t.y) && nearly(dt.z, t.z),
              "decompose translation");
        check(nearly(ds.x, s.x) && nearly(ds.y, s.y) && nearly(ds.z, s.z), "decompose scale");
        // Recomposing the decomposition must reproduce the original matrix (the quaternion
        // sign is free, so compare the matrices, not the quaternions).
        check(matrices_equal(compose_transform(dt, dr, ds), composed), "decompose recompose");

        // affine_inverse reconstructs the identity.
        check(matrices_equal(mul(composed, affine_inverse(composed)), identity(), 1e-4),
              "affine inverse");

        // quaternion_from_matrix inverts mat4_from_quaternion (up to sign).
        const Quaternion back = quaternion_from_matrix(mat4_from_quaternion(r));
        const Vector3 probe{0.4, -0.7, 0.55};
        check(length(rotate(back, probe) - rotate(r, probe)) < 1e-5, "quaternion from matrix");

        // Interpolation endpoints and unit length.
        const Quaternionf qa{0, 0, 0, 1};
        const Quaternionf qb{0.0f, 0.70710678f, 0.0f, 0.70710678f};
        const Quaternionf mid_s = slerp(qa, qb, 0.5f);
        const Quaternionf mid_n = nlerp(qa, qb, 0.5f);
        check(nearly(dot(mid_s, mid_s), 1.0, 1e-4), "slerp unit length");
        check(nearly(dot(mid_n, mid_n), 1.0, 1e-4), "nlerp unit length");
        const Quaternionf end = slerp(qa, qb, 1.0f);
        check(nearly(end.x, qb.x, 1e-4) && nearly(end.w, qb.w, 1e-4), "slerp endpoint");

        // Vector and scalar lerp.
        const Vector3f v = lerp(Vector3f{0, 0, 0}, Vector3f{2, 4, 8}, 0.25f);
        check(nearly(v.x, 0.5) && nearly(v.y, 1.0) && nearly(v.z, 2.0), "vector lerp");
        check(nearly(lerp(10.0f, 20.0f, 0.5f), 15.0), "scalar lerp");
    }

    // --- Skeleton cook / load -------------------------------------------------------
    void test_skeleton()
    {
        // A four-joint arm chain, authored leaf-first so the cook's topological sort has
        // real work to do: root is desc index 3, not 0.
        SkeletonDesc desc;
        JointDesc hand;     hand.name = "hand";     hand.parent = 1;
        hand.bind_translation = Vector3f{0.0f, 0.4f, 0.0f};
        JointDesc forearm;  forearm.name = "forearm";  forearm.parent = 2;
        forearm.bind_translation = Vector3f{0.0f, 0.5f, 0.0f};
        JointDesc upperarm; upperarm.name = "upperarm"; upperarm.parent = 3;
        upperarm.bind_translation = Vector3f{0.0f, 0.5f, 0.0f};
        JointDesc root;     root.name = "root";     root.parent = -1;
        root.bind_translation = Vector3f{0.0f, 1.0f, 0.0f};
        desc.joints = {hand, forearm, upperarm, root};

        std::vector<std::byte> blob;
        check(build_skeleton_blob(desc, blob), "cook succeeds");

        AnimationDatabase database;
        const AssetId id = database.add_skeleton(std::move(blob));
        check(id != INVALID_ASSET, "register skeleton");
        check(database.has_skeleton(id), "has skeleton");

        const SkeletonView skeleton = database.skeleton(id);
        check(skeleton.valid(), "view valid");
        check(skeleton.joint_count == 4, "joint count");
        check(skeleton.lod_count == 1 && skeleton.lod_joint_counts[0] == 4, "single full LOD");

        // Topological invariant: every parent precedes its child, root sorts to index 0.
        check(skeleton.parents[0] == NO_PARENT, "root sorts first");
        bool topological = true;
        for (std::uint32_t i = 0; i < skeleton.joint_count; ++i)
            if (skeleton.parents[i] != NO_PARENT && skeleton.parents[i] >= i)
                topological = false;
        check(topological, "parent[i] < i");

        // Name-hash lookup resolves each authored joint, and the chain is intact.
        const int root_index = skeleton.find_joint(hash_name("root"));
        const int hand_index = skeleton.find_joint(hash_name("hand"));
        check(root_index == 0, "find root");
        check(hand_index >= 0, "find hand");
        check(skeleton.find_joint(hash_name("missing")) < 0, "reject unknown joint");

        // The debug name table round-trips readable names for the editor.
        check(std::strcmp(skeleton.joint_name(static_cast<std::uint32_t>(root_index)), "root") == 0,
              "joint name string round-trip");
        check(std::strcmp(skeleton.joint_name(static_cast<std::uint32_t>(hand_index)), "hand") == 0,
              "hand name string round-trip");

        // End-to-end proof: model-space compose of the bind pose, times inverse-bind, is
        // the identity for every joint — a forward scan, since parent[i] < i.
        std::vector<Mat4> model(skeleton.joint_count);
        bool skin_is_identity = true;
        for (std::uint32_t i = 0; i < skeleton.joint_count; ++i)
        {
            const Vector3f& tt = skeleton.bind_translations[i];
            const Quaternionf& rr = skeleton.bind_rotations[i];
            const Vector3f& sc = skeleton.bind_scales[i];
            const Mat4 local = compose_transform(Vector3{tt.x, tt.y, tt.z},
                                                 Quaternion{rr.x, rr.y, rr.z, rr.w},
                                                 Vector3{sc.x, sc.y, sc.z});
            model[i] = skeleton.parents[i] == NO_PARENT
                           ? local
                           : mul(model[skeleton.parents[i]], local);
            const Mat4 skin = mul(model[i], to_mat4(skeleton.inverse_bind[i]));
            if (!matrices_equal(skin, identity(), 1e-4))
                skin_is_identity = false;
        }
        check(skin_is_identity, "bind-pose skin is identity");
    }
} // namespace

int main()
{
    test_math();
    test_skeleton();

    if (failures != 0)
    {
        std::printf("[skeleton_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[skeleton_demo] OK — math seam + skeleton cook/load/compose verified\n");
    return 0;
}
