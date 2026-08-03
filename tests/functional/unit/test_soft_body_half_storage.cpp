/**************************************************************************/
/* test_soft_body_half_storage.cpp                                        */
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

// §6.5's second half: `sycl::half` for a cosmetic body's *stored* pose, `float`
// for everything a projection does with it. Three things earn a case here —
// that widening what was narrowed loses only what half precision already
// costs, that the storage type is actually half-width rather than a `float`
// wearing a smaller name, and that routing a real trajectory through the
// narrow/widen seam once a tick stays close to the same trajectory computed
// with no seam at all.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <sycl/sycl.hpp>

#include <SushiEngine/physics/soft/soft_body_half_storage.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief A one-tetrahedron asset, enough to instantiate a `float` model. */
    struct TinyAsset
    {
        std::vector<Vector3> vertices;
        std::vector<Scalar> mass;
        std::vector<std::uint32_t> tetrahedra;
        std::vector<Vector3> rest_inverse;
        std::vector<Scalar> rest_volume;
        std::vector<Cooking::SoftBodyLevelRecord> levels;
        Cooking::SoftBodyAssetView view;

        TinyAsset()
        {
            vertices = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1}};
            mass = {Scalar(1), Scalar(1), Scalar(1), Scalar(1)};
            tetrahedra = {0, 1, 2, 3};
            rest_inverse = {Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1}};
            rest_volume = {Scalar(1.0 / 6.0)};

            Cooking::SoftBodyLevelRecord level{};
            level.first_vertex = 0;
            level.vertex_count = 4;
            level.first_tetrahedron = 0;
            level.tetrahedron_count = 1;
            levels.push_back(level);

            view.levels = levels.data();
            view.level_count = 1;
            view.vertices = vertices.data();
            view.vertex_mass = mass.data();
            view.vertex_count = 4;
            view.tetrahedra = tetrahedra.data();
            view.rest_inverse = rest_inverse.data();
            view.rest_volume = rest_volume.data();
            view.tetrahedron_count = 1;
            view.valid = true;
        }
    };

    /**
     * @brief The largest relative error a round-to-nearest `half` narrowing can add.
     *
     * A `sycl::half` mantissa is ten explicit bits plus its implicit leading
     * one, so the gap between adjacent representable values near a magnitude
     * `m` is `m * 2^-10`; round-to-nearest can be off by at most half that
     * gap. `4e-3` is a deliberately loose multiple of the `2^-11 ≈ 4.9e-4`
     * theoretical bound — this environment cannot compile and run the
     * concrete `sycl::half` rounding this assumes, so the margin is meant to
     * absorb an implementation detail this test did not anticipate rather
     * than to be a tight measurement.
     */
    constexpr float HALF_RELATIVE_ERROR_BOUND = 4e-3f;

    void expect_round_trip_within_half_precision(float value)
    {
        const Vector3T<float> original{value, -0.5f * value, 2.0f * value};
        const HalfVector3 stored = narrow_to_half_vector3(original);
        const Vector3T<float> restored = widen_half_vector3(stored);

        const float scale = std::fabs(value) > 1e-6f ? std::fabs(value) : 1e-6f;
        EXPECT_NEAR(restored.x, original.x, HALF_RELATIVE_ERROR_BOUND * scale)
            << "x diverged for value " << value;
        EXPECT_NEAR(restored.y, original.y, HALF_RELATIVE_ERROR_BOUND * scale)
            << "y diverged for value " << value;
        EXPECT_NEAR(restored.z, original.z, HALF_RELATIVE_ERROR_BOUND * scale)
            << "z diverged for value " << value;
    }
} // namespace

TEST(Unit_SoftBodyHalfStorage, ZeroRoundTripsExactly)
{
    // Zero is exactly representable at every IEEE width, so this is the one
    // case the bound above must hold at with no slack at all.
    const HalfVector3 stored = narrow_to_half_vector3(Vector3T<float>{0.0f, 0.0f, 0.0f});
    const Vector3T<float> restored = widen_half_vector3(stored);
    EXPECT_EQ(restored.x, 0.0f);
    EXPECT_EQ(restored.y, 0.0f);
    EXPECT_EQ(restored.z, 0.0f);
}

TEST(Unit_SoftBodyHalfStorage, RoundTripStaysWithinHalfPrecisionAcrossRepresentativeMagnitudes)
{
    // A soft body's cosmetic column spans millimetre detail (cloth, a curtain
    // fold) up to a vehicle shell's metre scale, so the representative set
    // below is chosen to bracket that range rather than to pick one
    // convenient number.
    const float magnitudes[] = {1e-3f, 1e-2f, 0.1f, 1.0f, 9.81f, 100.0f, 1000.0f};
    for (const float magnitude : magnitudes)
    {
        expect_round_trip_within_half_precision(magnitude);
        expect_round_trip_within_half_precision(-magnitude);
    }
}

TEST(Unit_SoftBodyHalfStorage, TheStorageTypeIsActuallyHalfWidth)
{
    // The one check that catches a `HalfVector3` that quietly became a
    // `float` wearing a smaller name: three `sycl::half` lanes and nothing
    // else, so its size is exactly half of the `float` column it mirrors.
    EXPECT_EQ(sizeof(HalfVector3), sizeof(sycl::half) * 3);
    EXPECT_EQ(sizeof(HalfVector3) * 2, sizeof(Vector3T<float>));
}

TEST(Unit_SoftBodyHalfStorage, StorageSizesItselfToTheModelItMirrors)
{
    const TinyAsset asset;
    const SoftBodyMaterialT<float> material;
    const FiniteElementModel<float> model = build_finite_element_model<float>(
        asset.view, 0u, material, Vector3T<float>{0, 0, 0});
    ASSERT_EQ(model.particles.size(), 4u);

    SoftBodyHalfStorage storage;
    storage.narrow_from(model);
    EXPECT_EQ(storage.particle_count(), model.particles.size());
    EXPECT_EQ(storage.positions().size(), model.particles.size());
    EXPECT_EQ(storage.velocities().size(), model.particles.size());

    // Positions and velocities together, at half width, must weigh less than
    // the `float` particle array they were narrowed from — the entire point
    // of the type, made concrete rather than asserted by name alone.
    const std::size_t half_bytes =
        storage.positions().size() * sizeof(HalfVector3) +
        storage.velocities().size() * sizeof(HalfVector3);
    const std::size_t float_bytes = model.particles.size() * sizeof(Vector3T<float>) * 2;
    EXPECT_LT(half_bytes, float_bytes);
}

TEST(Unit_SoftBodyHalfStorage, WidenIntoRestoresARestPoseWithinTolerance)
{
    // The asset's own rest pose, round-tripped with no tick in between: the
    // narrowest possible case, and the one a curtain hanging at rest would
    // actually exercise every time its storage is touched without moving.
    const TinyAsset asset;
    const SoftBodyMaterialT<float> material;
    FiniteElementModel<float> model = build_finite_element_model<float>(
        asset.view, 0u, material, Vector3T<float>{0, 0, 0});

    SoftBodyHalfStorage storage;
    storage.narrow_from(model);

    FiniteElementModel<float> restored = model;
    for (RigidBodyT<float>& particle : restored.particles)
        particle.position = Vector3T<float>{0, 0, 0};
    storage.widen_into(restored);

    for (std::size_t i = 0; i < model.particles.size(); ++i)
    {
        const Vector3T<float> difference =
            restored.particles[i].position - model.particles[i].position;
        const float error =
            std::sqrt(difference.x * difference.x + difference.y * difference.y +
                       difference.z * difference.z);
        EXPECT_LT(error, 1e-2f) << "particle " << i << " lost more than half precision explains";
    }
}

TEST(Unit_SoftBodyHalfStorage, FreeFallThroughTheHalfStorageSeamStaysCloseToPureFloat)
{
    // Not a determinism claim, exactly as the `float`-vs-`double` case in
    // test_soft_body_precision.cpp is not one — a body routed through the
    // narrow/widen seam once a tick is allowed to differ from one that never
    // is, and what is asserted is that it stays the *same simulation*: a
    // curtain that drifted centimetres off from decorating its own scene
    // would not be usable even as decoration.
    //
    // The bound below is derived, not measured, because this environment
    // cannot build and run the `sycl::half` rounding it depends on. Each
    // tick's narrow/widen round trip perturbs position and velocity by at
    // most a few parts in a thousand (see HALF_RELATIVE_ERROR_BOUND above);
    // this scene is an unpinned tetrahedron falling under a uniform
    // acceleration, so every particle moves identically and no relative
    // deformation drives the projection's nonlinear terms — the perturbation
    // therefore does not compound the way it does when two differently-sized
    // errors disagree about the body's shape. Ten ticks of that bounded a
    // per-tick perturbation stay within a generous multiple of a single
    // tick's own rounding, hence the tolerance is stated as a fraction of
    // distance fallen with substantial headroom rather than derived to be
    // tight. Whoever first runs this should tighten it against the real
    // number.
    const TinyAsset asset;
    const SoftBodyMaterialT<float> material;

    FiniteElementModel<float> pure_float = build_finite_element_model<float>(
        asset.view, 0u, material, Vector3T<float>{0, 0, 0});
    FiniteElementModel<float> half_stored = pure_float;

    pure_float.set_external_acceleration(Vector3T<float>{0, 0, -9.81f});
    half_stored.set_external_acceleration(Vector3T<float>{0, 0, -9.81f});

    SoftBodyHalfStorage storage;
    storage.narrow_from(half_stored);

    constexpr int TICKS = 10;
    constexpr int SUBSTEPS = 10;
    constexpr float DELTA_TIME = 1.0f / 60.0f;
    for (int tick = 0; tick < TICKS; ++tick)
    {
        pure_float.step(DELTA_TIME, SUBSTEPS);

        storage.widen_into(half_stored);
        half_stored.step(DELTA_TIME, SUBSTEPS);
        storage.narrow_from(half_stored);
    }

    float fallen = 0.0f;
    for (const RigidBodyT<float>& particle : pure_float.particles)
        fallen = std::max(fallen, -particle.position.z);
    fallen = std::max(fallen, 1e-3f);

    for (std::size_t i = 0; i < pure_float.particles.size(); ++i)
    {
        const Vector3T<float> difference =
            pure_float.particles[i].position - half_stored.particles[i].position;
        const float divergence = std::sqrt(difference.x * difference.x +
                                            difference.y * difference.y +
                                            difference.z * difference.z);
        EXPECT_LT(divergence, 0.05f * fallen)
            << "particle " << i << " diverged past what the half-storage seam explains, after "
            << "falling " << fallen << " m";
    }
}
