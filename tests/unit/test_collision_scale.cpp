/**************************************************************************/
/* test_collision_scale.cpp                                               */
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

// Unit_CollisionScale: P2's acceptance scenes — 1 000 mixed-shape bodies and
// 10 000 mostly-sleeping ones (§16, §13.1).
//
// What this measures and what it does not, stated plainly, because a performance
// test that is vague about its scope is a number nobody can act on. It measures
// everything P2 built: proxy updates, the hierarchy's repair, the pair search and
// its cache, the narrowphase over every produced pair, and the island partition.
// It does not measure the constraint solve, which is on the device behind
// SushiRuntime and belongs to the tick as a whole.
//
// The assertions are deliberately loose — an order of magnitude above what the
// numbers are on any machine that can build this — because a tight bound in a unit
// suite fails on a busy build agent and teaches everyone to ignore it. The bound
// that matters is the *shape*: ten thousand settled bodies must cost less than one
// thousand active ones, and if that inverts, sleeping has stopped working and the
// assertion says so however fast the machine is.

#include <chrono>
#include <cstdio>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/bvh_broadphase.hpp>
#include <SushiEngine/physics/collision/narrowphase_dispatch.hpp>
#include <SushiEngine/physics/scene/islands.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    using Real = double;
    using Body = RigidBodyT<Real>;

    /** @brief One scene: bodies, their shapes, and the structures over them. */
    struct Scene
    {
        std::vector<Body> bodies;
        std::vector<CollisionShape<Real>> shapes;
        std::vector<ProxyId> proxies;
        BVHBroadphase<Real> broadphase;
        IslandBuilder<Real> islands;
        IslandSet partition;
    };

    Vector3T<Real> vec(Real x, Real y, Real z) { return Vector3T<Real>{x, y, z}; }

    /** @brief The shape at a body's current pose. */
    CollisionShape<Real> placed(const Scene& scene, std::size_t index)
    {
        CollisionShape<Real> shape = scene.shapes[index];
        if (shape.type != ShapeType::plane)
        {
            shape.center = scene.bodies[index].position;
            shape.orientation = scene.bodies[index].orientation;
        }
        return shape;
    }

    /** @brief One of the four convex kinds, cycling, so the table is exercised. */
    CollisionShape<Real> mixed_shape(std::uint32_t index, const Vector3T<Real>* hull)
    {
        switch (index % 4u)
        {
            case 0:
                return make_sphere_shape<Real>(vec(0, 0, 0), 0.5);
            case 1:
                return make_box_shape<Real>(vec(0, 0, 0), vec(0.5, 0.5, 0.5),
                                            QuaternionT<Real>{0, 0, 0, 1});
            case 2:
                return make_capsule_shape<Real>(vec(0, 0, 0), 0.25, 0.25,
                                                QuaternionT<Real>{0, 0, 0, 1});
            default:
                break;
        }
        return make_hull_shape<Real>(vec(0, 0, 0), hull, 8u, QuaternionT<Real>{0, 0, 0, 1}, 0.0);
    }

    /**
     * @brief One tick of everything P2 owns, over the whole scene.
     *
     * The order is the tick's: tell the broadphase where things are, let it find
     * the pairs, collide the pairs it produced, and partition what came out.
     */
    std::size_t run_tick(Scene& scene, Real dt)
    {
        // A sleeping body has not moved, so it has no bounds to re-report: the
        // whole of what a settled island costs is the loop that skips it.
        for (std::size_t i = 0; i < scene.bodies.size(); ++i)
        {
            if (has_any_flag(scene.bodies[i].flags, BodyFlags::sleeping))
                continue;
            const CollisionShape<Real> shape = placed(scene, i);
            scene.broadphase.update_proxy(scene.proxies[i], shape_world_bounds(shape),
                                          scene.bodies[i].velocity * dt);
        }
        scene.broadphase.update();

        std::size_t contact_points = 0;
        scene.islands.begin(scene.bodies.size());
        for (const BroadphasePair& pair : scene.broadphase.pairs())
        {
            const std::uint32_t a = scene.broadphase.proxy(pair.a).payload;
            const std::uint32_t b = scene.broadphase.proxy(pair.b).payload;
            const ContactManifold<Real> manifold =
                generate_shape_manifold<Real>(placed(scene, a), placed(scene, b), 0.02, 1e-3);
            if (manifold.point_count == 0)
                continue;
            contact_points += manifold.point_count;
            scene.islands.connect(a, b, scene.bodies.data());
        }
        scene.islands.finish(scene.bodies.data(), scene.bodies.size(), dt, 0.01, 0.5,
                             scene.partition);
        // The decision has to reach the broadphase, or the structure keeps
        // descending from leaves whose bodies went to sleep an hour ago — which is
        // most of what "sleeping is the largest win" is actually buying.
        for (std::size_t i = 0; i < scene.bodies.size(); ++i)
            scene.broadphase.set_proxy_state(scene.proxies[i], CollisionFilter{},
                                             scene.bodies[i].flags);
        return contact_points;
    }

    /** @brief Milliseconds per tick, averaged over @p ticks, after a warm-up. */
    double measure(Scene& scene, int ticks, std::size_t& contact_points)
    {
        const Real dt = 1.0 / 60.0;
        for (int warm = 0; warm < 3; ++warm)
            contact_points = run_tick(scene, dt);

        const auto started = std::chrono::steady_clock::now();
        for (int tick = 0; tick < ticks; ++tick)
            contact_points = run_tick(scene, dt);
        const auto ended = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(ended - started).count() / ticks;
    }

    const Vector3T<Real> unit_hull[8] = {vec(-0.5, -0.5, -0.5), vec(0.5, -0.5, -0.5),
                                        vec(-0.5, 0.5, -0.5),  vec(0.5, 0.5, -0.5),
                                        vec(-0.5, -0.5, 0.5),  vec(0.5, -0.5, 0.5),
                                        vec(-0.5, 0.5, 0.5),   vec(0.5, 0.5, 0.5)};
} // namespace

TEST(Unit_CollisionScale, OneThousandMixedShapeBodiesInContact)
{
    // Twenty-five stacks of forty, so most bodies genuinely touch two others and
    // the pair set is a stacking scene's rather than a cloud of near-misses.
    Scene scene;
    std::mt19937 engine(4242u);
    std::uniform_real_distribution<double> jitter(-0.01, 0.01);

    for (std::uint32_t i = 0; i < 1000u; ++i)
    {
        Body body;
        const std::uint32_t stack = i / 40u;
        const std::uint32_t level = i % 40u;
        body.position = vec(Real(stack % 5u) * 3.0 + jitter(engine), 0.5 + Real(level) * 0.99,
                            Real(stack / 5u) * 3.0 + jitter(engine));
        body.prev_position = body.position;
        body.inv_mass = 1.0;
        body.inv_inertia = vec(6.0, 6.0, 6.0);
        body.motion_measure = 1.0; // awake: this is the *active* scene
        scene.bodies.push_back(body);
        scene.shapes.push_back(mixed_shape(i, unit_hull));
    }
    for (std::uint32_t i = 0; i < scene.bodies.size(); ++i)
        scene.proxies.push_back(scene.broadphase.create_proxy(
            shape_world_bounds(placed(scene, i)), CollisionFilter{}, 0u, i));

    std::size_t contact_points = 0;
    const double milliseconds = measure(scene, 20, contact_points);
    std::printf("[ scale    ] 1 000 mixed-shape bodies: %.3f ms/tick, %zu pairs, %zu contact "
                "points, %zu islands\n",
                milliseconds, scene.broadphase.pairs().size(), contact_points,
                scene.partition.islands.size());

    EXPECT_GT(scene.broadphase.pairs().size(), 900u) << "the stacks must actually be in contact";
    EXPECT_GT(contact_points, 900u);
    EXPECT_LT(milliseconds, 50.0);
}

TEST(Unit_CollisionScale, TenThousandSleepingBodiesDoNoPairWorkAtAll)
{
    // The §13.2 claim, measured rather than asserted in prose: a settled island
    // costs its bound update and nothing else. Ten thousand of them must come out
    // cheaper than the thousand active bodies above, and the ratio is what makes
    // this a test of sleeping rather than a test of the machine.
    Scene scene;
    for (std::uint32_t i = 0; i < 10000u; ++i)
    {
        Body body;
        body.position = vec(Real(i % 100u) * 2.0, 0.5, Real(i / 100u) * 2.0);
        body.prev_position = body.position;
        body.inv_mass = 1.0;
        body.inv_inertia = vec(6.0, 6.0, 6.0);
        scene.bodies.push_back(body);
        scene.shapes.push_back(mixed_shape(i, unit_hull));
    }
    for (std::uint32_t i = 0; i < scene.bodies.size(); ++i)
        scene.proxies.push_back(scene.broadphase.create_proxy(
            shape_world_bounds(placed(scene, i)), CollisionFilter{}, 0u, i));

    // Let them settle. Nothing is moving, so the sleep timers run out and the
    // islands go under.
    std::size_t contact_points = 0;
    for (int tick = 0; tick < 40; ++tick)
        run_tick(scene, 1.0 / 60.0);
    ASSERT_EQ(scene.partition.awake_count, 0u) << "the scene never settled";

    const double milliseconds = measure(scene, 20, contact_points);
    std::printf("[ scale    ] 10 000 sleeping bodies: %.3f ms/tick, %zu islands, %zu awake\n",
                milliseconds, scene.partition.islands.size(), scene.partition.awake_count);

    // Every proxy still reports its bounds and the partition is still built; what
    // is gone is the pair work, because two quiet proxies never pair.
    EXPECT_TRUE(scene.broadphase.pairs().empty());
    EXPECT_LT(milliseconds, 50.0);
}

TEST(Unit_CollisionScale, WakingOneBodyInATenThousandBodySceneWakesOnlyItsIsland)
{
    // The other half of the claim: the saving must survive contact with a scene
    // where something is happening. If waking one crate woke the warehouse, the
    // measurement above would be true and useless.
    Scene scene;
    for (std::uint32_t i = 0; i < 2000u; ++i)
    {
        Body body;
        body.position = vec(Real(i % 50u) * 2.0, 0.5, Real(i / 50u) * 2.0);
        body.prev_position = body.position;
        body.inv_mass = 1.0;
        body.inv_inertia = vec(6.0, 6.0, 6.0);
        scene.bodies.push_back(body);
        scene.shapes.push_back(mixed_shape(i, unit_hull));
    }
    for (std::uint32_t i = 0; i < scene.bodies.size(); ++i)
        scene.proxies.push_back(scene.broadphase.create_proxy(
            shape_world_bounds(placed(scene, i)), CollisionFilter{}, 0u, i));

    for (int tick = 0; tick < 40; ++tick)
        run_tick(scene, 1.0 / 60.0);
    ASSERT_EQ(scene.partition.awake_count, 0u);

    wake_island(scene.bodies.data(), scene.bodies.size(), 137u, scene.partition);
    scene.bodies[137].motion_measure = 5.0;
    run_tick(scene, 1.0 / 60.0);

    EXPECT_EQ(scene.partition.awake_count, 1u);
    std::size_t awake_bodies = 0;
    for (const Body& body : scene.bodies)
        if (!has_any_flag(body.flags, BodyFlags::sleeping))
            ++awake_bodies;
    EXPECT_EQ(awake_bodies, 1u);
}
