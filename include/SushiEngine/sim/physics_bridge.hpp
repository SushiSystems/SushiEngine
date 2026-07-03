/**************************************************************************/
/* physics_bridge.hpp                                                    */
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
 * @file physics_bridge.hpp
 * @brief The ECS-facing half of the physics seam: entity <-> `Physics::RigidBody`.
 *
 * `physics/physics_world.hpp` deliberately knows nothing about the ECS (see
 * ARCHITECTURE.md §4.1); this file is the glue that lives on the other side of
 * that seam, in `sim/` where a dependency on both `ecs/` and `physics/` is normal.
 * `PhysicsBody` names which of an entity's `PhysicsWorld` body a physics-driven
 * entity owns; `initial_rigid_body` builds that body's starting state from the
 * entity's current `Transform`/`Orientation` at registration time, and
 * `sync_transforms_from_physics` writes the solved pose back every tick. There is
 * deliberately no reverse (ECS -> physics) sync yet: nothing today needs to teleport
 * a physics-driven entity by editing its `Transform` directly.
 */

#include <cstddef>
#include <cstdint>
#include <memory>

#include <SushiEngine/ecs/archetype.hpp>
#include <SushiEngine/ecs/chunk.hpp>
#include <SushiEngine/ecs/component.hpp>
#include <SushiEngine/ecs/entity.hpp>
#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/physics/physics_world.hpp>
#include <SushiEngine/physics/rigid_body.hpp>
#include <SushiEngine/sim/components.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief Names the `Physics::PhysicsWorld` body a physics-driven entity owns.
         *
         * A component like any other, so it is added at spawn time
         * (`world.spawn(Transform{}, Orientation{}, PhysicsBody{})`) alongside the pose
         * components it drives. `INVALID` marks an entity that carries the component
         * but is not (yet) registered with a body, which `sync_transforms_from_physics`
         * skips rather than treating as body index 0.
         */
        struct PhysicsBody
        {
            static constexpr std::uint32_t INVALID = ~std::uint32_t(0);
            std::uint32_t body_id = INVALID;
        };

        /**
         * @brief Builds a body's starting XPBD state from an entity's current pose.
         *
         * Called once, at registration (`PhysicsWorld::add_body`), not per tick — after
         * that, the entity's `Transform`/`Orientation` are outputs of the simulation
         * (via `sync_transforms_from_physics`), not inputs to it.
         *
         * @param world      The world owning @p entity's components.
         * @param entity     The entity to read the starting pose from.
         * @param inv_mass   The new body's inverse mass (0 pins it).
         * @param inv_inertia The new body's diagonal body-local inverse inertia (0,0,0
         *                    for a point mass with no angular degrees of freedom).
         * @return A `RigidBody` at the entity's current pose, at rest.
         */
        inline Physics::RigidBody initial_rigid_body(const World& world, Entity entity,
                                                      Scalar inv_mass,
                                                      Vector3 inv_inertia = Vector3{0, 0, 0})
        {
            Physics::RigidBody body;
            body.position = world.get<Transform>(entity).position;
            body.orientation = world.get<Orientation>(entity).rotation;
            body.inv_mass = inv_mass;
            body.inv_inertia = inv_inertia;
            return body;
        }

        /**
         * @brief Writes every physics-driven entity's solved pose back into the ECS.
         *
         * The one-directional half of the bridge: `Physics::PhysicsWorld` is the
         * source of truth for a physics-driven entity's pose, so this copies
         * `RigidBody::position`/`orientation` into `Transform::position`/
         * `Orientation::rotation` for every live entity carrying `PhysicsBody`. Call
         * once per tick, after `PhysicsWorld::step()`, before the render extract that
         * reads `Transform`/`Orientation`.
         *
         * @tparam Constraint The constraint type the source `PhysicsWorld` uses.
         * @param world   The world whose `Transform`/`Orientation` columns are updated.
         * @param physics The physics world just stepped; read-only here.
         */
        template <typename Constraint>
        void sync_transforms_from_physics(World& world,
                                          const Physics::PhysicsWorld<Constraint>& physics)
        {
            const Signature required = make_signature<PhysicsBody, Transform, Orientation>();
            for (Archetype* archetype : world.query(required))
            {
                for (const std::unique_ptr<Chunk>& chunk_ptr : archetype->chunks())
                {
                    Chunk& chunk = *chunk_ptr;
                    const auto* bodies = reinterpret_cast<const PhysicsBody*>(
                        chunk.column(component_id<PhysicsBody>()));
                    auto* transforms =
                        reinterpret_cast<Transform*>(chunk.column(component_id<Transform>()));
                    auto* orientations = reinterpret_cast<Orientation*>(
                        chunk.column(component_id<Orientation>()));

                    for (std::size_t row = 0; row < chunk.count(); ++row)
                    {
                        const PhysicsBody& linked = bodies[row];
                        if (linked.body_id == PhysicsBody::INVALID)
                            continue;
                        const Physics::RigidBody& solved = physics.body(linked.body_id);
                        transforms[row].position = solved.position;
                        orientations[row].rotation = solved.orientation;
                    }
                }
            }
        }
    } // namespace Simulation
} // namespace SushiEngine
