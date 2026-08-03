/**************************************************************************/
/* physics_sample_scene.cpp                                               */
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

#include "physics_sample_scene.hpp"

#include <string>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            using Simulation::ColliderParameters;
            using Simulation::EntityId;
            using Simulation::NULL_ENTITY;
            using Simulation::EntityTransform;
            using Simulation::IWorldEditor;
            using Simulation::PhysicsBodyParameters;
            using Simulation::PhysicsJointParameters;

            /** @brief The height the ground sits at, so everything else is placed above it. */
            constexpr Scalar GROUND_Y = 0;

            /**
             * @brief Places @p id and gives it a size.
             *
             * The scale, not the collider's half-extents: a primitive created through
             * `create_box` already carries a Collider matching its Shape, and the extract
             * scales that collider by the entity's scale (§1.2 item 5). Sizing by scale
             * therefore keeps the drawing and the collision one statement, which is the
             * property that made scale worth honouring in the first place.
             */
            void place(IWorldEditor& world, EntityId id, const Vector3& position,
                       const Vector3& scale)
            {
                EntityTransform transform = world.transform(id);
                transform.position = position;
                transform.scale = scale;
                world.set_transform(id, transform);
            }

            /** @brief Gives @p id a rigid body of the authored density. */
            void make_dynamic(IWorldEditor& world, EntityId id, Scalar density)
            {
                PhysicsBodyParameters body;
                body.density = density;
                world.set_has_physics_body(id, true);
                world.set_physics_body_parameters(id, body);
            }

            /** @brief Gives @p id a surface, and optionally a layer other than the default. */
            void make_surface(IWorldEditor& world, EntityId id, Scalar static_friction,
                              Scalar dynamic_friction, Scalar restitution,
                              std::uint32_t layer = 0, std::uint32_t collides_with = 0xFFFFFFFFu)
            {
                ColliderParameters collider = world.collider_parameters(id);
                collider.static_friction = static_friction;
                collider.dynamic_friction = dynamic_friction;
                collider.restitution = restitution;
                collider.layer = layer;
                collider.collides_with = collides_with;
                world.set_collider_parameters(id, collider);
            }

            /** @brief The ground: a Plane collider with no body, which is what makes it static. */
            void build_ground(IWorldEditor& world)
            {
                const EntityId ground = world.create_terrain("Ground");
                place(world, ground, Vector3{0, GROUND_Y, 0}, Vector3{1, 1, 1});
            }

            /**
             * @brief Five boxes in a column, which settle and then fall asleep.
             *
             * §7.4's ten-crate-stack test, at a size somebody can watch. What it demonstrates
             * is not that a stack stands up — that is P1's test's job — but the *island*: five
             * bodies in contact are one island, they sleep together, and with the debug draw's
             * Islands and Sleeping categories on that is visible rather than inferred.
             */
            void build_stack(IWorldEditor& world)
            {
                for (int i = 0; i < 5; ++i)
                {
                    const EntityId box = world.create_box("Stack " + std::to_string(i));
                    place(world, box, Vector3{-4, GROUND_Y + Scalar(0.5) + Scalar(i) * Scalar(1.02), 0},
                          Vector3{1, 1, 1});
                    make_dynamic(world, box, Scalar(500));
                    make_surface(world, box, Scalar(0.6), Scalar(0.5), Scalar(0));
                }
            }

            /**
             * @brief A ramp with two boxes on it that differ only in their material.
             *
             * The one comparison that makes §5.3 legible. Both boxes are the same size, the
             * same density and on the same slope; one is ice and one is rubber, and the
             * *only* reason they end up in different places is the two numbers in their
             * Collider. An author who changes one of them and plays sees the difference
             * immediately, which is what a material preview can only approximate.
             */
            void build_material_comparison(IWorldEditor& world)
            {
                const EntityId ramp = world.create_box("Ramp");
                place(world, ramp, Vector3{4, GROUND_Y + 1, 0}, Vector3{6, 0.2, 4});
                EntityTransform ramp_transform = world.transform(ramp);
                // About 17 degrees: past ice's own sliding angle (roughly 3) and well below
                // rubber's (roughly 45), so the two boxes do visibly different things.
                ramp_transform.rotation =
                    Quaternion{0, 0, Scalar(0.1494), Scalar(0.9888)};
                world.set_transform(ramp, ramp_transform);
                make_surface(world, ramp, Scalar(0.6), Scalar(0.5), Scalar(0));

                const EntityId ice = world.create_box("Ice Block");
                place(world, ice, Vector3{3, GROUND_Y + 2.2, -1}, Vector3{0.6, 0.6, 0.6});
                make_dynamic(world, ice, Scalar(900));
                make_surface(world, ice, Scalar(0.05), Scalar(0.03), Scalar(0));

                const EntityId rubber = world.create_box("Rubber Block");
                place(world, rubber, Vector3{3, GROUND_Y + 2.2, 1}, Vector3{0.6, 0.6, 0.6});
                make_dynamic(world, rubber, Scalar(1100));
                make_surface(world, rubber, Scalar(1.0), Scalar(0.9), Scalar(0));

                const EntityId ball = world.create_sphere("Bouncing Ball");
                place(world, ball, Vector3{7, GROUND_Y + 6, 0}, Vector3{0.5, 0.5, 0.5});
                make_dynamic(world, ball, Scalar(1000));
                make_surface(world, ball, Scalar(0.4), Scalar(0.3), Scalar(0.75));
            }

            /**
             * @brief Two boxes on layers that do not see each other, dropped through each other.
             *
             * §7.7's filter, demonstrated the only way it can be: a pair that visibly does not
             * collide while colliding with everything else. Both still land on the ground,
             * which is the control — a filter bug that dropped *every* contact would look the
             * same as this without it.
             */
            void build_filter_pair(IWorldEditor& world)
            {
                const EntityId ghost_a = world.create_box("Ghost A");
                place(world, ghost_a, Vector3{0, GROUND_Y + 3, -6}, Vector3{1, 1, 1});
                make_dynamic(world, ghost_a, Scalar(400));
                // Layer 2 collides with everything except layer 3, and layer 3 the reverse.
                // Both directions, because the filter test requires both — a one-sided
                // exclusion does nothing at all.
                make_surface(world, ghost_a, Scalar(0.6), Scalar(0.5), Scalar(0), 2,
                             ~(std::uint32_t(1) << 3));

                const EntityId ghost_b = world.create_box("Ghost B");
                place(world, ghost_b, Vector3{0, GROUND_Y + 6, -6}, Vector3{1, 1, 1});
                make_dynamic(world, ghost_b, Scalar(400));
                make_surface(world, ghost_b, Scalar(0.6), Scalar(0.5), Scalar(0), 3,
                             ~(std::uint32_t(1) << 2));
            }

            /**
             * @brief P3's acceptance scene: a chassis with a door that swings, and tears off.
             *
             * *"The chassis-plus-hinged-door scene works end to end: the door swings within
             * its limits, carries load, reports its hinge force, and tears off above its break
             * threshold."* Every clause of that is here, and every one of them is authored
             * rather than compiled: the limits are the hinge's twist range, the load is what
             * the Assembly window's live list reads, and the threshold is a field.
             *
             * The chassis is pinned, so the door is the only thing that moves and the hinge
             * carries a load that is entirely the door's.
             */
            void build_door(IWorldEditor& world)
            {
                const EntityId chassis = world.create_box("Chassis");
                place(world, chassis, Vector3{-4, GROUND_Y + 3, 6}, Vector3{3, 1.2, 1.6});
                PhysicsBodyParameters pinned;
                pinned.inv_mass = 0;
                pinned.inv_inertia = Vector3{0, 0, 0};
                world.set_has_physics_body(chassis, true);
                world.set_physics_body_parameters(chassis, pinned);
                make_surface(world, chassis, Scalar(0.6), Scalar(0.5), Scalar(0), 4,
                             ~(std::uint32_t(1) << 4));

                const EntityId door = world.create_box("Door");
                place(world, door, Vector3{-3.2, GROUND_Y + 3, 7.4}, Vector3{1.2, 1.0, 0.1});
                make_dynamic(world, door, Scalar(400));
                // Same group as the chassis, which does not collide with itself — otherwise
                // the door's contact with the body it is bolted to fights the hinge holding
                // it there, and the assembly buzzes instead of hanging.
                make_surface(world, door, Scalar(0.6), Scalar(0.5), Scalar(0), 4,
                             ~(std::uint32_t(1) << 4));

                PhysicsJointParameters hinge;
                hinge.connected_body = chassis;
                hinge.joint.type = Simulation::JointType::Hinge;
                hinge.joint.anchor_a = Vector3{-0.6, 0, 0};
                hinge.joint.anchor_b = Vector3{0.2, 0, 1.4};
                hinge.joint.axis_a = Vector3{0, 1, 0};
                hinge.joint.axis_b = Vector3{0, 1, 0};
                hinge.joint.twist_limit =
                    Simulation::JointLimitDescription{Scalar(0), Scalar(1.7), Scalar(0), true};
                // High enough that the door's own weight does not tear it off, low enough
                // that a shove does — which is the range in which a break threshold is a
                // gameplay parameter rather than a switch.
                hinge.joint.break_force = Scalar(12000);
                world.set_has_joint(door, true);
                world.set_joint_parameters(door, hinge);
            }

            /**
             * @brief A three-link pendulum, which is what a joint chain looks like when it works.
             *
             * Ball joints rather than hinges, so it swings out of plane too: a chain that can
             * only move in one plane hides exactly the errors — a mis-set axis, a frame that
             * is not what the author thought — that a free one shows in the first second.
             */
            void build_pendulum(IWorldEditor& world)
            {
                EntityId previous = NULL_ENTITY;
                for (int i = 0; i < 4; ++i)
                {
                    const EntityId link = world.create_box("Pendulum " + std::to_string(i));
                    place(world, link, Vector3{8, GROUND_Y + 8 - Scalar(i) * Scalar(1.1), 6},
                          Vector3{0.3, 1.0, 0.3});
                    if (i == 0)
                    {
                        PhysicsBodyParameters anchor;
                        anchor.inv_mass = 0;
                        anchor.inv_inertia = Vector3{0, 0, 0};
                        world.set_has_physics_body(link, true);
                        world.set_physics_body_parameters(link, anchor);
                    }
                    else
                    {
                        make_dynamic(world, link, Scalar(700));
                        PhysicsJointParameters joint;
                        joint.connected_body = previous;
                        joint.joint.type = Simulation::JointType::Ball;
                        joint.joint.anchor_a = Vector3{0, 0.55, 0};
                        joint.joint.anchor_b = Vector3{0, -0.55, 0};
                        world.set_has_joint(link, true);
                        world.set_joint_parameters(link, joint);
                    }
                    // Every link is in its own group that excludes itself, so neighbouring
                    // links do not push each other apart against the joints holding them.
                    make_surface(world, link, Scalar(0.6), Scalar(0.5), Scalar(0), 5,
                                 ~(std::uint32_t(1) << 5));
                    previous = link;
                }
            }

            /** @brief A cloth sheet, so the non-rigid half of the solver is in the scene too. */
            void build_cloth(IWorldEditor& world)
            {
                const EntityId cloth = world.create_cloth("Cloth");
                EntityTransform transform = world.transform(cloth);
                transform.position = Vector3{-10, GROUND_Y + 7, 0};
                world.set_transform(cloth, transform);
            }
        } // namespace

        void build_physics_sample_scene(IWorldEditor& world)
        {
            // Replaced rather than added to: half a demonstration mixed into somebody's own
            // scene is worse than either of them.
            const std::vector<EntityId> existing = world.entities();
            for (const EntityId id : existing)
                world.destroy(id);

            build_ground(world);
            build_stack(world);
            build_material_comparison(world);
            build_filter_pair(world);
            build_door(world);
            build_pendulum(world);
            build_cloth(world);

            const EntityId key = world.create_light("Key Light");
            EntityTransform light_transform = world.transform(key);
            light_transform.position = Vector3{6, GROUND_Y + 12, 8};
            world.set_transform(key, light_transform);
        }
    } // namespace Editor
} // namespace SushiEngine
