/**************************************************************************/
/* physics_extract.hpp                                                    */
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
 * @file physics_extract.hpp
 * @brief Turning authored entity records into physics descriptors, and only that.
 *
 * `RuntimeSimulation` used to gather these inline, three private methods deep inside
 * a three-thousand-line file. Translating authored components into simulation input
 * is a responsibility in its own right — the renderer's extract has been its own
 * tested unit for exactly this reason — and it was untestable where it sat, because
 * reaching it meant standing up a whole live world.
 *
 * The functions here take a flat list of @ref PhysicsSourceEntity and return
 * descriptors. They touch no ECS, no world, and no physics: a test hands them three
 * structs and checks three structs, which is what makes the awkward cases (a
 * Cylinder collapsing to a sphere, a plane that is also a rigid body) checkable at
 * all.
 */

#include <cstddef>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/simulation/physics_services.hpp>
#include <SushiEngine/simulation/simulation.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief One entity, as much of it as the physics extract needs.
         *
         * Deliberately flat and deliberately pre-resolved: the caller has already
         * decided what is alive and already walked the transform hierarchy, so this
         * unit has no way to disagree with the live world about either.
         *
         * The two transforms are both here because the extract genuinely uses both,
         * and always has: a rigid body seeds from its entity's *local* transform,
         * while a static plane is placed by its *world* transform. That asymmetry
         * predates this refactor and is preserved rather than quietly fixed — a
         * parented rigid body would change behaviour if it were, and that is a
         * decision to take deliberately, not as a side effect of moving code.
         */
        struct PhysicsSourceEntity
        {
            EntityId id = NULL_ENTITY;

            /** @brief The entity's own transform, unparented. */
            Vector3 local_position;
            Quaternion local_orientation;

            /** @brief The entity's transform after the hierarchy is applied. */
            Vector3 world_position;
            Quaternion world_orientation;

            /**
             * @brief The entity's own scale, and its scale after the hierarchy.
             *
             * Here for the same reason the two transforms are: a rigid body seeds
             * from its own transform and a static plane is placed by its world one,
             * and scale follows whichever of those the shape follows. Until now the
             * physics was never handed either, which is §1.2 item 5.
             */
            Vector3 local_scale{Vector3{1, 1, 1}};
            Vector3 world_scale{Vector3{1, 1, 1}};

            bool has_physics_body = false;
            PhysicsBodyParameters physics_parameters;

            bool has_collider = false;
            ColliderParameters collider_parameters;

            bool has_shape = false;
            ShapeParameters shape_parameters;
        };

        /**
         * @brief The collider an entity collides as, at its scale.
         *
         * Its collider if it has one, else its visual shape, else a unit sphere —
         * the same precedence as before, now producing a `Collider` rather than a
         * bare radius, and now with the entity's scale applied. That last clause is
         * the whole of §1.2 item 5: doubling a crate in the editor used to double
         * the drawing and leave the physics colliding as the authored half-extents.
         *
         * @param entity The entity to resolve.
         * @return Its scaled collider.
         */
        inline Collider resolve_collider(const PhysicsSourceEntity& entity) noexcept
        {
            Collider collider;
            if (entity.has_collider)
                collider = collider_from_parameters(entity.collider_parameters);
            else if (entity.has_shape)
                collider = collider_from_parameters(ColliderParameters{
                    entity.shape_parameters.kind, entity.shape_parameters.parameters});
            else
                collider.shape = ColliderShape::Sphere;
            return scaled_collider(collider, entity.local_scale);
        }

        /**
         * @brief The surface an entity contacts as (§5.3).
         *
         * Read off the Collider, because that is where the surface is: two crates of the
         * same mesh can be ice and rubber, and nothing about them is shared but the shape.
         * An entity with no Collider gets the defaults, which describe an ordinary solid —
         * the same precedence @ref resolve_collider follows for the shape itself.
         *
         * @param entity The entity to resolve.
         * @return Its surface material.
         */
        inline Physics::PhysicsMaterialT<Scalar> resolve_material(
            const PhysicsSourceEntity& entity) noexcept
        {
            Physics::PhysicsMaterialT<Scalar> material;
            if (!entity.has_collider)
                return material;
            material.static_friction = entity.collider_parameters.static_friction;
            material.dynamic_friction = entity.collider_parameters.dynamic_friction;
            material.restitution = entity.collider_parameters.restitution;
            material.friction_combine =
                to_combine_mode(entity.collider_parameters.friction_combine);
            material.restitution_combine =
                to_combine_mode(entity.collider_parameters.restitution_combine);
            return material;
        }

        /**
         * @brief The radius a body collides as, when it collides as a sphere.
         *
         * Kept as a named function because the contact path still has a sphere
         * fallback and because the awkward cases were worth having a test each; it
         * is now @ref resolve_collider followed by a measurement, so it can no
         * longer disagree with the shape the body actually collides as.
         *
         * @param entity The entity to measure.
         * @return Its collision radius.
         */
        inline Scalar collision_radius(const PhysicsSourceEntity& entity) noexcept
        {
            return collider_sphere_radius(resolve_collider(entity));
        }

        /**
         * @brief One rigid-body descriptor per entity that has a body.
         *
         * The pose only seeds a *newly* added body; a body the simulation already
         * tracks carries its live state over instead. Built fresh from the current
         * entity set, so a destroyed entity simply drops out.
         *
         * A box collider (or, absent a collider, a box visual) collides as an
         * oriented box; anything else falls back to a sphere of @ref collision_radius.
         * Both are now read off the *scaled* collider, so the three shape fields and
         * the collider cannot describe different objects.
         *
         * **Mass.** When the body authors a positive density, its inverse mass and
         * inverse inertia are derived from the scaled shape and that density, and
         * the hand-authored numbers are ignored. When it does not — the default —
         * the authored numbers are used exactly as before. A silent switch would be
         * worse than either: an author who typed an inverse mass and got a
         * different one has no way to tell what happened, so opting in is a field
         * they set. This is P0 carry-over 2.
         *
         * @param entities The scene's entities, in the order bodies should be added.
         * @return One descriptor per rigid-body entity, in that same order.
         */
        inline std::vector<RigidBodyDescription> extract_rigid_bodies(
            const std::vector<PhysicsSourceEntity>& entities)
        {
            std::vector<RigidBodyDescription> descriptions;
            descriptions.reserve(entities.size());
            for (const PhysicsSourceEntity& entity : entities)
            {
                if (!entity.has_physics_body)
                    continue;
                RigidBodyDescription description;
                description.id = entity.id;
                description.position = entity.local_position;
                description.orientation = entity.local_orientation;
                description.inv_mass = entity.physics_parameters.inv_mass;
                description.inv_inertia = entity.physics_parameters.inv_inertia;
                description.drag_coefficient = entity.physics_parameters.drag_coefficient;

                description.collider = resolve_collider(entity);
                description.material = resolve_material(entity);

                const Physics::MassProperties<Scalar> mass = collider_mass_properties(
                    description.collider, entity.physics_parameters.density);
                if (mass.mass > Scalar(0))
                {
                    description.inv_mass = Physics::inverse_mass(mass.mass);
                    description.inv_inertia = Physics::to_inverse(mass.inertia);
                }
                descriptions.push_back(description);
            }
            return descriptions;
        }

        /**
         * @brief One static half-space per plane collider that is not a rigid body.
         *
         * The collider's local normal is rotated into world space at the entity's
         * world position. An entity carrying both a plane collider and a rigid body
         * is skipped: a moving plane is not a static surface, and admitting it would
         * mean the surface and the body it belongs to disagree about where it is.
         *
         * @param entities The scene's entities.
         * @return One plane per qualifying entity, in the order given.
         */
        inline std::vector<PlaneDescription> extract_static_planes(
            const std::vector<PhysicsSourceEntity>& entities)
        {
            std::vector<PlaneDescription> planes;
            for (const PhysicsSourceEntity& entity : entities)
            {
                if (!entity.has_collider || entity.has_physics_body ||
                    entity.collider_parameters.kind != PrimitiveKind::Plane)
                    continue;
                PlaneDescription plane;
                plane.point = entity.world_position;
                plane.normal =
                    rotate(entity.world_orientation, entity.collider_parameters.parameters);
                planes.push_back(plane);
            }
            return planes;
        }
    } // namespace Simulation
} // namespace SushiEngine
