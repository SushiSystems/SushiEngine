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
#include <SushiEngine/sim/physics_services.hpp>
#include <SushiEngine/sim/simulation.hpp>

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

            bool has_physics_body = false;
            PhysicsBodyParams physics_params;

            bool has_collider = false;
            ColliderParams collider_params;

            bool has_shape = false;
            ShapeParams shape_params;
        };

        /**
         * @brief The radius a body collides as, when it collides as a sphere.
         *
         * Taken from the entity's collider if it has one, else its visual shape, else
         * a unit default. A box or cylinder reports its *smallest* half-extent, so it
         * rests on the ground at about the right height instead of hovering by its
         * bounding radius — a deliberate under-approximation, and the reason a
         * cylinder currently behaves like a small sphere rather than a cylinder.
         *
         * @param entity The entity to measure.
         * @return Its collision radius.
         */
        inline Scalar collision_radius(const PhysicsSourceEntity& entity) noexcept
        {
            const auto radius_of = [](PrimitiveKind kind, const Vector3& p) -> Scalar
            {
                switch (kind)
                {
                    case PrimitiveKind::Sphere:
                        return p.x;
                    case PrimitiveKind::Cylinder:
                        return p.x;
                    case PrimitiveKind::Box:
                    {
                        const Scalar xy = p.x < p.y ? p.x : p.y;
                        return xy < p.z ? xy : p.z;
                    }
                    case PrimitiveKind::Plane:
                        return Scalar(0.25);
                }
                return Scalar(0.5);
            };
            if (entity.has_collider)
                return radius_of(entity.collider_params.kind, entity.collider_params.params);
            if (entity.has_shape)
                return radius_of(entity.shape_params.kind, entity.shape_params.params);
            return Scalar(0.5);
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
         * That fallback is why a cylinder collider silently simulates as a sphere,
         * and it stays visible here rather than buried in a world rebuild.
         *
         * @param entities The scene's entities, in the order bodies should be added.
         * @return One descriptor per rigid-body entity, in that same order.
         */
        inline std::vector<RigidBodyDesc> extract_rigid_bodies(
            const std::vector<PhysicsSourceEntity>& entities)
        {
            std::vector<RigidBodyDesc> descs;
            descs.reserve(entities.size());
            for (const PhysicsSourceEntity& entity : entities)
            {
                if (!entity.has_physics_body)
                    continue;
                RigidBodyDesc desc;
                desc.id = entity.id;
                desc.position = entity.local_position;
                desc.orientation = entity.local_orientation;
                desc.inv_mass = entity.physics_params.inv_mass;
                desc.inv_inertia = entity.physics_params.inv_inertia;
                desc.drag_coefficient = entity.physics_params.drag_coefficient;
                desc.radius = collision_radius(entity);
                if (entity.has_collider)
                {
                    desc.box = entity.collider_params.kind == PrimitiveKind::Box;
                    desc.half_extents = entity.collider_params.params;
                }
                else if (entity.has_shape)
                {
                    desc.box = entity.shape_params.kind == PrimitiveKind::Box;
                    desc.half_extents = entity.shape_params.params;
                }
                descs.push_back(desc);
            }
            return descs;
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
        inline std::vector<PlaneDesc> extract_static_planes(
            const std::vector<PhysicsSourceEntity>& entities)
        {
            std::vector<PlaneDesc> planes;
            for (const PhysicsSourceEntity& entity : entities)
            {
                if (!entity.has_collider || entity.has_physics_body ||
                    entity.collider_params.kind != PrimitiveKind::Plane)
                    continue;
                PlaneDesc plane;
                plane.point = entity.world_position;
                plane.normal = rotate(entity.world_orientation, entity.collider_params.params);
                planes.push_back(plane);
            }
            return planes;
        }
    } // namespace Simulation
} // namespace SushiEngine
