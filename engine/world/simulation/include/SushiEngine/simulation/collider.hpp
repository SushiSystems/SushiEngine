/**************************************************************************/
/* collider.hpp                                                           */
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
 * @file collider.hpp
 * @brief What a body collides as — including, at last, how big the author made it.
 *
 * §5.5's `Collider` record, and the answer to §1.2 item 5. `Transform::scale` has
 * been a render concept: scaling a crate in the editor made the *drawing* twice
 * the size and left the physics colliding as the authored half-extents, so the
 * crate rested with its feet buried and its lid in the air. Nothing about that was
 * subtle to see and nothing about it was findable in the physics, because the
 * physics was never told.
 *
 * Three things live here and they are one idea — the collider is *derived*, and
 * every step of the derivation is a function that can be tested without a world:
 *
 * 1. @ref collider_from_parameters turns the authoring component into the record.
 * 2. @ref scaled_collider applies the entity's scale to it.
 * 3. @ref collider_mass_properties derives mass and inertia from the *scaled*
 *    shape and a density, which is what P0 carry-over 2 asks for:
 *    `mass_properties.hpp` is only usable once there is a scaled shape to hand it.
 *
 * **On `ColliderParameters`.** §5.5 says this record supersedes it, and it does for
 * everything downstream: the extract reads `Collider`, and the physics never sees
 * `ColliderParameters` again. What `ColliderParameters` remains is the *authoring* surface
 * — the two fields the editor's inspector and the scene file write — and it stays
 * that way until the cooked `CollisionAsset` it is meant to be able to name
 * actually exists, which is P4. @ref Collider already carries the asset
 * identifier, so that day is a field being filled rather than a record being
 * reshaped.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/narrowphase_dispatch.hpp>
#include <SushiEngine/physics/core/body_flags.hpp>
#include <SushiEngine/physics/core/material.hpp>
#include <SushiEngine/physics/geometry/mass_properties.hpp>
#include <SushiEngine/simulation/simulation.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief Identifies a cooked collision asset; zero is "none" (P4, §5.4). */
        using CollisionAssetId = std::uint32_t;

        /** @brief The absent asset. */
        constexpr CollisionAssetId NULL_COLLISION_ASSET = 0;

        /** @brief What kind of shape a collider is. */
        enum class ColliderShape : std::uint32_t
        {
            Sphere = 0,
            Box,
            Capsule,
            Plane,
            /** @brief A cooked convex hull or mesh, named by @ref Collider::asset. */
            CookedAsset,
        };

        /**
         * @brief One body's collision shape, its filter, and what it weighs.
         *
         * Deliberately shaped like `Physics::CollisionShape` rather than like the
         * authoring component: this is the record that crosses into the physics, so
         * it carries the union of what the shapes need and lets the unused fields
         * be unused. A record per shape kind would be tidier to look at and would
         * mean a variant, a visitor, and a new case in the extract for every shape
         * P4 adds.
         */
        struct Collider
        {
            ColliderShape shape = ColliderShape::Box;

            /** @brief Box half-extents; also the plane's normal for @ref ColliderShape::Plane. */
            Vector3 half_extents{Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)}};

            /** @brief Sphere and capsule radius. */
            Scalar radius = Scalar(0.5);

            /** @brief Capsule half-segment, excluding the caps. */
            Scalar half_height = Scalar(0.5);

            /** @brief The cooked asset, when @ref shape is @ref ColliderShape::CookedAsset. */
            CollisionAssetId asset = NULL_COLLISION_ASSET;

            /** @brief Where the shape sits relative to the body's origin. */
            Vector3 local_offset;

            /** @brief The scale already applied to this record; carried for the cooker. */
            Vector3 applied_scale{Vector3{1, 1, 1}};

            Physics::CollisionFilter filter{};

            /** @brief `Physics::BodyFlags` bits — trigger, continuous collision, and so on. */
            std::uint32_t flags = 0;
        };

        /**
         * @brief The authoring component, as a collider.
         *
         * The one place `PrimitiveKind`'s meaning for physics is decided, and the
         * place each approximation is stated rather than left silent: a `Cylinder`
         * becomes a **capsule**, not a sphere. That is still an
         * approximation — a capsule has round ends and a barrel does not — but it
         * is one that rolls about the right axis and stands up on the ground, which
         * a sphere of the cylinder's radius does neither of (§1.2 item 4). A true
         * cylinder shape is a support function away whenever it is wanted.
         *
         * @param parameters The authored collider component.
         * @return The unscaled collider it means.
         */
        inline Physics::MaterialCombineMode to_combine_mode(std::uint32_t value) noexcept
        {
            // Clamped rather than trusted: the value crosses a scene file, and a mode past
            // the end would index `combine_coefficient`'s switch into its default anyway —
            // saying so here means one place knows the enumeration's extent instead of every
            // reader having to.
            return value <= std::uint32_t(Physics::MaterialCombineMode::maximum)
                       ? static_cast<Physics::MaterialCombineMode>(value)
                       : Physics::MaterialCombineMode::average;
        }

        inline Collider collider_from_parameters(const ColliderParameters& parameters) noexcept
        {
            Collider collider;
            // A body is in exactly one layer, so the authored index becomes the one-bit mask
            // `CollisionFilter::layer` means. Done here rather than at each call site because
            // an unshifted index would collide with layer 0 and nothing else, silently.
            collider.filter.layer = std::uint32_t(1) << (parameters.layer & 31u);
            collider.filter.collides_with = parameters.collides_with;
            collider.flags =
                (parameters.trigger ? Physics::BodyFlags::trigger : 0u) |
                (parameters.continuous_collision ? Physics::BodyFlags::continuous_collision : 0u);
            switch (parameters.kind)
            {
                case PrimitiveKind::Sphere:
                    collider.shape = ColliderShape::Sphere;
                    collider.radius = parameters.parameters.x;
                    break;
                case PrimitiveKind::Box:
                    collider.shape = ColliderShape::Box;
                    collider.half_extents = parameters.parameters;
                    break;
                case PrimitiveKind::Cylinder:
                {
                    collider.shape = ColliderShape::Capsule;
                    collider.radius = parameters.parameters.x;
                    // The authored half-height includes the caps; the capsule's does
                    // not, so a short, fat cylinder degenerates to a sphere rather
                    // than growing longer than it was drawn.
                    const Scalar half = parameters.parameters.y - parameters.parameters.x;
                    collider.half_height = half > Scalar(0) ? half : Scalar(0);
                    break;
                }
                case PrimitiveKind::Plane:
                    collider.shape = ColliderShape::Plane;
                    collider.half_extents = parameters.parameters; // the plane's local normal
                    break;
            }
            return collider;
        }

        /**
         * @brief The same collider at an entity's scale.
         *
         * Per shape, because scaling is not one operation:
         *
         * - A **box** scales per axis, exactly.
         * - A **sphere** takes the largest axis. A non-uniformly scaled sphere is an
         *   ellipsoid, which is not a shape the narrowphase has; the choice is
         *   between the largest axis and the smallest, and the largest is the one
         *   that keeps the visible mesh inside the collider rather than poking out
         *   of it. The cooker's hull path is the real answer and arrives with P4.
         * - A **capsule** scales its radius by the larger of the two cross-axes and
         *   its half-segment along Y, for the same reason.
         * - A **plane** does not scale at all: a half-space has no size, and its
         *   normal is a direction. (Under a non-uniform scale a normal transforms by
         *   the inverse transpose, not by the scale — but an authored plane normal
         *   is a *direction the author chose*, not a normal derived from geometry,
         *   so applying anything to it would be inventing a rotation they did not
         *   ask for.)
         * - A **cooked asset** records the scale and is scaled at instancing, since
         *   the vertices are shared by every instance and must not be rewritten.
         *
         * Negative scale is taken by magnitude throughout: a mirrored entity
         * collides as its mirror image, which for every shape here is itself.
         *
         * @param collider The unscaled collider.
         * @param scale    The entity's per-axis scale.
         * @return The scaled collider.
         */
        inline Collider scaled_collider(const Collider& collider, const Vector3& scale) noexcept
        {
            const Scalar x = std::abs(scale.x);
            const Scalar y = std::abs(scale.y);
            const Scalar z = std::abs(scale.z);

            Collider scaled = collider;
            scaled.applied_scale = Vector3{x, y, z};
            scaled.local_offset =
                Vector3{collider.local_offset.x * x, collider.local_offset.y * y,
                        collider.local_offset.z * z};

            switch (collider.shape)
            {
                case ColliderShape::Sphere:
                    scaled.radius = collider.radius * (x > y ? (x > z ? x : z) : (y > z ? y : z));
                    break;
                case ColliderShape::Box:
                    scaled.half_extents = Vector3{collider.half_extents.x * x,
                                                  collider.half_extents.y * y,
                                                  collider.half_extents.z * z};
                    break;
                case ColliderShape::Capsule:
                    scaled.radius = collider.radius * (x > z ? x : z);
                    scaled.half_height = collider.half_height * y;
                    break;
                case ColliderShape::Plane:
                case ColliderShape::CookedAsset:
                    break;
            }
            return scaled;
        }

        /**
         * @brief The radius a collider collides as when it collides as a sphere.
         *
         * The old `collision_radius`, kept because the contact path still has a
         * sphere fallback, but now measured off the *scaled* collider. A box reports
         * its smallest half-extent, which under-approximates deliberately: it rests
         * at about the right height instead of hovering by its bounding radius.
         */
        inline Scalar collider_sphere_radius(const Collider& collider) noexcept
        {
            switch (collider.shape)
            {
                case ColliderShape::Sphere:
                    return collider.radius;
                case ColliderShape::Capsule:
                    return collider.radius;
                case ColliderShape::Box:
                {
                    const Scalar xy = collider.half_extents.x < collider.half_extents.y
                                          ? collider.half_extents.x
                                          : collider.half_extents.y;
                    return xy < collider.half_extents.z ? xy : collider.half_extents.z;
                }
                case ColliderShape::Plane:
                    return Scalar(0.25);
                case ColliderShape::CookedAsset:
                    break;
            }
            return Scalar(0.5);
        }

        /**
         * @brief Mass and inertia from a scaled collider and a density.
         *
         * P0 carry-over 2. The load-bearing word is *scaled*: deriving mass
         * from the authored shape would give a doubled crate the mass of a single
         * one, which is a worse failure than a hand-authored number, because it
         * looks derived.
         *
         * A cooked asset reports nothing: its mass properties are integrated over
         * its faces by the cooker and travel in the asset (§8.4). A plane reports
         * nothing either — a half-space has infinite mass by construction, and
         * zero here means exactly that everywhere else in the engine.
         *
         * @param collider The scaled collider.
         * @param density  Mass per unit volume; zero or negative reports no mass.
         * @return Its mass properties; a zero mass means "not derived, keep what was
         *         authored".
         */
        inline Physics::MassProperties<Scalar> collider_mass_properties(const Collider& collider,
                                                                        Scalar density) noexcept
        {
            if (density <= Scalar(0))
                return Physics::MassProperties<Scalar>{};
            switch (collider.shape)
            {
                case ColliderShape::Sphere:
                    return Physics::sphere_mass_properties(collider.radius, density);
                case ColliderShape::Box:
                    return Physics::box_mass_properties(collider.half_extents, density);
                case ColliderShape::Capsule:
                    return Physics::capsule_mass_properties(collider.radius, collider.half_height,
                                                            density);
                case ColliderShape::Plane:
                case ColliderShape::CookedAsset:
                    break;
            }
            return Physics::MassProperties<Scalar>{};
        }

        /**
         * @brief The collision shape this collider is, placed at a world pose.
         *
         * The one place the authoring record becomes something the narrowphase can
         * collide. It is a translation and not a decision: every field it reads has
         * already been resolved by @ref scaled_collider, and every shape it produces
         * is one the dispatch table has a routine for.
         *
         * A **cooked asset falls back to its half-extents as a box**, and that is a
         * placeholder with a date on it: the asset the identifier names does not
         * exist until P4 (§8.4), and colliding a body as its bounding box is wrong
         * in a way an author can see and correct, where colliding it as nothing at
         * all is wrong in a way they cannot.
         *
         * @tparam T The physics scalar type.
         * @param collider    The scaled collider.
         * @param position    The body's world position.
         * @param orientation The body's world orientation.
         * @return A shape the narrowphase dispatch can collide.
         */
        template <typename T>
        inline Physics::CollisionShape<T> collider_shape(
            const Collider& collider, const Vector3T<T>& position,
            const QuaternionT<T>& orientation) noexcept
        {
            const Vector3T<T> offset =
                rotate(orientation, Vector3T<T>{T(collider.local_offset.x),
                                                T(collider.local_offset.y),
                                                T(collider.local_offset.z)});
            const Vector3T<T> center = position + offset;
            const Vector3T<T> half{T(collider.half_extents.x), T(collider.half_extents.y),
                                   T(collider.half_extents.z)};

            switch (collider.shape)
            {
                case ColliderShape::Sphere:
                    return Physics::make_sphere_shape<T>(center, T(collider.radius));
                case ColliderShape::Capsule:
                    return Physics::make_capsule_shape<T>(center, T(collider.half_height),
                                                          T(collider.radius), orientation);
                case ColliderShape::Plane:
                {
                    // The record keeps a plane's normal in `half_extents` and its
                    // offset in `radius`; a plane has no pose of its own, so the
                    // body's is not applied to it.
                    const Vector3T<T> normal = normalize(half);
                    return Physics::make_plane_shape<T>(normal, T(collider.radius));
                }
                case ColliderShape::Box:
                case ColliderShape::CookedAsset:
                    break;
            }
            return Physics::make_box_shape<T>(center, half, orientation);
        }
    } // namespace Simulation
} // namespace SushiEngine
