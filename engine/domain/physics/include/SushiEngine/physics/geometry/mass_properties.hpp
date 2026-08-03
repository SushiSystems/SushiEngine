/**************************************************************************/
/* mass_properties.hpp                                                    */
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
 * @file mass_properties.hpp
 * @brief Mass and inertia derived from a shape and a density.
 *
 * Today an author types an inverse mass and a *diagonal inverse inertia tensor* by
 * hand. Almost nobody can do that correctly for anything but a sphere, and getting
 * it wrong produces a body that tumbles plausibly enough that the error is never
 * traced back — which is worse than a body that obviously misbehaves.
 *
 * These are the closed forms for the primitives, expressed about the shape's own
 * centre of mass and in its own local frame, which is exactly the convention
 * `RigidBodyT::inv_inertia` stores. A shape with an offset centre (a compound, a
 * cooked hull) shifts its tensor with @ref shift_inertia.
 *
 * Every function reports the *inertia*, not its inverse, and @ref to_inverse does
 * the one-way conversion. That split is deliberate: zero is a meaningful inverse
 * inertia (an axis a body cannot rotate about) and an infinite inertia is not a
 * number, so inverting at the end keeps the degenerate case in one place.
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A shape's mass, its centre of mass, and its diagonal inertia.
         *
         * `inertia` is the diagonal of the inertia tensor about @ref center_of_mass,
         * expressed in the shape's local frame — the frame in which the primitives
         * below are diagonal by symmetry.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct MassProperties
        {
            T mass = 0;
            Vector3T<T> center_of_mass;
            Vector3T<T> inertia;
        };

        /**
         * @brief Mass properties of a solid sphere of uniform density.
         *
         * @tparam T The scalar element type.
         * @param radius  The sphere's radius.
         * @param density Mass per unit volume.
         * @return The sphere's mass, centre (its own), and inertia 2/5 m r^2 per axis.
         */
        template <typename T>
        inline MassProperties<T> sphere_mass_properties(T radius, T density) noexcept
        {
            const T pi = T(3.14159265358979323846);
            const T volume = (T(4) / T(3)) * pi * radius * radius * radius;
            const T mass = volume * density;
            const T moment = (T(2) / T(5)) * mass * radius * radius;
            return MassProperties<T>{mass, Vector3T<T>{}, Vector3T<T>{moment, moment, moment}};
        }

        /**
         * @brief Mass properties of a solid box of uniform density.
         *
         * @tparam T The scalar element type.
         * @param half_extents The box's per-axis half-extents.
         * @param density      Mass per unit volume.
         * @return The box's mass, centre (its own), and inertia m/3 (b^2 + c^2) per axis.
         */
        template <typename T>
        inline MassProperties<T> box_mass_properties(const Vector3T<T>& half_extents,
                                                     T density) noexcept
        {
            const Vector3T<T> extents = half_extents * T(2);
            const T mass = extents.x * extents.y * extents.z * density;
            // I_x = m (y^2 + z^2) / 12 with full extents; expressed in half-extents
            // the twelfth becomes a third.
            const T factor = mass / T(3);
            return MassProperties<T>{
                mass, Vector3T<T>{},
                Vector3T<T>{
                    factor * (half_extents.y * half_extents.y + half_extents.z * half_extents.z),
                    factor * (half_extents.x * half_extents.x + half_extents.z * half_extents.z),
                    factor * (half_extents.x * half_extents.x + half_extents.y * half_extents.y)}};
        }

        /**
         * @brief Mass properties of a solid cylinder about its local Y axis.
         *
         * Included because `ColliderParameters` already authors a cylinder, and today the
         * extract collapses it to a sphere. Getting its inertia right is what makes a
         * barrel roll about its axis differently from how it topples.
         *
         * @tparam T The scalar element type.
         * @param radius      The cylinder's radius.
         * @param half_height Half its extent along the local Y axis.
         * @param density     Mass per unit volume.
         * @return The cylinder's mass, centre (its own), and diagonal inertia.
         */
        template <typename T>
        inline MassProperties<T> cylinder_mass_properties(T radius, T half_height,
                                                          T density) noexcept
        {
            const T pi = T(3.14159265358979323846);
            const T height = half_height * T(2);
            const T mass = pi * radius * radius * height * density;
            const T axial = T(0.5) * mass * radius * radius;
            const T radial =
                mass * (T(3) * radius * radius + height * height) / T(12);
            return MassProperties<T>{mass, Vector3T<T>{},
                                     Vector3T<T>{radial, axial, radial}};
        }

        /**
         * @brief Mass properties of a solid capsule about its local Y axis.
         *
         * A cylinder with a hemisphere on each end, integrated as exactly that: the
         * cylinder's own inertia plus each hemisphere's, shifted to the capsule's
         * centre by the parallel-axis theorem. The shift is where this gets got
         * wrong — a hemisphere's centre of mass sits three-eighths of a radius from
         * its flat face, not at the face — and the resulting cross term
         * (`3/4 · m · h · r` across the pair) is what makes a long capsule
         * appreciably harder to tumble than the sphere-plus-cylinder sum suggests.
         *
         * @tparam T The scalar element type.
         * @param radius      The capsule's radius.
         * @param half_height Half the *segment*, excluding the caps.
         * @param density     Mass per unit volume.
         * @return Its mass, centre (its own), and diagonal inertia.
         */
        template <typename T>
        inline MassProperties<T> capsule_mass_properties(T radius, T half_height,
                                                         T density) noexcept
        {
            const T pi = T(3.14159265358979323846);
            const T radius_squared = radius * radius;
            const T cylinder_mass = pi * radius_squared * (half_height * T(2)) * density;
            const T cap_mass = (T(2) / T(3)) * pi * radius_squared * radius * density;

            // Along the axis the caps are just hemispheres about their own centre.
            const T axial = T(0.5) * cylinder_mass * radius_squared +
                            T(2) * (T(2) / T(5)) * cap_mass * radius_squared;
            // Across it: the cylinder's rod term, and each hemisphere shifted out.
            const T radial =
                cylinder_mass * (radius_squared / T(4) +
                                 (half_height * T(2)) * (half_height * T(2)) / T(12)) +
                T(2) * cap_mass *
                    (T(0.4) * radius_squared + half_height * half_height +
                     T(0.375) * half_height * radius * T(2));

            return MassProperties<T>{cylinder_mass + T(2) * cap_mass, Vector3T<T>{},
                                     Vector3T<T>{radial, axial, radial}};
        }

        /**
         * @brief Moves a diagonal inertia from the centre of mass to a parallel axis.
         *
         * The parallel-axis theorem, restricted to the diagonal. A compound body's
         * children each report an inertia about their own centre; assembling them
         * means shifting each to the parent's centre before summing, and forgetting
         * the shift is the classic way a multi-part body ends up far too easy to
         * spin.
         *
         * @tparam T The scalar element type.
         * @param inertia The diagonal inertia about the shape's own centre of mass.
         * @param mass    The shape's mass.
         * @param offset  The vector from the new axis to the centre of mass.
         * @return The diagonal inertia about the shifted axis.
         */
        template <typename T>
        inline Vector3T<T> shift_inertia(const Vector3T<T>& inertia, T mass,
                                         const Vector3T<T>& offset) noexcept
        {
            return Vector3T<T>{
                inertia.x + mass * (offset.y * offset.y + offset.z * offset.z),
                inertia.y + mass * (offset.x * offset.x + offset.z * offset.z),
                inertia.z + mass * (offset.x * offset.x + offset.y * offset.y)};
        }

        /**
         * @brief The inverse of a mass, treating zero as "infinitely heavy".
         *
         * @tparam T The scalar element type.
         * @param mass The mass to invert.
         * @return `1 / mass`, or zero when @p mass is zero or negative.
         */
        template <typename T>
        inline T inverse_mass(T mass) noexcept
        {
            return mass > T(0) ? T(1) / mass : T(0);
        }

        /**
         * @brief The component-wise inverse of a diagonal inertia.
         *
         * A zero component stays zero, which is the encoding `RigidBodyT` already
         * uses for "cannot rotate about this axis".
         *
         * @tparam T The scalar element type.
         * @param inertia The diagonal inertia to invert.
         * @return The diagonal inverse inertia.
         */
        template <typename T>
        inline Vector3T<T> to_inverse(const Vector3T<T>& inertia) noexcept
        {
            return Vector3T<T>{inertia.x > T(0) ? T(1) / inertia.x : T(0),
                               inertia.y > T(0) ? T(1) / inertia.y : T(0),
                               inertia.z > T(0) ? T(1) / inertia.z : T(0)};
        }

        /**
         * @brief Mass properties of a sphere collider at a density.
         * @tparam T The scalar element type.
         * @param sphere  The collider.
         * @param density Mass per unit volume.
         * @return Its mass properties, with the centre at the collider's centre.
         */
        template <typename T>
        inline MassProperties<T> mass_properties_of(const SphereCollider<T>& sphere,
                                                    T density) noexcept
        {
            MassProperties<T> properties = sphere_mass_properties(sphere.radius, density);
            properties.center_of_mass = sphere.center;
            return properties;
        }

        /**
         * @brief Mass properties of a box collider at a density.
         * @tparam T The scalar element type.
         * @param box     The collider.
         * @param density Mass per unit volume.
         * @return Its mass properties, with the centre at the collider's centre.
         */
        template <typename T>
        inline MassProperties<T> mass_properties_of(const BoxCollider<T>& box,
                                                    T density) noexcept
        {
            MassProperties<T> properties = box_mass_properties(box.half_extents, density);
            properties.center_of_mass = box.center;
            return properties;
        }

        /**
         * @brief Mass properties of an oriented box at a density.
         *
         * The inertia is reported in the box's own local frame, unrotated: that is
         * the frame `RigidBodyT::inv_inertia` is expressed in, and the projection
         * already applies the body's orientation around it.
         *
         * @tparam T The scalar element type.
         * @param box     The collider.
         * @param density Mass per unit volume.
         * @return Its mass properties, with the centre at the collider's centre.
         */
        template <typename T>
        inline MassProperties<T> mass_properties_of(const OrientedBox<T>& box,
                                                    T density) noexcept
        {
            MassProperties<T> properties = box_mass_properties(box.half_extents, density);
            properties.center_of_mass = box.center;
            return properties;
        }

        /**
         * @brief Mass properties of a capsule collider at a density.
         *
         * Completes the overload set for the shapes P2 added — which is the third
         * of §4.2's three obligations for a new shape, and the one most easily
         * forgotten, because a shape with no mass function still *collides* and
         * only misbehaves once something spins it.
         *
         * A cooked convex hull has no overload here on purpose: its mass properties
         * are integrated over its faces, the cooker is what produces faces, and so
         * they arrive in the asset itself rather than being recomputed per instance
         * at load (§5.4, §8.4).
         *
         * @tparam T The scalar element type.
         * @param capsule The collider.
         * @param density Mass per unit volume.
         * @return Its mass properties, with the centre at the collider's centre.
         */
        template <typename T>
        inline MassProperties<T> mass_properties_of(const CapsuleCollider<T>& capsule,
                                                    T density) noexcept
        {
            MassProperties<T> properties =
                capsule_mass_properties(capsule.radius, capsule.half_height, density);
            properties.center_of_mass = capsule.center;
            return properties;
        }
    } // namespace Physics
} // namespace SushiEngine
