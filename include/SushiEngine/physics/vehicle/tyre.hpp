/**************************************************************************/
/* tyre.hpp                                                               */
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
 * @file tyre.hpp
 * @brief §11.5's force model: what a contact patch does, as arithmetic.
 *
 * *"A slip-based force model — a Pacejka-style magic-formula curve, or a simpler Brush
 * model as the first implementation — evaluated per wheel: compute longitudinal and
 * lateral slip from the wheel's contact patch velocity, look up the force curve, apply
 * the force at the contact point. Combined-slip handling by the friction ellipse. Load
 * sensitivity from the contact normal force the solver already recovered."* This file is
 * the middle of that sentence and nothing else: slip and load in, force out. Where the
 * patch is, what is under it and who takes the reaction are @ref tyre_projection.hpp's.
 *
 * ### Why the brush model and not the magic formula
 *
 * Pacejka's formula is a *fit*. Its coefficients have no physical meaning on their own,
 * they are measured on a rig from a real tyre, and an author who has not measured one is
 * left tuning fourteen numbers that interact. The brush model is a *derivation*: bristles
 * on the tread deflect until the local shear reaches the friction limit and then slide,
 * and the whole curve falls out of two stiffnesses and a friction coefficient. Every
 * number in @ref TyreSettingsT is therefore something an author can reason about, and
 * the shape — a linear rise, a rounded peak, a fall into sliding — is the shape of a real
 * tyre because it comes from the same argument, not because someone matched it.
 *
 * The magic formula stays reachable: it would be a second @ref tyre_force with the same
 * signature, and nothing above here would change. §11.5's node-beam pressurized tyre is
 * the other alternative, and it is a soft body rather than a force model.
 *
 * ### The friction circle is not bolted on
 *
 * Longitudinal and lateral slip are one two-dimensional slip vector, saturated once as a
 * vector. Combined slip is then automatic and cannot be got wrong — a tyre already at its
 * limit under braking has nothing left to steer with, which is what understeer under
 * braking *is*, and it falls out rather than being detected. Computing the two axes
 * separately and clipping afterwards is the classic mistake: it lets a wheel produce
 * `μN` sideways *and* `μN` forwards, which is 1.41 times the friction the surface has.
 */

#include <cmath>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief One tyre, as an author states it.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct TyreSettingsT
        {
            /**
             * @brief Peak friction coefficient at @ref rated_load, dimensionless.
             *
             * At or below zero the tyre generates nothing, which is the off switch: a
             * corner authored without a tyre is a castor that the solver's own friction
             * still handles, and no extra flag is needed to say so.
             */
            T friction = T(1.2);

            /**
             * @brief Longitudinal slip stiffness, per unit of normal load.
             *
             * Dimensionless: the force per unit slip ratio is this times the load. Real
             * tyres stiffen roughly in proportion to what they carry, so quoting the
             * stiffness per unit load is both closer to the truth and one fewer number to
             * keep consistent with @ref rated_load.
             */
            T longitudinal_stiffness = T(15);

            /** @brief Cornering stiffness per unit of normal load, per radian of slip. */
            T lateral_stiffness = T(12);

            /**
             * @brief The load @ref friction is quoted at, in newtons.
             *
             * A quarter of a laden car is the sensible value, because that is the load the
             * tyre spends its life at and the load an author has a feel for.
             */
            T rated_load = T(4000);

            /**
             * @brief How much grip is lost as load rises above @ref rated_load.
             *
             * A real tyre's friction coefficient *falls* with load, which is why a car
             * transfers weight to the end it wants to grip less and why a heavier car
             * corners worse than its extra tyre load suggests. Zero switches the effect
             * off and makes the tyre a textbook Coulomb surface.
             */
            T load_sensitivity = T(0.15);

            /**
             * @brief The speed below which slip is measured against speed, in metres per
             *        second.
             *
             * Slip ratio divides by the hub's forward speed and is therefore undefined at
             * rest, which is not an edge case — it is *parked*, the state a car spends
             * most of its life in. Below this speed the divisor is held here, which turns
             * the model linear in velocity: the tyre becomes a damper, a stationary car
             * sits still instead of buzzing, and nothing has to special-case standstill.
             */
            T low_speed_reference = T(2);
        };

        /**
         * @brief How far a patch is from rolling, in the two directions it can slide.
         *
         * Both are signed so that a positive value asks for a positive force along the
         * same axis: the model's whole job is to oppose slip, and folding the sign in here
         * means no reader downstream has to remember which way round it goes.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct TyreSlipT
        {
            /** @brief Longitudinal slip ratio: tread speed over hub speed, less one. */
            T longitudinal = T(0);

            /** @brief Lateral slip, as the tangent of the slip angle. */
            T lateral = T(0);
        };

        /**
         * @brief What a patch produced, in its own two directions.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct TyreForceT
        {
            /** @brief Force along the rolling direction, in newtons. */
            T longitudinal = T(0);

            /** @brief Force along the axle, in newtons. */
            T lateral = T(0);

            /**
             * @brief How much of the available grip is spent, from zero to one.
             *
             * One is a patch entirely sliding. The readout a traction control, a skid
             * sound or a diagnostic overlay wants, and the only way to tell a tyre at its
             * limit from one merely working hard — the forces alone cannot, because both
             * are large.
             */
            T saturation = T(0);
        };

        /**
         * @brief The friction coefficient at a given load.
         *
         * @tparam T The scalar element type.
         * @param settings The tyre.
         * @param load     Normal force on the patch, in newtons.
         */
        template <typename T>
        inline T tyre_friction(const TyreSettingsT<T>& settings, T load) noexcept
        {
            if (!(settings.friction > T(0)) || !(load > T(0)))
                return T(0);
            if (!(settings.rated_load > T(0)) || !(settings.load_sensitivity > T(0)))
                return settings.friction;

            const T excess = load / settings.rated_load - T(1);
            const T scaled = settings.friction * (T(1) - settings.load_sensitivity * excess);
            // A tyre that lost all its grip to load would let a heavy car slide with no
            // warning at all; the floor keeps the fall continuous instead of terminal.
            const T floor = settings.friction * T(0.1);
            return scaled < floor ? floor : scaled;
        }

        /**
         * @brief Slip from the patch's own velocities.
         *
         * @tparam T The scalar element type.
         * @param settings         The tyre, for @ref TyreSettingsT::low_speed_reference.
         * @param patch_forward    Patch slip speed along the rolling direction — for a
         *                         wheel that is rolling truly this is zero, because the
         *                         hub's motion and the tread's cancel there.
         * @param patch_lateral    Patch slip speed along the axle.
         * @param hub_forward      The hub's own speed along the rolling direction, which
         *                         is what slip is measured *against*.
         */
        template <typename T>
        inline TyreSlipT<T> tyre_slip(const TyreSettingsT<T>& settings, T patch_forward,
                                      T patch_lateral, T hub_forward) noexcept
        {
            const T speed = hub_forward < T(0) ? -hub_forward : hub_forward;
            const T reference =
                speed > settings.low_speed_reference ? speed : settings.low_speed_reference;
            if (!(reference > T(0)))
                return TyreSlipT<T>{};

            TyreSlipT<T> slip;
            slip.longitudinal = -patch_forward / reference;
            slip.lateral = -patch_lateral / reference;
            return slip;
        }

        /**
         * @brief The brush model: slip and load in, force out.
         *
         * The bristles deflect linearly until the local shear reaches `μN`, and past that
         * the trailing part of the patch slides. Integrating that over a parabolic
         * pressure distribution gives the classic cubic, in which `θ` is how far the
         * linear force has run toward three times the friction limit:
         *
         * `F = |g|·(1 − θ + θ²/3)` for `θ < 1`, and `μN` beyond it, with `θ = |g|/3μN`.
         *
         * It reaches exactly `μN` at `θ = 1` and its slope there is exactly zero — the
         * derivative is `(1 − θ)²` — so the curve arrives at the peak smoothly instead of
         * hitting a corner. That matters more than it sounds: a kink at the limit is a
         * discontinuity a car crosses several times a second under hard driving, and it is
         * felt as a tyre that grips and lets go rather than one that gives way.
         *
         * Saturation is measured on the *unsaturated* magnitude, because once the force
         * has been clipped every sliding tyre reports the same number.
         *
         * @tparam T The scalar element type.
         * @param settings The tyre.
         * @param slip     How far the patch is from rolling.
         * @param load     Normal force on the patch, in newtons.
         */
        template <typename T>
        inline TyreForceT<T> tyre_force(const TyreSettingsT<T>& settings,
                                        const TyreSlipT<T>& slip, T load) noexcept
        {
            TyreForceT<T> force;
            const T limit = tyre_friction(settings, load) * load;
            if (!(limit > T(0)))
                return force;

            const T reach_x = settings.longitudinal_stiffness * load * slip.longitudinal;
            const T reach_y = settings.lateral_stiffness * load * slip.lateral;
            const T reach = std::sqrt(reach_x * reach_x + reach_y * reach_y);
            if (!(reach > T(0)))
                return force;

            const T span = T(3) * limit;
            const T theta = reach / span;
            const T magnitude =
                theta < T(1) ? reach * (T(1) - theta + theta * theta / T(3)) : limit;

            const T scale = magnitude / reach;
            force.longitudinal = reach_x * scale;
            force.lateral = reach_y * scale;
            force.saturation = theta < T(1) ? theta : T(1);
            return force;
        }

        /** @brief The boundary tyre: @ref TyreSettingsT fixed to `Scalar`. */
        using TyreSettings = TyreSettingsT<Scalar>;

        /** @brief The boundary slip pair. */
        using TyreSlip = TyreSlipT<Scalar>;

        /** @brief The boundary patch force. */
        using TyreForce = TyreForceT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
