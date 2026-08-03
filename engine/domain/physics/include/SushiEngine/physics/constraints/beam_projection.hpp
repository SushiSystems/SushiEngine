/**************************************************************************/
/* beam_projection.hpp                                                    */
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
 * @file beam_projection.hpp
 * @brief How a beam corrects its two nodes, what load it reports, and when it dents.
 *
 * Three pieces, in the order a tick runs them:
 *
 * | When | What |
 * |---|---|
 * | Every substep, positional pass | @ref BeamProjectionT — the axial row, and the load recovery |
 * | Every substep, after `update_velocity` | @ref BeamVelocityProjectionT — axial damping |
 * | Once per tick | @ref apply_beam_plasticity — the permanent dent |
 *
 * Splitting them that way is not a style choice, it is where each one is *correct*.
 * The axial row is positional because XPBD's step-size independence is a property of
 * the positional formulation. Damping is a statement about a rate, and until the
 * substep's pose change has been read back as a velocity there is no rate to make it
 * about — the same argument `JointVelocityProjectionT` makes. And plasticity runs
 * once per tick because `plastic_creep` is read as *the fraction of the current
 * excess that becomes permanent per tick*, exactly as
 * @ref apply_fem_plasticity reads it; running it per substep would make a beam's
 * permanent set depend on the substep schedule, and the substep schedule is derived
 * from scene motion (§6.2). A dent that deepened because something else in the scene
 * sped up would be the least explicable behaviour in the system.
 *
 * ### The load, and why it is `-lambda / h²`
 *
 * §10.4's recovery: an XPBD row's Lagrange multiplier is an impulse along the row's
 * gradient, so `force = lambda / h²`. The sign follows from this projection's error
 * convention — `error = length - rest_length` is positive when the beam is stretched,
 * and the multiplier that corrects it is negative — so the reported load negates it
 * to make **tension positive**, which is what @ref BeamConstraintT::axial_force
 * promises and what an engineer expects a member's load to mean.
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/beam_constraint.hpp>
#include <SushiEngine/physics/core/body_flags.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Projects one beam's axial row and folds what it carried into its load.
         *
         * One XPBD row along the line between the two nodes, at the beam's compliance,
         * with the correction split by inverse mass. A captureless functor with the
         * mutable-descriptor signature `JointProjectionT` established, because a beam
         * has the same need: the thing the projection computes (a load) is the thing
         * the tick boundary reads to decide whether the beam is still there.
         *
         * The tick's load accounting is reset here rather than in a node of its own,
         * for the reason `JointProjectionT` gives — it is one branch on a value the
         * kernel has already loaded, against a whole graph node per colour per tick.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct BeamProjectionT
        {
            /**
             * @param beam          The beam; its load accumulators are updated in place.
             * @param bodies        The node array the slots index; corrected in place.
             * @param h             The substep duration, in seconds (> 0).
             * @param first_substep Whether this is the tick's first substep, which is
             *                      when the load accumulators are cleared.
             */
            void operator()(BeamConstraintT<T>& beam, RigidBodyT<T>* bodies, T h,
                            bool first_substep) const noexcept
            {
                RigidBodyT<T>& node_a = bodies[beam.a];
                RigidBodyT<T>& node_b = bodies[beam.b];

                // A beam whose pair is out of the solve keeps its last live load, for
                // the reason `JointProjectionT` states: a settled structure reporting
                // zero load is a wrong answer, and it is the answer a reader gets from
                // a sleeping island unless the last measurement survives.
                if (has_any_flag(node_a.flags | node_b.flags, BodyFlags::sleeping))
                    return;
                if (!is_simulated(node_a.flags) && !is_simulated(node_b.flags))
                    return;

                if (first_substep)
                {
                    beam.force_sum = T(0);
                    beam.peak_force = T(0);
                    beam.force_samples = 0;
                }
                if (!beam_is_active(beam.flags))
                    return;

                const Vector3T<T> delta = node_b.position - node_a.position;
                const T delta_length = length(delta);
                // Two nodes at the same point have no axis to correct along. Reported
                // as a sample carrying nothing rather than skipped, so a collapsed beam
                // does not read as one nobody stepped.
                if (!(delta_length > T(1e-8)))
                {
                    ++beam.force_samples;
                    return;
                }

                const Vector3T<T> axis = delta * (T(1) / delta_length);
                const T error = delta_length - beam.rest_length;

                const T w = node_a.inv_mass + node_b.inv_mass;
                if (!(w > T(0)))
                {
                    ++beam.force_samples;
                    return;
                }

                const T alpha_tilde = h > T(0) ? beam.compliance / (h * h) : T(0);
                // One iteration per substep, so the multiplier starts each substep at
                // zero and the accumulated-lambda term of the full XPBD update drops
                // out. Written as the closed form rather than as an update against a
                // stored zero, which is the same arithmetic with a load in front of it.
                const T lambda = -error / (w + alpha_tilde);

                const Vector3T<T> impulse = axis * lambda;
                node_a.position = node_a.position - impulse * node_a.inv_mass;
                node_b.position = node_b.position + impulse * node_b.inv_mass;

                const T force = h > T(0) ? -lambda / (h * h) : T(0);
                beam.axial_force = force;
                beam.force_sum += force;
                const T magnitude = force < T(0) ? -force : force;
                if (magnitude > beam.peak_force)
                    beam.peak_force = magnitude;
                ++beam.force_samples;
            }
        };

        /**
         * @brief Removes a fraction of a beam's relative axial velocity.
         *
         * After `update_velocity`, so there is a rate to damp. The fraction is
         * `min(1, damping * h)`, which makes @ref BeamConstraintT::damping a rate in
         * inverse seconds rather than a per-substep fraction — a beam damped at 20 s⁻¹
         * damps the same amount per second whatever the substep schedule derived, and
         * the clamp means an aggressively damped beam brings its nodes to a common
         * axial velocity rather than overshooting into a growing oscillation.
         *
         * Only the *axial* component is touched. A beam has no opinion about its nodes
         * sliding past each other sideways — that is the neighbouring beams' row to
         * project — and damping the full relative velocity would quietly make a beam
         * network resist shear it was never given the stiffness to resist.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct BeamVelocityProjectionT
        {
            /**
             * @param beam   The beam; read only, since damping carries no load the
             *               positional row has not already reported.
             * @param bodies The node array the slots index; velocities updated in place.
             * @param h      The substep duration, in seconds.
             */
            void operator()(const BeamConstraintT<T>& beam, RigidBodyT<T>* bodies,
                            T h) const noexcept
            {
                if (!beam_is_active(beam.flags) || !(beam.damping > T(0)))
                    return;

                RigidBodyT<T>& node_a = bodies[beam.a];
                RigidBodyT<T>& node_b = bodies[beam.b];
                if (has_any_flag(node_a.flags | node_b.flags, BodyFlags::sleeping))
                    return;

                const Vector3T<T> delta = node_b.position - node_a.position;
                const T delta_length = length(delta);
                if (!(delta_length > T(1e-8)))
                    return;
                const Vector3T<T> axis = delta * (T(1) / delta_length);

                const T w = node_a.inv_mass + node_b.inv_mass;
                if (!(w > T(0)))
                    return;

                T fraction = beam.damping * h;
                if (fraction > T(1))
                    fraction = T(1);

                const T relative = dot(node_b.velocity - node_a.velocity, axis);
                const Vector3T<T> impulse = axis * (-relative * fraction / w);
                node_a.velocity = node_a.velocity - impulse * node_a.inv_mass;
                node_b.velocity = node_b.velocity + impulse * node_b.inv_mass;
            }
        };

        /**
         * @brief Walks a beam's rest length toward its current length, once it has yielded.
         *
         * §11.1's `beamDeform`, and the axial case of §9.4's mechanism: past the
         * threshold, a fraction of the elastic deviation stops being elastic. The
         * fraction is clamped so the accumulator never overshoots
         * `maximum_plastic_strain`, which is what makes a repeatedly hit panel
         * work-harden and stop denting rather than creep away without limit.
         *
         * A no-op when the beam cannot yield, when it has already hardened, when it
         * never reached its threshold this tick, or when it has no cooked length to
         * measure strain against.
         *
         * Measured against @ref BeamConstraintT::peak_force rather than the mean, for
         * the same reason breakage is: the load that dents a panel is the impact, and
         * an impact's mean over a tick that also contains the rebound is near zero.
         *
         * Takes the two node positions rather than the node array because the caller
         * that runs this is the one holding the structure, and a structure's nodes are
         * solver *slots* it reads back one pair at a time — it has no array indexed by
         * `beam.a` to hand over. The array form below is the same function for a caller
         * that does.
         *
         * @tparam T The scalar element type.
         * @param position_a Where the beam's first node is.
         * @param position_b Where the beam's second node is.
         * @param beam       The beam; its rest length and accumulated strain are updated
         *                   in place. The yield *load* is its own
         *                   @ref BeamConstraintT::deform_force, because a material's
         *                   yield stress becomes a force only once a cross-section is
         *                   known.
         */
        template <typename T>
        inline void apply_beam_plasticity(const Vector3T<T>& position_a,
                                          const Vector3T<T>& position_b,
                                          BeamConstraintT<T>& beam) noexcept
        {
            if (!(beam.plastic_creep > T(0)) || !(beam.maximum_plastic_strain > T(0)))
                return;
            if (beam.accumulated_plastic_strain >= beam.maximum_plastic_strain)
                return;
            if (!(beam.deform_force > T(0)) || !(beam.peak_force > beam.deform_force))
                return;
            if (!(beam.initial_rest_length > T(0)) || beam.force_samples == 0)
                return;

            const T delta_length = length(position_b - position_a);
            const T deviation = delta_length - beam.rest_length;
            const T strain_magnitude =
                (deviation < T(0) ? -deviation : deviation) / beam.initial_rest_length;
            if (!(strain_magnitude > T(0)))
                return;

            T creep = beam.plastic_creep;
            const T remaining = beam.maximum_plastic_strain - beam.accumulated_plastic_strain;
            if (strain_magnitude * creep > remaining)
                creep = remaining / strain_magnitude;
            if (!(creep > T(0)))
                return;

            beam.rest_length += deviation * creep;
            beam.accumulated_plastic_strain += strain_magnitude * creep;
            if (beam.accumulated_plastic_strain > beam.maximum_plastic_strain)
                beam.accumulated_plastic_strain = beam.maximum_plastic_strain;
        }

        /**
         * @brief Walks a beam's rest length toward its current length, from a node array.
         *
         * The form a caller holding the solver's node buffer uses; it resolves the two
         * slots and defers to the position form, so there is one plasticity rule and
         * not two that could drift.
         *
         * @tparam T The scalar element type.
         * @param bodies The owning solver's node array, indexed by the beam's slots.
         * @param beam   The beam; updated in place.
         */
        template <typename T>
        inline void apply_beam_plasticity(const RigidBodyT<T>* bodies,
                                          BeamConstraintT<T>& beam) noexcept
        {
            apply_beam_plasticity(bodies[beam.a].position, bodies[beam.b].position, beam);
        }

        /** @brief The boundary beam projection: @ref BeamProjectionT fixed to `Scalar`. */
        using BeamProjection = BeamProjectionT<Scalar>;

        /** @brief The boundary beam velocity projection. */
        using BeamVelocityProjection = BeamVelocityProjectionT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
