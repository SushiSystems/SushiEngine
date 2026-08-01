/**************************************************************************/
/* beam_constraint.hpp                                                    */
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
 * @file beam_constraint.hpp
 * @brief §11.1's beam: a distance constraint that dents, and then lets go.
 *
 * A node-beam structure is a cloud of particles held by axial links, and §11.1 maps
 * the link onto "an `XpbdDistanceConstraint` with a compliance from its spring rate,
 * a damping term, a **deform threshold**, and a **break threshold**". Those four
 * additions are why this is a constraint *kind* rather than a distance constraint
 * with a lookup table beside it: two of them mutate the constraint (the rest length
 * creeps, the beam breaks) and the other two are read inside the projection, so a
 * side table would be read once per beam per substep and written back to — which is
 * the descriptor, with an indirection.
 *
 * ### What a beam does not have: anchors, and therefore torque
 *
 * `XpbdDistanceConstraintT` carries a local anchor per body so it can hold two
 * *points on two rigid bodies* a distance apart. A beam holds two **nodes**, and
 * §11.1 defines a node as a particle — a body with zero inverse inertia and no
 * meaningful orientation. An anchor on a body that cannot rotate is a constant
 * offset that could have been folded into the node's position, so the two anchors
 * would be 48 bytes per beam spent to express nothing. They are dropped, and the
 * consequence is stated rather than left to be discovered: **a beam applies no
 * torque**. Attaching a deformable shell to a rigid chassis core is §10.3's
 * attachment constraint, which does carry a lever, and not a beam.
 *
 * ### Why the material parameters are baked in
 *
 * `mu` and `lambda` sit inside `FemTetrahedronT` for the same reason `compliance`
 * and the two thresholds sit here: the projection reads them every substep, and a
 * material index would put a dependent load in front of every one of them. The
 * *plastic* parameters are not baked in, also for the same reason as the element —
 * @ref apply_beam_plasticity runs once per tick, where an extra argument costs
 * nothing and keeps creep authored in one place per material rather than copied
 * across every beam cooked from it.
 *
 * §11.2's first row is what this is for. The numbers below are derived from a
 * `SoftBodyMaterial` and a cross-section by `beam_properties.hpp`; hand-authoring
 * them stays possible, and stops being mandatory.
 */

#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief Bit flags on a beam, orthogonal to its numbers. */
        namespace BeamFlags
        {
            /** @brief Nothing set. */
            inline constexpr std::uint32_t none = 0u;

            /**
             * @brief Whether the beam is projected at all.
             *
             * A disabled beam keeps its slot and its authoring and is skipped, which
             * is what a level-of-detail switch needs: a distant vehicle's shell stops
             * being solved without its topology being destroyed and rebuilt when it
             * comes back.
             */
            inline constexpr std::uint32_t enabled = 1u << 0;

            /**
             * @brief Whether the beam has already passed its break threshold.
             *
             * Set by the scene at the tick boundary, read by the projection, which
             * skips a broken beam. The two-step — flag now, remove at the boundary —
             * is §6.6's rule that a topology change never happens against a running
             * graph, and it is exactly what @ref JointFlags::broken does for a joint.
             */
            inline constexpr std::uint32_t broken = 1u << 1;
        } // namespace BeamFlags

        /** @brief Whether @p flags describes a beam the solver should project. */
        inline bool beam_is_active(std::uint32_t flags) noexcept
        {
            return (flags & BeamFlags::enabled) != 0 && (flags & BeamFlags::broken) == 0;
        }

        /**
         * @brief One beam: two nodes, a rest length that can move, and a load it reports.
         *
         * Exposes `a`/`b` in the shape `color_constraints` and `ConstraintStore` expect,
         * so a beam colours in the same union as distance constraints, joints, contacts
         * and elements without the colourer knowing beams exist (§6.3).
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct BeamConstraintT
        {
            /** @brief The scalar element type, so a solver can derive its precision. */
            using Real = T;

            /** @brief First node slot. */
            std::uint32_t a = 0;

            /** @brief Second node slot. */
            std::uint32_t b = 0;

            /** @brief @ref BeamFlags bits. */
            std::uint32_t flags = BeamFlags::enabled;

            /**
             * @brief How many substeps have folded into the load accumulators this tick.
             *
             * Carried rather than inferred from the substep count for the reason
             * @ref JointConstraintT::force_samples gives: a beam added mid-tick, disabled
             * for part of one, or skipped because its nodes were asleep contributed to
             * fewer substeps than the tick ran.
             */
            std::uint32_t force_samples = 0;

            /**
             * @brief The length the beam currently rests at, in metres.
             *
             * **Mutable.** This is where a permanent dent lives: @ref apply_beam_plasticity
             * walks it toward the beam's current length once the load passes
             * @ref deform_force, so a panel that has been hit stays hit.
             */
            T rest_length = 0;

            /**
             * @brief The length the beam was cooked at, in metres.
             *
             * Kept beside the live one because plastic strain is only meaningful
             * relative to the shape the body was *built* at. Measuring it against
             * @ref rest_length would compare a length to itself and read zero however
             * far the beam had crept.
             */
            T initial_rest_length = 0;

            /**
             * @brief XPBD compliance of the axial row, in metres per newton.
             *
             * Zero is a rigid link and costs exactly what a soft one costs — §11.2's
             * third row, and the reason a node-beam structure here does not need a
             * 2 kHz rate to keep its springs from exploding.
             */
            T compliance = 0;

            /**
             * @brief Axial velocity damping, as a rate in inverse seconds.
             *
             * Applied in the velocity pass as a fraction `min(1, damping * h)` of the
             * relative axial velocity removed per substep, so it is a *rate* and not a
             * per-substep fraction — a beam damped at 20 s⁻¹ damps the same amount per
             * second whatever the substep count, which is the same step-size
             * independence the positional row gets from compliance.
             */
            T damping = 0;

            /**
             * @brief Axial load above which the rest length starts to creep, in newtons.
             *
             * `beamDeform`, expressed against the quantity the solver already recovers.
             * At or below zero the beam is perfectly elastic and never dents.
             */
            T deform_force = 0;

            /**
             * @brief Axial load above which the beam is broken, in newtons.
             *
             * `beamStrength`. At or below zero the beam is unbreakable. Compared
             * against @ref peak_force and not the mean, for the reason
             * @ref JointConstraintT::break_force records at length: a load whose
             * direction reverses between substeps has a mean near zero and a peak
             * that is what actually tore the beam out.
             */
            T break_force = 0;

            /**
             * @brief The fraction of the elastic deviation that becomes permanent per tick.
             *
             * `SoftBodyMaterialT::plastic_creep`, copied into the beam rather than
             * reached through a material. The element solver can afford the indirection
             * because @ref apply_fem_plasticity takes a material argument, but a beam
             * cannot be given one: `physics/soft` includes `physics/constraints` and
             * not the other way about, and a constraint kind that named a soft-body
             * material would invert that. Two scalars per beam is the price of the
             * layering, and it is a price a vehicle's few thousand beams can pay.
             */
            T plastic_creep = 0;

            /** @brief The most permanent strain the beam may take before it hardens. */
            T maximum_plastic_strain = 0;

            /**
             * @brief How much permanent strain the beam has taken, dimensionless.
             *
             * The accumulator @ref maximum_plastic_strain bounds, so a panel
             * work-hardens instead of creeping without limit under a load that never
             * lets up.
             */
            T accumulated_plastic_strain = 0;

            /**
             * @brief The signed axial load from the last projected substep, in newtons.
             *
             * An output. **Positive is tension** — the beam being pulled longer than it
             * rests at and pulling its nodes back together. Sign is kept rather than
             * folded into a magnitude because a structure's failure mode differs by it:
             * a strut that buckles and a tie that snaps are the same number with
             * different signs, and §11.3's cooker is where that distinction will be
             * given different thresholds.
             */
            T axial_force = 0;

            /**
             * @brief The largest load magnitude any single substep of the tick carried.
             *
             * An output, and what @ref break_force is measured against.
             */
            T peak_force = 0;

            /**
             * @brief Sum over the tick's substeps of the signed axial load.
             *
             * An output. Divide by @ref force_samples for the mean; see @ref beam_force.
             * The mean is the load readout an inspector shows — *which way and how hard
             * is this member being worked* — and the peak is what decides whether it is
             * still there. The same two questions §10.4's recovery answers for a joint.
             */
            T force_sum = 0;
        };

        /**
         * @brief The mean axial load a beam carried over the last tick, in newtons.
         *
         * Zero before the beam has been stepped, which is the honest answer rather than
         * a division by zero.
         *
         * @tparam T The scalar element type.
         * @param beam The beam to read.
         * @return The mean signed load; positive in tension.
         */
        template <typename T>
        inline T beam_force(const BeamConstraintT<T>& beam) noexcept
        {
            if (beam.force_samples == 0)
                return T(0);
            return beam.force_sum / T(beam.force_samples);
        }

        /**
         * @brief The beam's permanent length change as a fraction of its cooked length.
         *
         * The *mukavemet* readout for a structural member (§9.3's rigid analogue):
         * positive where the beam has been stretched permanently, negative where it has
         * been crushed. Zero for a beam that has never yielded, and zero rather than a
         * division by zero for a degenerate beam of no length.
         *
         * @tparam T The scalar element type.
         * @param beam The beam to read.
         */
        template <typename T>
        inline T beam_plastic_strain(const BeamConstraintT<T>& beam) noexcept
        {
            if (!(beam.initial_rest_length > T(0)))
                return T(0);
            return (beam.rest_length - beam.initial_rest_length) / beam.initial_rest_length;
        }

        /**
         * @brief Whether a beam's load has passed its break threshold.
         *
         * Measured against the peak substep load; see @ref BeamConstraintT::break_force.
         * False for a beam that has not been stepped, so a structure built and inspected
         * before its first tick does not report itself as already failed.
         *
         * @tparam T The scalar element type.
         * @param beam The beam to test, after a step.
         * @return True when the beam should be broken.
         */
        template <typename T>
        inline bool beam_should_break(const BeamConstraintT<T>& beam) noexcept
        {
            if (beam.force_samples == 0)
                return false;
            return beam.break_force > T(0) && beam.peak_force > beam.break_force;
        }

        /** @brief The boundary beam: @ref BeamConstraintT fixed to `Scalar`. */
        using BeamConstraint = BeamConstraintT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
