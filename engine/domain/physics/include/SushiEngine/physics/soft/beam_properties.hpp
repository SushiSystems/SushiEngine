/**************************************************************************/
/* beam_properties.hpp                                                    */
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
 * @file beam_properties.hpp
 * @brief §11.2's first row: "sheet steel" is a material, not four thousand constants.
 *
 * BeamNG's node-beam description *is* the source asset, so every beam's stiffness,
 * damping, deform and break value is a number a human typed, with no stated relation
 * to any real material. This file is the correction: a beam's four numbers are
 * derived from a `SoftBodyMaterialT` and one geometric fact — the cross-section the
 * beam stands for — by the textbook relations for an axially loaded bar.
 *
 * | Beam number | Derived as | Why |
 * |---|---|---|
 * | `compliance` | `L / (E·A)` | The axial stiffness of a bar is `k = E·A/L`, and XPBD compliance is `1/k`. |
 * | `deform_force` | `yield_stress · A` | Stress is force over area; yield is a stress, and the solver recovers a force. |
 * | `break_force` | `fracture_stress · A` | The same conversion at the other threshold. |
 * | `damping` | the material's own | Already a rate in inverse seconds, and @ref BeamConstraintT::damping is read as one. |
 *
 * ### Why this lives in `physics/soft` and not beside the beam
 *
 * `physics/soft` includes `physics/constraints`; the reverse would be a cycle. The
 * beam descriptor is a constraint and cannot name a soft-body material, so the one
 * place both are named is here. That is also the honest placement by responsibility:
 * this file knows nothing about how a beam is *projected*, only about what a
 * material means when it is stretched into a bar.
 *
 * ### The cross-section is the cooker's number, not the material's
 *
 * A material has no area. A beam's area is a property of the *structure* — how much
 * of the body's cross-section this one member is standing in for — so it is passed
 * in, and §11.3's cooker is what computes it from the lattice it placed. @ref
 * beam_tributary_area states the rule that cooker uses, here rather than inside it,
 * because it is a claim about physics (the network as a whole must carry the
 * material's real stiffness) and not about voxel grids.
 */

#include <cstddef>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/beam_constraint.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The four numbers a beam's projection reads, derived rather than authored.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct BeamProperties
        {
            /** @brief XPBD compliance of the axial row, in metres per newton. */
            T compliance = 0;

            /** @brief Axial velocity damping, as a rate in inverse seconds. */
            T damping = 0;

            /** @brief Axial load above which the rest length creeps, in newtons. */
            T deform_force = 0;

            /** @brief Axial load above which the beam breaks, in newtons. */
            T break_force = 0;
        };

        /**
         * @brief Derives a beam's stiffness and thresholds from what it is made of.
         *
         * The axial-bar relations, applied literally. A degenerate beam — no length or
         * no area — comes back rigid and unbreakable rather than with an infinity: a
         * zero-area member carries no stress by any reading, so there is no threshold
         * it could sensibly be given, and a compliance of zero is the stiffest a row
         * can be rather than a special case the projection has to test for.
         *
         * @tparam T The scalar element type.
         * @param material    What the beam is made of; `young_modulus`, `damping`,
         *                    `yield_stress` and `fracture_stress` are read.
         * @param rest_length The beam's cooked length, in metres (> 0).
         * @param area        The cross-section the beam stands for, in square metres
         *                    (> 0); see @ref beam_tributary_area.
         * @return The derived numbers, ready to be copied into a @ref BeamConstraintT.
         */
        template <typename T>
        inline BeamProperties<T> beam_properties_from_material(
            const SoftBodyMaterialT<T>& material, T rest_length, T area) noexcept
        {
            BeamProperties<T> properties;
            properties.damping = material.damping;
            if (!(rest_length > T(0)) || !(area > T(0)))
                return properties;

            const T stiffness = material.young_modulus * area / rest_length;
            properties.compliance = stiffness > T(0) ? T(1) / stiffness : T(0);

            // A material that does not yield or does not fracture carries the sentinel
            // 1e30 rather than a flag, so the conversion would produce a threshold no
            // load reaches. Passed through as zero instead, which is what the beam
            // reads as "never" — the sentinel travelling into a force would work by
            // accident and would stop working the day someone applied a big enough load.
            const T never = T(1e29);
            if (material.yield_stress < never)
                properties.deform_force = material.yield_stress * area;
            if (material.fracture_stress < never)
                properties.break_force = material.fracture_stress * area;
            return properties;
        }

        /**
         * @brief Writes a material's derived numbers onto a beam, rest length and all.
         *
         * The one call a cooker or an authoring path makes, so the four derived numbers
         * and the two plastic ones cannot be copied across in some places and forgotten
         * in others. Sets both @ref BeamConstraintT::rest_length and
         * @ref BeamConstraintT::initial_rest_length, since a beam being built has not
         * yet crept and the two must start equal or the first strain readout is wrong.
         *
         * @tparam T The scalar element type.
         * @param beam        The beam to fill; its `a`, `b` and flags are left alone.
         * @param material    What the beam is made of.
         * @param rest_length The beam's cooked length, in metres.
         * @param area        The cross-section the beam stands for, in square metres.
         */
        template <typename T>
        inline void apply_beam_material(BeamConstraintT<T>& beam,
                                        const SoftBodyMaterialT<T>& material, T rest_length,
                                        T area) noexcept
        {
            const BeamProperties<T> properties =
                beam_properties_from_material(material, rest_length, area);
            beam.rest_length = rest_length;
            beam.initial_rest_length = rest_length;
            beam.compliance = properties.compliance;
            beam.damping = properties.damping;
            beam.deform_force = properties.deform_force;
            beam.break_force = properties.break_force;
            beam.plastic_creep = material.plastic_creep;
            beam.maximum_plastic_strain = material.maximum_plastic_strain;
            beam.accumulated_plastic_strain = T(0);
        }

        /**
         * @brief The cross-section one beam of a network stands for, in square metres.
         *
         * A node-beam network is a discretization, so the question "what area does this
         * member have" has no answer from the member alone — only the network as a whole
         * is required to behave like the material. The rule is a conservation statement:
         * the network's total `Σ A·L` must equal the body's volume, because that is what
         * says the beams are made of the body's material and no more of it. Distributing
         * that volume evenly gives each beam `A = V / (n · L)`.
         *
         * The honest limitation, stated because a caller may need to correct for it: an
         * even split makes the *stiffness* right in aggregate for a roughly isotropic
         * lattice and no more than roughly right for one whose beams differ in length
         * by a large factor, since a long beam and a short one then stand for the same
         * volume and the long one is disproportionately compliant. §11.3's cooker places
         * a regular lattice, where the lengths take one of two values, so the residual
         * error is a factor of √2 on the bracing members rather than an open-ended one.
         *
         * @tparam T The scalar element type.
         * @param volume      The body's volume, in cubic metres (> 0).
         * @param beam_count  How many beams share it (> 0).
         * @param rest_length This beam's length, in metres (> 0).
         * @return The area, or zero when any input is degenerate.
         */
        template <typename T>
        inline T beam_tributary_area(T volume, std::size_t beam_count, T rest_length) noexcept
        {
            if (!(volume > T(0)) || beam_count == 0 || !(rest_length > T(0)))
                return T(0);
            return volume / (T(beam_count) * rest_length);
        }
    } // namespace Physics
} // namespace SushiEngine
