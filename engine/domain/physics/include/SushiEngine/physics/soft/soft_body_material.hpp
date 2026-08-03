/**************************************************************************/
/* soft_body_material.hpp                                                 */
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
 * @file soft_body_material.hpp
 * @brief §9.2's artist-facing surface: what a soft body is made of, as data.
 *
 * `PhysicsMaterialT` (`physics/core/material.hpp`) is a rigid-contact material —
 * friction, restitution, the combine modes two contacting surfaces resolve
 * between them. A soft body's constitutive parameters are a different kind of
 * thing entirely: they belong to *one* element's projection, not to a pair, so
 * there is no combine mode to speak of. That is the whole reason this is a
 * separate type in `physics/soft/` rather than a second half of
 * `PhysicsMaterialT` — the two materials answer questions nothing else asks of
 * the other.
 *
 * Presets exist so an artist starts from a name rather than a number they have
 * to guess (§9.2's own framing). The figures are textbook order-of-magnitude
 * values for the named material class, not a measured sample of any specific
 * object — the fidelity a game needs from "rubber" is that it behaves like
 * rubber relative to steel, not that it matches one manufacturer's datasheet.
 */

#include <cmath>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A soft body's constitutive, plastic, and fracture parameters.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct SoftBodyMaterialT
        {
            /** @brief Stiffness, in pascals. Rubber ~1e7, mild steel ~2e11. */
            T young_modulus = T(1e6);

            /** @brief How much the material bulges when squeezed, in [0, 0.5). */
            T poisson_ratio = T(0.4);

            /** @brief Mass per unit volume, in kg/m^3. */
            T density = T(1000);

            /**
             * @brief Velocity damping applied every substep, in [0, 1] per second.
             *
             * A simple per-particle damping (`velocity *= max(0, 1 - damping*h)`)
             * rather than Bender et al.'s "damp toward the element's own rigid-body
             * mode" — the more elaborate version measures and removes a rotating
             * element's own spin before damping, which this does not yet do. Stated
             * as a scoping decision, not an oversight: this is P6-A's minimum
             * viable damping, and it is enough that a struck body settles instead
             * of ringing forever.
             */
            T damping = T(0);

            /** @brief Von Mises stress above which deformation becomes permanent, in pascals (§9.4). */
            T yield_stress = T(1e30);

            /** @brief How fast permanent deformation accumulates past yield, in [0, 1]. */
            T plastic_creep = T(0);

            /** @brief The largest plastic strain the material can accumulate before it stops growing. */
            T maximum_plastic_strain = T(0);

            /** @brief Von Mises stress above which an element separates, in pascals (§9.5). */
            T fracture_stress = T(1e30);
        };

        /** @brief The boundary soft-body material: `SoftBodyMaterialT` fixed to `Scalar`. */
        using SoftBodyMaterial = SoftBodyMaterialT<Scalar>;

        /**
         * @brief The Lame parameters a neo-Hookean element's two constraints read.
         *
         * `mu` scales the deviatoric (shape-preserving) constraint's compliance;
         * `lambda` scales the hydrostatic (volume-preserving) one. Both are derived
         * from the artist-facing Young's modulus and Poisson ratio rather than
         * authored directly, because Young's modulus and Poisson ratio are the pair
         * an engineer or a materials datasheet actually states.
         */
        template <typename T>
        struct LameParameters
        {
            T mu = 0;
            T lambda = 0;
        };

        /**
         * @brief Derives the Lame parameters from a material's Young's modulus and Poisson ratio.
         *
         * The standard isotropic-elasticity relations:
         * `mu = E / (2(1+v))`, `lambda = E*v / ((1+v)(1-2v))`.
         *
         * @param material The material; `poisson_ratio` is clamped away from 0.5
         *                 (incompressible) and -1, where `lambda` and `mu`
         *                 respectively diverge.
         */
        template <typename T>
        inline LameParameters<T> lame_parameters(const SoftBodyMaterialT<T>& material) noexcept
        {
            T poisson = material.poisson_ratio;
            // 0.5 is the incompressible limit, where lambda diverges to infinity;
            // held a small margin short of it rather than refused, since an artist
            // sliding a parameter toward the limit should get a very stiff volume
            // term, not a not-a-number.
            if (poisson > T(0.499))
                poisson = T(0.499);
            if (poisson < T(-0.999))
                poisson = T(-0.999);

            LameParameters<T> result;
            result.mu = material.young_modulus / (T(2) * (T(1) + poisson));
            result.lambda = (material.young_modulus * poisson) /
                            ((T(1) + poisson) * (T(1) - T(2) * poisson));
            return result;
        }

        /** @brief Soft rubber: very compliant, nearly incompressible. */
        template <typename T>
        inline SoftBodyMaterialT<T> rubber_material() noexcept
        {
            SoftBodyMaterialT<T> material;
            material.young_modulus = T(1e7);
            material.poisson_ratio = T(0.49);
            material.density = T(1100);
            material.damping = T(0.5);
            material.yield_stress = T(1e30); // rubber does not yield in the plastic sense
            material.fracture_stress = T(1e7);
            return material;
        }

        /** @brief Open-cell foam: very compliant and compressible, so it does not resist volume loss. */
        template <typename T>
        inline SoftBodyMaterialT<T> foam_material() noexcept
        {
            SoftBodyMaterialT<T> material;
            material.young_modulus = T(2e5);
            material.poisson_ratio = T(0.1);
            material.density = T(100);
            material.damping = T(1.5);
            material.yield_stress = T(1e30);
            material.fracture_stress = T(5e5);
            return material;
        }

        /** @brief Soft biological tissue: extremely compliant, nearly incompressible. */
        template <typename T>
        inline SoftBodyMaterialT<T> soft_tissue_material() noexcept
        {
            SoftBodyMaterialT<T> material;
            material.young_modulus = T(6e4);
            material.poisson_ratio = T(0.48);
            material.density = T(1050);
            material.damping = T(1.0);
            material.yield_stress = T(1e30);
            material.fracture_stress = T(1e6);
            return material;
        }

        /** @brief Sheet steel: stiff, permanently dents past yield, tears past fracture (§11's body panels). */
        template <typename T>
        inline SoftBodyMaterialT<T> sheet_steel_material() noexcept
        {
            SoftBodyMaterialT<T> material;
            material.young_modulus = T(2e11);
            material.poisson_ratio = T(0.3);
            material.density = T(7850);
            material.damping = T(0.05);
            material.yield_stress = T(2.5e8);
            material.plastic_creep = T(0.6);
            material.maximum_plastic_strain = T(0.2);
            material.fracture_stress = T(4e8);
            return material;
        }

        /** @brief Structural aluminium: stiffer than it is strong, relative to steel. */
        template <typename T>
        inline SoftBodyMaterialT<T> aluminium_material() noexcept
        {
            SoftBodyMaterialT<T> material;
            material.young_modulus = T(6.9e10);
            material.poisson_ratio = T(0.33);
            material.density = T(2700);
            material.damping = T(0.05);
            material.yield_stress = T(2.0e8);
            material.plastic_creep = T(0.6);
            material.maximum_plastic_strain = T(0.15);
            material.fracture_stress = T(3.0e8);
            return material;
        }
    } // namespace Physics
} // namespace SushiEngine
