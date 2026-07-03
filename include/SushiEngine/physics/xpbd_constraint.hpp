/**************************************************************************/
/* xpbd_constraint.hpp                                                   */
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

#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A compliant distance constraint between two rigid-body attachment points.
         *
         * The rigid-body generalization of `DistanceConstraint`: instead of pinning
         * two point masses, it holds two attachment points — `local_anchor_a`/`_b`,
         * offsets in each body's own local frame — a fixed distance apart, so it can
         * express both a rope/rod between arbitrary points on two bodies and a rigid
         * link at their centers of mass (anchors at the origin). Exposes `a`/`b` in
         * the same shape `color_constraints<Constraint>()` expects, so the existing
         * graph-colouring is reused unchanged.
         */
        struct XpbdDistanceConstraint
        {
            std::uint32_t a = 0; /**< First body index. */
            std::uint32_t b = 0; /**< Second body index. */
            Vec3 local_anchor_a; /**< Attachment point on body @c a, in its local frame. */
            Vec3 local_anchor_b; /**< Attachment point on body @c b, in its local frame. */
            Scalar rest_length = Scalar(0); /**< Target distance between the attachment points. */

            /**
             * @brief XPBD compliance (inverse stiffness), in meters per Newton.
             *
             * Zero is a fully rigid (infinitely stiff) constraint; the standard
             * Projected Gauss-Seidel distance constraint is the `compliance == 0` case
             * of this constraint. Positive values give a soft spring-like constraint
             * whose stiffness does not change with the solver's iteration count or
             * step size, which is XPBD's defining property over plain PBD.
             */
            Scalar compliance = Scalar(0);
        };
    } // namespace Physics
} // namespace SushiEngine
