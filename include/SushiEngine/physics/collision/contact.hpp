/**************************************************************************/
/* contact.hpp                                                            */
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
 * @file contact.hpp
 * @brief What the narrowphase reports: one overlap between two shapes.
 *
 * Separate from the functions that produce it so the solver can name a contact
 * without naming any shape — `physics/solver` is not allowed to know what a shape
 * is (§3.2), and it does not need to in order to resolve one.
 *
 * Convention: `normal` points from the first shape toward the second and is unit
 * length; `depth` is the positive overlap along that normal. Resolving moves the
 * first shape by `-normal` and the second by `+normal`, weighted by inverse mass,
 * until `depth` reaches zero.
 */

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The result of a narrowphase test: whether, where, and how deep two shapes overlap.
         *
         * `hit` is false for a miss, in which case the other fields are unspecified.
         */
        template <typename T>
        struct Contact
        {
            bool hit = false;
            Vector3T<T> normal;  /**< Unit contact normal, from the first shape to the second. */
            T depth = 0;         /**< Positive penetration depth along @ref normal. */
            Vector3T<T> point;   /**< A representative contact point in world space. */
        };
    } // namespace Physics
} // namespace SushiEngine
