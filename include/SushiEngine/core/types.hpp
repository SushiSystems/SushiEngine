/**************************************************************************/
/* types.hpp                                                              */
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
 * @file types.hpp
 * @brief The single integration seam for the linear-algebra backend.
 *
 * Every part of the engine takes its scalar and vector types from here and
 * nowhere else. SushiBLAS will own these types (tensors, and the floats derived
 * from them); this file is the one place that decides where they come from.
 *
 * When SushiBLAS is available, replace the placeholder block below with:
 *
 *     #include <SushiBLAS/SushiBLAS.hpp>
 *     namespace SushiEngine
 *     {
 *         using Scalar = SushiBLAS::Float;
 *         using Vec3   = SushiBLAS::Vec3;
 *     }
 *
 * and delete core/blas_placeholder.hpp. Nothing else in the engine changes.
 */

#include <SushiEngine/core/blas_placeholder.hpp>

namespace SushiEngine
{
    /** @brief Scalar type used throughout the engine. */
    using Scalar = placeholder::Float;

    /** @brief Three-component vector used for positions and velocities. */
    using Vec3 = placeholder::Vec3;
} // namespace SushiEngine
