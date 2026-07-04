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
 *         using Vector3   = SushiBLAS::Vector3;
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
    using Vector3 = placeholder::Vector3;

    /** @brief Column-major 4x4 matrix for transforms and camera projections. */
    using Mat4 = placeholder::Mat4;

    /** @brief Unit-quaternion rotation. */
    using Quaternion = placeholder::Quaternion;

    /**
     * @brief The vector and quaternion types, parametric on element type.
     *
     * `Vector3`/`Quaternion` above fix the element to `Scalar` — the boundary
     * precision that crosses the render seam. Simulation code that must compute in a
     * runtime-selected precision (float or double in the same build) uses these
     * templates directly, e.g. `Vector3T<double>`, so a physics solver can run in one
     * precision while the renderer draws in another. This is the seam that makes the
     * float/double choice a runtime decision rather than a build-time one.
     */
    template <typename T>
    using Vector3T = placeholder::Vector3T<T>;

    /** @brief Element-parametric unit quaternion; see @ref Vector3T. */
    template <typename T>
    using QuaternionT = placeholder::QuaternionT<T>;

    /**
     * @brief Always-double 3-component vector for absolute (ECEF) world positions.
     *
     * Fixed at double precision regardless of the SE_SCALAR_DOUBLE build option,
     * which only chooses @c Scalar. Planet-scale coordinates need the extra range;
     * see FloatingOriginVector3 for the representation gameplay and physics actually
     * compute with.
     */
    using WorldVector3 = placeholder::WorldVector3;

    /** @brief Integer index of a floating-origin sector on the planet grid. */
    using SectorCoord = placeholder::SectorCoord;

    /**
     * @brief A world position split into a sector index and a `Scalar`-precision
     * local offset — the floating-origin representation used by sim and render.
     */
    using FloatingOriginVector3 = placeholder::FloatingOriginVector3;

    // Vector, matrix, and quaternion operations, from the same seam as the types.
    using placeholder::compose_transform;
    using placeholder::conjugate;
    using placeholder::cross;
    using placeholder::dot;
    using placeholder::from_floating_origin;
    using placeholder::length;
    using placeholder::look_at;
    using placeholder::mat4_from_quaternion;
    using placeholder::mul;
    using placeholder::normalize;
    using placeholder::perspective;
    using placeholder::quaternion_axis_angle;
    using placeholder::rotate;
    using placeholder::scaling;
    using placeholder::to_floating_origin;
    using placeholder::translation;
} // namespace SushiEngine
