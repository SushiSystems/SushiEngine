/**************************************************************************/
/* blas_placeholder.hpp                                                   */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/* Licensed under the Apache License, Version 2.0.                         */
/**************************************************************************/

#pragma once

#include <cstddef>

namespace SushiEngine
{
    /**
     * @brief Temporary stand-in for the value types SushiBLAS will own.
     *
     * The engine's vector and scalar types belong to SushiBLAS (tensors, and
     * floats derived from them). Until that library exists this namespace holds
     * the smallest device-usable placeholders the simulation needs. It is reached
     * only through core/types.hpp, so when SushiBLAS lands this whole file is
     * deleted and the seam re-pointed in one place. Do not reference it directly.
     */
    namespace placeholder
    {
        /** @brief Scalar element type; maps to a SushiBLAS float later. */
        using Float = float;

        /**
         * @brief A trivially copyable 3-component vector usable in device code.
         *
         * Only the operations the integrator needs are defined; everything richer
         * is SushiBLAS's job, not the engine's.
         */
        struct Vec3
        {
            Float x = 0;
            Float y = 0;
            Float z = 0;

            /** @brief Componentwise sum. */
            constexpr Vec3 operator+(const Vec3& o) const noexcept
            {
                return Vec3{x + o.x, y + o.y, z + o.z};
            }

            /** @brief Scaling by a scalar. */
            constexpr Vec3 operator*(Float s) const noexcept
            {
                return Vec3{x * s, y * s, z * s};
            }
        };
    } // namespace placeholder
} // namespace SushiEngine
