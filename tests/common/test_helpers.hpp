/**************************************************************************/
/* test_helpers.hpp                                                       */
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

// Shared test building blocks: one process-wide runtime (hardware discovery is
// expensive, so it is created once and shared) and small numeric comparators.

#pragma once

#include <cmath>
#include <cstddef>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    // Not named "Test": inside a GoogleTest TEST() body the enclosing class
    // derives from ::testing::Test, whose injected base name would shadow a
    // namespace called Test and break unqualified lookup of these helpers.
    namespace Harness
    {
        /**
         * @brief The process-wide runtime every test shares.
         *
         * The runtime owns hardware discovery and all USM storage and must outlive
         * every World/Schedule/solver built against it. Creating one per test would
         * re-run discovery and risk contending for the same devices, so a single
         * function-local static (guaranteed-elided from the create() prvalue in
         * C++17, so no move constructor is required) backs the whole suite.
         *
         * @return A reference to the shared runtime.
         */
        inline SushiRuntime::API::Runtime& shared_runtime()
        {
            static SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
            return runtime;
        }

        /**
         * @brief True when two scalars agree to within @p tolerance.
         * @param a         First value.
         * @param b         Second value.
         * @param tolerance Maximum allowed absolute difference.
         * @return Whether |a - b| <= tolerance.
         */
        inline bool approx_equal(Scalar a, Scalar b, Scalar tolerance)
        {
            return std::fabs(a - b) <= tolerance;
        }

        /**
         * @brief True when two vectors agree componentwise to within @p tolerance.
         * @param a         First vector.
         * @param b         Second vector.
         * @param tolerance Maximum allowed absolute difference on each component.
         * @return Whether every component pair is within tolerance.
         */
        inline bool approx_equal(const Vector3& a, const Vector3& b, Scalar tolerance)
        {
            return approx_equal(a.x, b.x, tolerance) &&
                   approx_equal(a.y, b.y, tolerance) &&
                   approx_equal(a.z, b.z, tolerance);
        }

        /**
         * @brief True when two absolute world vectors agree componentwise.
         *
         * The WorldVector3 counterpart of the Vector3 comparator: the astro layer works
         * in absolute heliocentric-ecliptic metres (WorldVector3), so its round-trip and
         * frame-change tests need a comparator for that type.
         *
         * @param a         First world vector.
         * @param b         Second world vector.
         * @param tolerance Maximum allowed absolute difference on each component.
         * @return Whether every component pair is within tolerance.
         */
        inline bool approx_equal(const WorldVector3& a, const WorldVector3& b, Scalar tolerance)
        {
            return approx_equal(a.x, b.x, tolerance) &&
                   approx_equal(a.y, b.y, tolerance) &&
                   approx_equal(a.z, b.z, tolerance);
        }

        /**
         * @brief True when @p vector has unit length to within @p tolerance.
         * @param vector    The vector to test.
         * @param tolerance Maximum allowed deviation of the length from 1.
         * @return Whether |length(vector) - 1| <= tolerance.
         */
        inline bool is_unit(const Vector3& vector, Scalar tolerance)
        {
            return approx_equal(length(vector), Scalar(1), tolerance);
        }

        /**
         * @brief True when three vectors form a right-handed orthonormal basis.
         *
         * Checks each axis is unit, each pair is orthogonal, and the handedness is
         * right-handed (a x b == c). The topocentric, tangent, and body-equatorial bases
         * all promise this, and several transforms are exact only because their basis is.
         *
         * @param a         First basis axis.
         * @param b         Second basis axis.
         * @param c         Third basis axis.
         * @param tolerance Maximum allowed deviation for the unit/orthogonality/handedness checks.
         * @return Whether {a, b, c} is a right-handed orthonormal basis within tolerance.
         */
        inline bool is_orthonormal_basis(const Vector3& a, const Vector3& b, const Vector3& c,
                                         Scalar tolerance)
        {
            const bool unit = is_unit(a, tolerance) && is_unit(b, tolerance) && is_unit(c, tolerance);
            const bool orthogonal = approx_equal(dot(a, b), Scalar(0), tolerance) &&
                                    approx_equal(dot(a, c), Scalar(0), tolerance) &&
                                    approx_equal(dot(b, c), Scalar(0), tolerance);
            const bool right_handed = approx_equal(cross(a, b), c, tolerance);
            return unit && orthogonal && right_handed;
        }
    } // namespace Harness
} // namespace SushiEngine
