/**************************************************************************/
/* blas_placeholder.hpp                                                   */
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

#include <cmath>
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

            /**
             * @brief Componentwise sum.
             * @param o The other vector to add.
             * @return A new vector containing the sum.
             */
            constexpr Vec3 operator+(const Vec3& o) const noexcept
            {
                return Vec3{x + o.x, y + o.y, z + o.z};
            }

            /**
             * @brief Scaling by a scalar.
             * @param s The scalar value to multiply by.
             * @return A new vector with scaled components.
             */
            constexpr Vec3 operator*(Float s) const noexcept
            {
                return Vec3{x * s, y * s, z * s};
            }

            /**
             * @brief Componentwise difference.
             * @param o The vector to subtract.
             * @return A new vector containing this minus @p o.
             */
            constexpr Vec3 operator-(const Vec3& o) const noexcept
            {
                return Vec3{x - o.x, y - o.y, z - o.z};
            }
        };

        /** @brief Dot product of two vectors. */
        inline Float dot(const Vec3& a, const Vec3& b) noexcept
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        /** @brief Right-handed cross product a x b. */
        inline Vec3 cross(const Vec3& a, const Vec3& b) noexcept
        {
            return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
        }

        /** @brief Euclidean length of a vector. */
        inline Float length(const Vec3& v) noexcept
        {
            return std::sqrt(dot(v, v));
        }

        /** @brief Unit vector in the direction of @p v; returns @p v unchanged if degenerate. */
        inline Vec3 normalize(const Vec3& v) noexcept
        {
            const Float len = length(v);
            return len > Float(0) ? v * (Float(1) / len) : v;
        }

        /**
         * @brief A 4x4 matrix in column-major storage, GLSL's native layout.
         *
         * Element at row @c r, column @c c lives at @c m[c * 4 + r]. Only the
         * operations the renderer and camera need are provided; SushiBLAS owns the
         * rest. Trivially copyable so it can cross into device code as data.
         */
        struct Mat4
        {
            Float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        };

        /** @brief Matrix product a * b (column-major). */
        inline Mat4 mul(const Mat4& a, const Mat4& b) noexcept
        {
            Mat4 r{};
            for (int c = 0; c < 4; ++c)
                for (int row = 0; row < 4; ++row)
                {
                    Float sum = 0;
                    for (int k = 0; k < 4; ++k)
                        sum += a.m[k * 4 + row] * b.m[c * 4 + k];
                    r.m[c * 4 + row] = sum;
                }
            return r;
        }

        /** @brief A translation matrix by @p t. */
        inline Mat4 translation(const Vec3& t) noexcept
        {
            Mat4 r{};
            r.m[12] = t.x;
            r.m[13] = t.y;
            r.m[14] = t.z;
            return r;
        }

        /** @brief A non-uniform scale matrix by @p s. */
        inline Mat4 scaling(const Vec3& s) noexcept
        {
            Mat4 r{};
            r.m[0] = s.x;
            r.m[5] = s.y;
            r.m[10] = s.z;
            return r;
        }

        /**
         * @brief A right-handed perspective projection for Vulkan clip space.
         *
         * Maps depth to [0, 1] and flips Y so world up is screen up under Vulkan's
         * inverted clip Y.
         *
         * @param fovy_radians Vertical field of view in radians.
         * @param aspect       Width / height of the target.
         * @param near_plane   Near clip distance (> 0).
         * @param far_plane    Far clip distance (> near).
         * @return The projection matrix.
         */
        inline Mat4 perspective(Float fovy_radians, Float aspect, Float near_plane,
                                Float far_plane) noexcept
        {
            const Float f = Float(1) / std::tan(fovy_radians * Float(0.5));
            Mat4 r{};
            for (Float& value : r.m)
                value = 0;
            r.m[0] = f / aspect;
            r.m[5] = -f; // Vulkan Y flip
            r.m[10] = far_plane / (near_plane - far_plane);
            r.m[11] = -1;
            r.m[14] = -(far_plane * near_plane) / (far_plane - near_plane);
            return r;
        }

        /**
         * @brief A right-handed view matrix looking from @p eye toward @p center.
         * @param eye    Camera position in world space.
         * @param center The point the camera looks at.
         * @param up     The world up direction.
         * @return The view (world-to-camera) matrix.
         */
        inline Mat4 look_at(const Vec3& eye, const Vec3& center, const Vec3& up) noexcept
        {
            const Vec3 f = normalize(center - eye);
            const Vec3 s = normalize(cross(f, up));
            const Vec3 u = cross(s, f);
            Mat4 r{};
            r.m[0] = s.x;  r.m[4] = s.y;  r.m[8] = s.z;
            r.m[1] = u.x;  r.m[5] = u.y;  r.m[9] = u.z;
            r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
            r.m[12] = -dot(s, eye);
            r.m[13] = -dot(u, eye);
            r.m[14] = dot(f, eye);
            return r;
        }

        /**
         * @brief A unit quaternion rotation, @c (x, y, z) vector part and @c w scalar.
         *
         * Trivially copyable; the component storage a Transform carries and the
         * renderer resolves to a matrix.
         */
        struct Quat
        {
            Float x = 0;
            Float y = 0;
            Float z = 0;
            Float w = 1;
        };

        /** @brief A quaternion of @p angle radians about unit axis @p axis. */
        inline Quat quat_axis_angle(const Vec3& axis, Float angle) noexcept
        {
            const Float half = angle * Float(0.5);
            const Float s = std::sin(half);
            return Quat{axis.x * s, axis.y * s, axis.z * s, std::cos(half)};
        }

        /** @brief Hamilton product a * b (apply b then a). */
        inline Quat mul(const Quat& a, const Quat& b) noexcept
        {
            return Quat{
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
        }

        /** @brief Unit quaternion; returns identity if degenerate. */
        inline Quat normalize(const Quat& q) noexcept
        {
            const Float len = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (len <= Float(0))
                return Quat{};
            const Float inv = Float(1) / len;
            return Quat{q.x * inv, q.y * inv, q.z * inv, q.w * inv};
        }

        /** @brief The rotation matrix equivalent to unit quaternion @p q. */
        inline Mat4 mat4_from_quat(const Quat& q) noexcept
        {
            const Float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
            const Float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
            const Float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
            Mat4 r{};
            r.m[0] = 1 - 2 * (yy + zz); r.m[1] = 2 * (xy + wz);     r.m[2] = 2 * (xz - wy);
            r.m[4] = 2 * (xy - wz);     r.m[5] = 1 - 2 * (xx + zz); r.m[6] = 2 * (yz + wx);
            r.m[8] = 2 * (xz + wy);     r.m[9] = 2 * (yz - wx);     r.m[10] = 1 - 2 * (xx + yy);
            return r;
        }

        /**
         * @brief Composes a translation-rotation-scale model matrix.
         * @param position World position.
         * @param rotation Unit-quaternion orientation.
         * @param scale    Per-axis scale.
         * @return translation * rotation * scale.
         */
        inline Mat4 compose_transform(const Vec3& position, const Quat& rotation,
                                      const Vec3& scale) noexcept
        {
            return mul(translation(position), mul(mat4_from_quat(rotation), scaling(scale)));
        }
    } // namespace placeholder
} // namespace SushiEngine
