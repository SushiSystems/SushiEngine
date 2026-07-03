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
#include <cstdint>

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
        /**
         * @brief Scalar element type; maps to a SushiBLAS float later.
         *
         * Single precision by default; the SE_SCALAR_DOUBLE build option (declared in
         * cmake/ProjectOptions.cmake, threaded on the SushiEngine INTERFACE target)
         * switches the whole engine to double. This one line is the entire precision
         * seam — everything downstream is written in terms of @c Float, not a literal.
         */
#ifdef SE_SCALAR_DOUBLE
        using Float = double;
#else
        using Float = float;
#endif

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
         * @brief Always-double 3-component vector for absolute world coordinates.
         *
         * ECEF positions on a planet-sized world need double precision regardless of
         * @c Float's build-time choice (SE_SCALAR_DOUBLE picks the render/simulation
         * Scalar, not the coordinate system's absolute precision). WorldVec3 is that
         * fixed precision, independent of the Scalar seam above.
         */
        struct WorldVec3
        {
            double x = 0;
            double y = 0;
            double z = 0;

            /**
             * @brief Componentwise sum.
             * @param o The other vector to add.
             * @return A new vector containing the sum.
             */
            constexpr WorldVec3 operator+(const WorldVec3& o) const noexcept
            {
                return WorldVec3{x + o.x, y + o.y, z + o.z};
            }

            /**
             * @brief Componentwise difference.
             * @param o The vector to subtract.
             * @return A new vector containing this minus @p o.
             */
            constexpr WorldVec3 operator-(const WorldVec3& o) const noexcept
            {
                return WorldVec3{x - o.x, y - o.y, z - o.z};
            }
        };

        /**
         * @brief Integer index of a floating-origin sector on the planet grid.
         *
         * Each sector is a cube of world space @c sector_size units on a side; a
         * sector's own corner is always within @c sector_size of the coordinates
         * inside it, which is what keeps the local offset in FloatingOriginVec3
         * representable in single precision even far from the world origin.
         */
        struct SectorCoord
        {
            std::int64_t x = 0;
            std::int64_t y = 0;
            std::int64_t z = 0;

            /** @brief Componentwise equality. */
            constexpr bool operator==(const SectorCoord& o) const noexcept
            {
                return x == o.x && y == o.y && z == o.z;
            }

            /** @brief Componentwise inequality. */
            constexpr bool operator!=(const SectorCoord& o) const noexcept
            {
                return !(*this == o);
            }
        };

        /**
         * @brief A world position split into a sector index and a local offset.
         *
         * The sector's corner sits at @c sector * sector_size in world (ECEF) space;
         * @c local is the offset from that corner, in @c Float precision. Rebasing
         * (moving the local origin as an entity crosses a sector boundary) is what
         * keeps @c local small, which is the entire point of a floating origin: it
         * lets rendering, physics, and gameplay work in single precision without
         * losing accuracy at planetary distances.
         */
        struct FloatingOriginVec3
        {
            SectorCoord sector;
            Vec3 local;
        };

        /**
         * @brief Decomposes an absolute world position into sector + local offset.
         * @param world       Absolute position in world (ECEF) space.
         * @param sector_size Side length of one sector, in world units (> 0).
         * @return The sector containing @p world and the local offset within it.
         */
        inline FloatingOriginVec3 to_floating_origin(const WorldVec3& world,
                                                      double sector_size) noexcept
        {
            const auto sector_index = [sector_size](double v) -> std::int64_t
            {
                return static_cast<std::int64_t>(std::floor(v / sector_size));
            };
            const SectorCoord sector{sector_index(world.x), sector_index(world.y),
                                      sector_index(world.z)};
            const WorldVec3 corner{sector.x * sector_size, sector.y * sector_size,
                                    sector.z * sector_size};
            return FloatingOriginVec3{
                sector,
                Vec3{static_cast<Float>(world.x - corner.x),
                     static_cast<Float>(world.y - corner.y),
                     static_cast<Float>(world.z - corner.z)}};
        }

        /**
         * @brief Recomposes a sector + local offset into an absolute world position.
         * @param position    Sector index and local offset to recompose.
         * @param sector_size Side length of one sector, in world units (> 0); must
         *                    match the value used to produce @p position.
         * @return The absolute position in world (ECEF) space.
         */
        inline WorldVec3 from_floating_origin(const FloatingOriginVec3& position,
                                               double sector_size) noexcept
        {
            return WorldVec3{
                position.sector.x * sector_size + static_cast<double>(position.local.x),
                position.sector.y * sector_size + static_cast<double>(position.local.y),
                position.sector.z * sector_size + static_cast<double>(position.local.z)};
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

        /** @brief The conjugate (inverse, for unit quaternions) of @p q. */
        inline Quat conjugate(const Quat& q) noexcept
        {
            return Quat{-q.x, -q.y, -q.z, q.w};
        }

        /**
         * @brief Rotates @p v by unit quaternion @p q.
         *
         * Uses the standard two-cross-product form (no matrix build): with
         * `qv = (q.x, q.y, q.z)`, `v' = v + 2*w*(qv x v) + 2*(qv x (qv x v))`.
         */
        inline Vec3 rotate(const Quat& q, const Vec3& v) noexcept
        {
            const Vec3 qv{q.x, q.y, q.z};
            const Vec3 t = cross(qv, v) * Float(2);
            return v + t * q.w + cross(qv, t);
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
