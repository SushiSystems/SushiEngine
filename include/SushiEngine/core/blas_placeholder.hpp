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
     *
     * The vector and quaternion types are parametric on their element type so a
     * single build can compute in more than one precision at once — the boundary
     * aliases (`Vector3`, `Quaternion`) fix that element to `Float` (always double,
     * see below), while the physics/simulation core can still instantiate a narrower
     * `Vector3T<float>` beside it for a runtime-selected solve precision. This
     * anticipates the parametric element type SushiBLAS will own.
     */
    namespace placeholder
    {
        /**
         * @brief Scalar element type of the boundary types; maps to a SushiBLAS float later.
         *
         * Always double. The engine simulates planet- and solar-scale worlds, where the
         * ~1 m float32 quantisation at 1e7 m makes single precision unusable for camera
         * and transform math; double is the engine's one and only boundary/render
         * `Scalar`. It picks the element of the `Vector3`/`Quaternion`/`Mat4` aliases
         * below — the precision that crosses the `ISimulation` seam and reaches the
         * renderer (which then casts to float per draw, camera-relative). The physics
         * solve's own compute precision is chosen separately at runtime (see the
         * namespace note).
         */
        using Float = double;

        /**
         * @brief A trivially copyable 3-component vector, parametric on element type.
         *
         * Only the operations the integrator needs are defined; everything richer
         * is SushiBLAS's job, not the engine's.
         *
         * @tparam T The element type (`float` or `double`).
         */
        template <typename T>
        struct Vector3T
        {
            T x = 0;
            T y = 0;
            T z = 0;

            /**
             * @brief Componentwise sum.
             * @param o The other vector to add.
             * @return A new vector containing the sum.
             */
            constexpr Vector3T operator+(const Vector3T& o) const noexcept
            {
                return Vector3T{x + o.x, y + o.y, z + o.z};
            }

            /**
             * @brief Scaling by a scalar.
             * @param s The scalar value to multiply by.
             * @return A new vector with scaled components.
             */
            constexpr Vector3T operator*(T s) const noexcept
            {
                return Vector3T{x * s, y * s, z * s};
            }

            /**
             * @brief Componentwise difference.
             * @param o The vector to subtract.
             * @return A new vector containing this minus @p o.
             */
            constexpr Vector3T operator-(const Vector3T& o) const noexcept
            {
                return Vector3T{x - o.x, y - o.y, z - o.z};
            }
        };

        /** @brief Dot product of two vectors. */
        template <typename T>
        inline T dot(const Vector3T<T>& a, const Vector3T<T>& b) noexcept
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        /** @brief Right-handed cross product a x b. */
        template <typename T>
        inline Vector3T<T> cross(const Vector3T<T>& a, const Vector3T<T>& b) noexcept
        {
            return Vector3T<T>{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                               a.x * b.y - a.y * b.x};
        }

        /** @brief Euclidean length of a vector. */
        template <typename T>
        inline T length(const Vector3T<T>& v) noexcept
        {
            return std::sqrt(dot(v, v));
        }

        /** @brief Unit vector in the direction of @p v; returns @p v unchanged if degenerate. */
        template <typename T>
        inline Vector3T<T> normalize(const Vector3T<T>& v) noexcept
        {
            const T len = length(v);
            return len > T(0) ? v * (T(1) / len) : v;
        }

        /**
         * @brief A unit quaternion rotation, parametric on element type.
         *
         * `(x, y, z)` vector part and `w` scalar. Trivially copyable; the component
         * storage a Transform carries and the renderer resolves to a matrix.
         *
         * @tparam T The element type (`float` or `double`).
         */
        template <typename T>
        struct QuaternionT
        {
            T x = 0;
            T y = 0;
            T z = 0;
            T w = 1;
        };

        /** @brief A quaternion of @p angle radians about unit axis @p axis. */
        template <typename T>
        inline QuaternionT<T> quaternion_axis_angle(const Vector3T<T>& axis, T angle) noexcept
        {
            const T half = angle * T(0.5);
            const T s = std::sin(half);
            return QuaternionT<T>{axis.x * s, axis.y * s, axis.z * s, std::cos(half)};
        }

        /** @brief Hamilton product a * b (apply b then a). */
        template <typename T>
        inline QuaternionT<T> mul(const QuaternionT<T>& a, const QuaternionT<T>& b) noexcept
        {
            return QuaternionT<T>{
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
        }

        /** @brief The conjugate (inverse, for unit quaternions) of @p q. */
        template <typename T>
        inline QuaternionT<T> conjugate(const QuaternionT<T>& q) noexcept
        {
            return QuaternionT<T>{-q.x, -q.y, -q.z, q.w};
        }

        /**
         * @brief Rotates @p v by unit quaternion @p q.
         *
         * Uses the standard two-cross-product form (no matrix build): with
         * `qv = (q.x, q.y, q.z)`, `v' = v + 2*w*(qv x v) + 2*(qv x (qv x v))`.
         */
        template <typename T>
        inline Vector3T<T> rotate(const QuaternionT<T>& q, const Vector3T<T>& v) noexcept
        {
            const Vector3T<T> qv{q.x, q.y, q.z};
            const Vector3T<T> t = cross(qv, v) * T(2);
            return v + t * q.w + cross(qv, t);
        }

        /** @brief Unit quaternion; returns identity if degenerate. */
        template <typename T>
        inline QuaternionT<T> normalize(const QuaternionT<T>& q) noexcept
        {
            const T len = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (len <= T(0))
                return QuaternionT<T>{};
            const T inv = T(1) / len;
            return QuaternionT<T>{q.x * inv, q.y * inv, q.z * inv, q.w * inv};
        }

        /**
         * @brief The boundary 3-component vector: `Vector3T` fixed to `Float`.
         *
         * The precision that crosses the `ISimulation` seam and reaches the renderer.
         * Simulation code that must compute in a different precision instantiates
         * `Vector3T<T>` directly instead of this alias.
         */
        using Vector3 = Vector3T<Float>;

        /** @brief The boundary unit quaternion: `QuaternionT` fixed to `Float`. */
        using Quaternion = QuaternionT<Float>;

        /**
         * @brief Always-double 3-component vector for absolute world coordinates.
         *
         * A distinct type from @c Vector3 to mark, at the type level, coordinates that
         * are absolute ECEF rather than a floating-origin local offset. Both are double
         * (@c Float is double), but the separation keeps absolute-vs-local intent
         * explicit at every seam.
         */
        struct WorldVector3
        {
            double x = 0;
            double y = 0;
            double z = 0;

            /**
             * @brief Componentwise sum.
             * @param o The other vector to add.
             * @return A new vector containing the sum.
             */
            constexpr WorldVector3 operator+(const WorldVector3& o) const noexcept
            {
                return WorldVector3{x + o.x, y + o.y, z + o.z};
            }

            /**
             * @brief Componentwise difference.
             * @param o The vector to subtract.
             * @return A new vector containing this minus @p o.
             */
            constexpr WorldVector3 operator-(const WorldVector3& o) const noexcept
            {
                return WorldVector3{x - o.x, y - o.y, z - o.z};
            }
        };

        /**
         * @brief Integer index of a floating-origin sector on the planet grid.
         *
         * Each sector is a cube of world space @c sector_size units on a side; a
         * sector's own corner is always within @c sector_size of the coordinates
         * inside it, which is what keeps the local offset in FloatingOriginVector3
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
        struct FloatingOriginVector3
        {
            SectorCoord sector;
            Vector3 local;
        };

        /**
         * @brief Decomposes an absolute world position into sector + local offset.
         * @param world       Absolute position in world (ECEF) space.
         * @param sector_size Side length of one sector, in world units (> 0).
         * @return The sector containing @p world and the local offset within it.
         */
        inline FloatingOriginVector3 to_floating_origin(const WorldVector3& world,
                                                      double sector_size) noexcept
        {
            const auto sector_index = [sector_size](double v) -> std::int64_t
            {
                return static_cast<std::int64_t>(std::floor(v / sector_size));
            };
            const SectorCoord sector{sector_index(world.x), sector_index(world.y),
                                      sector_index(world.z)};
            const WorldVector3 corner{sector.x * sector_size, sector.y * sector_size,
                                    sector.z * sector_size};
            return FloatingOriginVector3{
                sector,
                Vector3{static_cast<Float>(world.x - corner.x),
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
        inline WorldVector3 from_floating_origin(const FloatingOriginVector3& position,
                                               double sector_size) noexcept
        {
            return WorldVector3{
                position.sector.x * sector_size + static_cast<double>(position.local.x),
                position.sector.y * sector_size + static_cast<double>(position.local.y),
                position.sector.z * sector_size + static_cast<double>(position.local.z)};
        }

        /**
         * @brief A 4x4 matrix in column-major storage, GLSL's native layout.
         *
         * Element at row @c r, column @c c lives at @c m[c * 4 + r]. Only the
         * operations the renderer and camera need are provided; SushiBLAS owns the
         * rest. Trivially copyable so it can cross into device code as data. Fixed at
         * `Float`: matrices are render-side (projection, view, model), never part of
         * the simulation's compute precision.
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
        inline Mat4 translation(const Vector3& t) noexcept
        {
            Mat4 r{};
            r.m[12] = t.x;
            r.m[13] = t.y;
            r.m[14] = t.z;
            return r;
        }

        /** @brief A non-uniform scale matrix by @p s. */
        inline Mat4 scaling(const Vector3& s) noexcept
        {
            Mat4 r{};
            r.m[0] = s.x;
            r.m[5] = s.y;
            r.m[10] = s.z;
            return r;
        }

        /**
         * @brief A right-handed reverse-Z perspective projection with an infinite far plane.
         *
         * Maps the near plane to clip depth 1 and infinity to 0 (reverse-Z), and flips Y
         * so world up is screen up under Vulkan's inverted clip Y. Reverse-Z spreads a
         * floating-point depth buffer's precision almost uniformly across the whole range,
         * which is what lets one camera see a 5 cm prop and a planet 10^7 m away in the same
         * frame without z-fighting; the infinite far plane means nothing is ever clipped for
         * being too distant. The pipeline must clear depth to 0 and compare with
         * GREATER_OR_EQUAL to match. @p far_plane is accepted for call-site compatibility but
         * unused — there is no far clip.
         *
         * @param fovy_radians Vertical field of view in radians.
         * @param aspect       Width / height of the target.
         * @param near_plane   Near clip distance (> 0); the only precision knob.
         * @param far_plane    Ignored (the far plane is at infinity).
         * @return The projection matrix.
         */
        inline Mat4 perspective(Float fovy_radians, Float aspect, Float near_plane,
                                Float far_plane) noexcept
        {
            (void)far_plane;
            const Float f = Float(1) / std::tan(fovy_radians * Float(0.5));
            Mat4 r{};
            for (Float& value : r.m)
                value = 0;
            r.m[0] = f / aspect;
            r.m[5] = -f; // Vulkan Y flip
            // Reverse-Z, infinite far: clip.z = near, clip.w = -z_view, so ndc.z =
            // near / -z_view — 1 at the near plane, approaching 0 at infinity.
            r.m[10] = 0;
            r.m[11] = -1;
            r.m[14] = near_plane;
            return r;
        }

        /**
         * @brief A right-handed view matrix looking from @p eye toward @p center.
         * @param eye    Camera position in world space.
         * @param center The point the camera looks at.
         * @param up     The world up direction.
         * @return The view (world-to-camera) matrix.
         */
        inline Mat4 look_at(const Vector3& eye, const Vector3& center, const Vector3& up) noexcept
        {
            const Vector3 f = normalize(center - eye);
            const Vector3 s = normalize(cross(f, up));
            const Vector3 u = cross(s, f);
            Mat4 r{};
            r.m[0] = s.x;  r.m[4] = s.y;  r.m[8] = s.z;
            r.m[1] = u.x;  r.m[5] = u.y;  r.m[9] = u.z;
            r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
            r.m[12] = -dot(s, eye);
            r.m[13] = -dot(u, eye);
            r.m[14] = dot(f, eye);
            return r;
        }

        /** @brief The rotation matrix equivalent to unit quaternion @p q. */
        inline Mat4 mat4_from_quaternion(const Quaternion& q) noexcept
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
        inline Mat4 compose_transform(const Vector3& position, const Quaternion& rotation,
                                      const Vector3& scale) noexcept
        {
            return mul(translation(position), mul(mat4_from_quaternion(rotation), scaling(scale)));
        }
    } // namespace placeholder
} // namespace SushiEngine
