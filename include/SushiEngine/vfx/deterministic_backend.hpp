/**************************************************************************/
/* deterministic_backend.hpp                                              */
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
 * @file deterministic_backend.hpp
 * @brief The CPU-deterministic particle backend — a fixed-pool, byte-reproducible integrator.
 *
 * The gameplay-authoritative half of the hybrid system (design §6). Its entire per-emitter
 * state is a @ref DeterministicEmitterState: a fixed-capacity pool of @ref GpuParticle, an
 * integer count, a @ref Pcg32, and a handful of scalars — no heap, no pointers — so a state is
 * trivially copyable and a rolled-back-then-replayed tick reproduces it byte-for-byte. It runs
 * a fixed-step Euler integrator over the same @ref CompiledEmitter and baked LUTs the GPU path
 * uses, so an effect looks the same whichever domain simulates it. Nothing here reads the wall
 * clock or a global RNG; the only randomness is drawn from the state's own generator.
 *
 * The stepper is a set of static methods on @ref CpuDeterministicBackend (no instance state):
 * @ref reset seeds a pool, @ref step advances it one fixed tick.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/vfx/compiled_emitter.hpp>
#include <SushiEngine/vfx/random.hpp>

namespace SushiEngine
{
    namespace Vfx
    {
        /**
         * @brief One emitter's complete deterministic simulation state.
         *
         * Trivially copyable and pointer-free, so it may be snapshotted with a memcpy and
         * compared with memcmp — the property the rollback contract needs. The pool holds up to
         * @ref MAX_DETERMINISTIC_PARTICLES live particles packed in [0, @ref alive_count).
         */
        struct DeterministicEmitterState
        {
            GpuParticle particles[MAX_DETERMINISTIC_PARTICLES]; /**< Packed live pool. */
            std::uint32_t alive_count = 0;      /**< Live particles, in [0, capacity]. */
            std::uint32_t spawn_serial = 0;     /**< Monotonic id assigned to each spawned particle. */
            Pcg32 rng;                          /**< The one and only source of randomness. */
            float time = 0.0f;                  /**< Seconds since the emitter started. */
            float spawn_accumulator = 0.0f;     /**< Fractional continuous-spawn carry. */
        };

        static_assert(std::is_trivially_copyable<DeterministicEmitterState>::value,
                      "DeterministicEmitterState must be byte-snapshottable for rollback.");

        /**
         * @brief The stateless deterministic particle stepper.
         *
         * All methods are static; the caller owns the @ref DeterministicEmitterState. This keeps
         * the backend a pure function of (state, emitter, dt, transform), which is exactly what
         * makes replay reproducible.
         */
        class CpuDeterministicBackend
        {
            public:
                /**
                 * @brief Clears a pool and seeds its generator.
                 * @param state The pool to reset.
                 * @param seed  The emitter seed; the stream is @c seed with sequence @c seed.
                 */
                static void reset(DeterministicEmitterState& state, std::uint32_t seed) noexcept
                {
                    state.alive_count = 0;
                    state.spawn_serial = 0;
                    state.time = 0.0f;
                    state.spawn_accumulator = 0.0f;
                    state.rng.seed(seed, static_cast<std::uint64_t>(seed) * 2u + 1u);
                }

                /**
                 * @brief Advances a pool by one fixed step.
                 *
                 * Spawns the step's new particles (continuous rate + bursts), then integrates
                 * every live particle (forces, drag, turbulence, motion, age), retiring the
                 * dead by swap-removal and applying the size/colour-over-life LUTs.
                 *
                 * @param state             The pool to advance (mutated in place).
                 * @param emitter           The compiled emitter driving the pool.
                 * @param effect            The compiled effect owning the baked LUTs.
                 * @param dt                The fixed step, seconds (> 0).
                 * @param emitter_position  The emitter's world position.
                 * @param emitter_rotation  The emitter's world orientation.
                 */
                static void step(DeterministicEmitterState& state, const CompiledEmitter& emitter,
                                 const CompiledEffect& effect, float dt,
                                 const Vector3& emitter_position,
                                 const Quaternion& emitter_rotation) noexcept
                {
                    const float previous_time = state.time;
                    state.time += dt;

                    emit(state, emitter, dt, previous_time, emitter_position, emitter_rotation);
                    integrate(state, emitter, effect, dt);
                }

            private:
                /** @brief The clamped live capacity of an emitter's pool. */
                static std::uint32_t pool_capacity(const CompiledEmitter& emitter) noexcept
                {
                    return emitter.capacity < MAX_DETERMINISTIC_PARTICLES
                               ? emitter.capacity
                               : MAX_DETERMINISTIC_PARTICLES;
                }

                /** @brief Spawns this step's continuous-rate and burst particles. */
                static void emit(DeterministicEmitterState& state, const CompiledEmitter& emitter,
                                 float dt, float previous_time, const Vector3& emitter_position,
                                 const Quaternion& emitter_rotation) noexcept
                {
                    const bool looping = (emitter.flags & EMITTER_LOOPING) != 0;
                    const bool within_life = looping || previous_time < emitter.duration;

                    std::uint32_t spawn_count = 0;
                    if (within_life && emitter.spawn_rate > 0.0f)
                    {
                        state.spawn_accumulator += emitter.spawn_rate * dt;
                        spawn_count = static_cast<std::uint32_t>(state.spawn_accumulator);
                        state.spawn_accumulator -= static_cast<float>(spawn_count);
                    }
                    for (std::uint32_t i = 0; i < spawn_count; ++i)
                        spawn_one(state, emitter, emitter_position, emitter_rotation);

                    for (std::uint32_t b = 0; b < emitter.burst_count; ++b)
                    {
                        const std::uint32_t fires = burst_fire_count(
                            previous_time, state.time, emitter.burst_time[b], emitter.duration, looping);
                        for (std::uint32_t f = 0; f < fires; ++f)
                            for (std::uint32_t c = 0; c < emitter.burst_amount[b]; ++c)
                                spawn_one(state, emitter, emitter_position, emitter_rotation);
                    }
                }

                /**
                 * @brief Counts how many times a burst fires over the step's time interval.
                 *
                 * A burst fires at @c burst_time + k*duration for integer k >= 0 (once, at
                 * @c burst_time, when not looping). Counts the k whose fire time lands in
                 * [previous, now).
                 */
                static std::uint32_t burst_fire_count(float previous, float now, float burst_time,
                                                      float duration, bool looping) noexcept
                {
                    if (!looping)
                        return (previous <= burst_time && burst_time < now) ? 1u : 0u;
                    if (duration <= 0.0f)
                        return 0u;
                    // Smallest k with burst_time + k*duration >= previous, largest with < now.
                    const float k_low_real = (previous - burst_time) / duration;
                    float k_low = std::ceil(k_low_real);
                    if (k_low < 0.0f)
                        k_low = 0.0f;
                    const float k_high_real = (now - burst_time) / duration;
                    const float k_high = std::ceil(k_high_real) - 1.0f; // strictly < now
                    if (k_high < k_low)
                        return 0u;
                    return static_cast<std::uint32_t>(k_high - k_low) + 1u;
                }

                /** @brief Births one particle from the shape and init modules. */
                static void spawn_one(DeterministicEmitterState& state, const CompiledEmitter& emitter,
                                      const Vector3& emitter_position,
                                      const Quaternion& emitter_rotation) noexcept
                {
                    if (state.alive_count >= pool_capacity(emitter))
                        return;

                    Vector3T<float> local_position{0, 0, 0};
                    Vector3T<float> local_direction{0, 1, 0};
                    sample_shape(state.rng, emitter, local_position, local_direction);

                    const float speed = state.rng.next_range(emitter.speed_min, emitter.speed_max);
                    const float size = state.rng.next_range(emitter.size_min, emitter.size_max);
                    const float lifetime =
                        state.rng.next_range(emitter.lifetime_min, emitter.lifetime_max);
                    const float rotation =
                        state.rng.next_range(emitter.rotation_min, emitter.rotation_max);
                    const float angular = state.rng.next_range(emitter.angular_velocity_min,
                                                               emitter.angular_velocity_max);

                    const Vector3 world_position =
                        emitter_position + rotate(emitter_rotation, to_double(local_position));
                    const Vector3 world_velocity =
                        rotate(emitter_rotation, to_double(local_direction * speed));

                    GpuParticle& particle = state.particles[state.alive_count];
                    particle = GpuParticle{};
                    particle.position[0] = static_cast<float>(world_position.x);
                    particle.position[1] = static_cast<float>(world_position.y);
                    particle.position[2] = static_cast<float>(world_position.z);
                    particle.velocity[0] = static_cast<float>(world_velocity.x);
                    particle.velocity[1] = static_cast<float>(world_velocity.y);
                    particle.velocity[2] = static_cast<float>(world_velocity.z);
                    particle.color[0] = emitter.color[0];
                    particle.color[1] = emitter.color[1];
                    particle.color[2] = emitter.color[2];
                    particle.alpha = 1.0f;
                    particle.size = size;
                    particle.birth_size = size;
                    particle.rotation = rotation;
                    particle.angular_velocity = angular;
                    particle.lifetime = lifetime > 0.0f ? lifetime : 0.0001f;
                    particle.age = 0.0f;
                    particle.life = 1.0f;
                    particle.seed = state.spawn_serial;
                    particle.flipbook_frame = 0;

                    ++state.alive_count;
                    ++state.spawn_serial;
                }

                /** @brief Integrates every live particle one step, retiring the dead. */
                static void integrate(DeterministicEmitterState& state, const CompiledEmitter& emitter,
                                      const CompiledEffect& effect, float dt) noexcept
                {
                    const float* size_row = effect.curve_row(emitter.size_curve_lut);
                    const float* color_row = effect.gradient_row(emitter.color_gradient_lut);
                    const bool has_gravity = (emitter.update_flags & UPDATE_GRAVITY) != 0;
                    const bool has_drag = (emitter.update_flags & UPDATE_DRAG) != 0;
                    const bool has_turbulence = (emitter.update_flags & UPDATE_TURBULENCE) != 0;
                    const bool has_size = (emitter.update_flags & UPDATE_SIZE_OVER_LIFE) != 0;
                    const bool has_color = (emitter.update_flags & UPDATE_COLOR_OVER_LIFE) != 0;
                    const std::uint32_t flip_cells = emitter.flipbook_rows * emitter.flipbook_columns;

                    std::uint32_t i = 0;
                    while (i < state.alive_count)
                    {
                        GpuParticle& p = state.particles[i];

                        float ax = 0.0f, ay = 0.0f, az = 0.0f;
                        if (has_gravity)
                        {
                            ax += emitter.gravity[0];
                            ay += emitter.gravity[1];
                            az += emitter.gravity[2];
                        }
                        if (has_turbulence)
                        {
                            float cx, cy, cz;
                            curl_noise(p.position[0] * emitter.turbulence_frequency,
                                       p.position[1] * emitter.turbulence_frequency,
                                       p.position[2] * emitter.turbulence_frequency, cx, cy, cz);
                            ax += cx * emitter.turbulence_amplitude;
                            ay += cy * emitter.turbulence_amplitude;
                            az += cz * emitter.turbulence_amplitude;
                        }

                        p.velocity[0] += ax * dt;
                        p.velocity[1] += ay * dt;
                        p.velocity[2] += az * dt;
                        if (has_drag)
                        {
                            float damp = 1.0f - emitter.drag_coefficient * dt;
                            if (damp < 0.0f)
                                damp = 0.0f;
                            p.velocity[0] *= damp;
                            p.velocity[1] *= damp;
                            p.velocity[2] *= damp;
                        }
                        p.position[0] += p.velocity[0] * dt;
                        p.position[1] += p.velocity[1] * dt;
                        p.position[2] += p.velocity[2] * dt;
                        p.rotation += p.angular_velocity * dt;
                        p.age += dt;

                        if (p.age >= p.lifetime)
                        {
                            state.particles[i] = state.particles[state.alive_count - 1];
                            --state.alive_count;
                            continue;
                        }

                        const float normalized_age = p.age / p.lifetime;
                        p.life = 1.0f - normalized_age;
                        if (has_size)
                            p.size = p.birth_size * sample_curve_lut(size_row, normalized_age, 1.0f);
                        if (has_color)
                        {
                            float rgba[4];
                            sample_gradient_lut(color_row, normalized_age, rgba);
                            p.color[0] = rgba[0];
                            p.color[1] = rgba[1];
                            p.color[2] = rgba[2];
                            p.alpha = rgba[3];
                        }
                        if (flip_cells > 1)
                        {
                            std::uint32_t frame = static_cast<std::uint32_t>(
                                normalized_age * static_cast<float>(flip_cells));
                            if (frame >= flip_cells)
                                frame = flip_cells - 1;
                            p.flipbook_frame = frame;
                        }
                        ++i;
                    }
                }

                /** @brief Widens a float vector to the boundary double precision. */
                static Vector3 to_double(const Vector3T<float>& v) noexcept
                {
                    return Vector3{static_cast<Scalar>(v.x), static_cast<Scalar>(v.y),
                                   static_cast<Scalar>(v.z)};
                }

                /**
                 * @brief Samples a birth position and emit direction in the emitter's local frame.
                 *
                 * The local frame's up is +Y. Positions are inside (or on the shell of) the birth
                 * volume; directions are unit vectors pointing the way a particle initially moves.
                 */
                static void sample_shape(Pcg32& rng, const CompiledEmitter& emitter,
                                         Vector3T<float>& out_position,
                                         Vector3T<float>& out_direction) noexcept
                {
                    const bool shell = (emitter.shape_flags & SHAPE_EMIT_FROM_SHELL) != 0;
                    switch (emitter.shape)
                    {
                        case EmitterShape::Point:
                            out_position = Vector3T<float>{0, 0, 0};
                            out_direction = Vector3T<float>{0, 1, 0};
                            break;
                        case EmitterShape::Sphere:
                        case EmitterShape::Hemisphere:
                        {
                            Vector3T<float> dir = sample_unit_sphere(rng);
                            if (emitter.shape == EmitterShape::Hemisphere && dir.y < 0.0f)
                                dir.y = -dir.y;
                            const float r = shell ? emitter.shape_radius
                                                  : emitter.shape_radius * std::cbrt(rng.next_float());
                            out_position = dir * r;
                            out_direction = dir;
                            break;
                        }
                        case EmitterShape::Cone:
                        {
                            const float base = std::sqrt(rng.next_float()) * emitter.shape_radius;
                            const float phi = rng.next_range(0.0f, emitter.shape_arc);
                            out_position = Vector3T<float>{base * std::cos(phi), 0.0f, base * std::sin(phi)};
                            const float cone = emitter.shape_cone_angle;
                            const float cos_max = std::cos(cone);
                            const float cos_theta = rng.next_range(cos_max, 1.0f);
                            const float sin_theta = std::sqrt(std::max(0.0f, 1.0f - cos_theta * cos_theta));
                            const float dphi = rng.next_range(0.0f, 6.2831853f);
                            out_direction = Vector3T<float>{sin_theta * std::cos(dphi), cos_theta,
                                                            sin_theta * std::sin(dphi)};
                            break;
                        }
                        case EmitterShape::Box:
                            out_position = Vector3T<float>{
                                rng.next_range(-emitter.shape_box_half_extents[0], emitter.shape_box_half_extents[0]),
                                rng.next_range(-emitter.shape_box_half_extents[1], emitter.shape_box_half_extents[1]),
                                rng.next_range(-emitter.shape_box_half_extents[2], emitter.shape_box_half_extents[2])};
                            out_direction = Vector3T<float>{0, 1, 0};
                            break;
                        case EmitterShape::Circle:
                        {
                            const float phi = rng.next_range(0.0f, emitter.shape_arc);
                            const float r = shell ? emitter.shape_radius
                                                  : emitter.shape_radius * std::sqrt(rng.next_float());
                            out_position = Vector3T<float>{r * std::cos(phi), 0.0f, r * std::sin(phi)};
                            out_direction = Vector3T<float>{std::cos(phi), 0.0f, std::sin(phi)};
                            break;
                        }
                    }
                }

                /** @brief A uniformly-distributed unit vector on the sphere. */
                static Vector3T<float> sample_unit_sphere(Pcg32& rng) noexcept
                {
                    const float z = rng.next_range(-1.0f, 1.0f);
                    const float phi = rng.next_range(0.0f, 6.2831853f);
                    const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
                    return Vector3T<float>{r * std::cos(phi), z, r * std::sin(phi)};
                }

                /** @brief A 3D integer hash in [0, 1), used to build the noise field. */
                static float hash(std::int32_t x, std::int32_t y, std::int32_t z) noexcept
                {
                    std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393u +
                                      static_cast<std::uint32_t>(y) * 668265263u +
                                      static_cast<std::uint32_t>(z) * 2147483647u;
                    h = (h ^ (h >> 13u)) * 1274126177u;
                    h ^= h >> 16u;
                    return static_cast<float>(h) * (1.0f / 4294967296.0f);
                }

                /** @brief Trilinearly-interpolated value noise at a point. */
                static float value_noise(float x, float y, float z) noexcept
                {
                    const float fx = std::floor(x), fy = std::floor(y), fz = std::floor(z);
                    const std::int32_t ix = static_cast<std::int32_t>(fx);
                    const std::int32_t iy = static_cast<std::int32_t>(fy);
                    const std::int32_t iz = static_cast<std::int32_t>(fz);
                    const float tx = x - fx, ty = y - fy, tz = z - fz;
                    const float ux = smooth(tx), uy = smooth(ty), uz = smooth(tz);
                    const float c000 = hash(ix, iy, iz), c100 = hash(ix + 1, iy, iz);
                    const float c010 = hash(ix, iy + 1, iz), c110 = hash(ix + 1, iy + 1, iz);
                    const float c001 = hash(ix, iy, iz + 1), c101 = hash(ix + 1, iy, iz + 1);
                    const float c011 = hash(ix, iy + 1, iz + 1), c111 = hash(ix + 1, iy + 1, iz + 1);
                    const float x00 = c000 + (c100 - c000) * ux;
                    const float x10 = c010 + (c110 - c010) * ux;
                    const float x01 = c001 + (c101 - c001) * ux;
                    const float x11 = c011 + (c111 - c011) * ux;
                    const float y0 = x00 + (x10 - x00) * uy;
                    const float y1 = x01 + (x11 - x01) * uy;
                    return y0 + (y1 - y0) * uz;
                }

                /** @brief Smoothstep-family fade for value-noise interpolation. */
                static float smooth(float t) noexcept
                {
                    return t * t * (3.0f - 2.0f * t);
                }

                /**
                 * @brief A divergence-free curl of a noise potential at a point.
                 *
                 * Three decorrelated noise fields form a vector potential; its numerical curl is
                 * divergence-free, so particles swirl through eddies without converging or
                 * diverging. Central differences over a fixed epsilon keep it a pure function.
                 */
                static void curl_noise(float x, float y, float z, float& out_x, float& out_y,
                                       float& out_z) noexcept
                {
                    const float e = 0.1f;
                    const float inv = 1.0f / (2.0f * e);
                    // Potential components sampled from offset lattices for decorrelation.
                    const float p1_y1 = value_noise(x, y + e, z + 41.0f);
                    const float p1_y0 = value_noise(x, y - e, z + 41.0f);
                    const float p1_z1 = value_noise(x, y + 17.0f, z + e);
                    const float p1_z0 = value_noise(x, y + 17.0f, z - e);
                    const float p2_z1 = value_noise(x + 23.0f, y, z + e);
                    const float p2_z0 = value_noise(x + 23.0f, y, z - e);
                    const float p2_x1 = value_noise(x + e, y + 7.0f, z);
                    const float p2_x0 = value_noise(x - e, y + 7.0f, z);
                    const float p3_x1 = value_noise(x + e, y, z + 3.0f);
                    const float p3_x0 = value_noise(x - e, y, z + 3.0f);
                    const float p3_y1 = value_noise(x, y + e, z + 29.0f);
                    const float p3_y0 = value_noise(x, y - e, z + 29.0f);
                    out_x = ((p3_y1 - p3_y0) - (p2_z1 - p2_z0)) * inv;
                    out_y = ((p1_z1 - p1_z0) - (p3_x1 - p3_x0)) * inv;
                    out_z = ((p2_x1 - p2_x0) - (p1_y1 - p1_y0)) * inv;
                }
        };
    } // namespace Vfx
} // namespace SushiEngine
