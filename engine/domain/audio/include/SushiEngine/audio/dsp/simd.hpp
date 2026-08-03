/**************************************************************************/
/* simd.hpp                                                              */
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

#ifndef SUSHIENGINE_AUDIO_DSP_SIMD_HPP
#define SUSHIENGINE_AUDIO_DSP_SIMD_HPP

/**
 * @file simd.hpp
 * @brief The block-level sample kernels: gain, scaled copy, mix-accumulate, pan.
 *
 * These are the per-block hot loops the whole mixer is built from — applied to
 * *summed* bus buffers, so they run O(bus) not O(voice). On x86 the core loops use
 * a 4-wide SSE path with a scalar remainder tail; on any other target the scalar
 * fallback is used. All entry points take plain `float*` and unaligned loads/stores,
 * so a caller never has to reason about alignment (the DSP core keeps buffers
 * naturally aligned anyway). Everything here is `noexcept` and allocation-free — safe
 * to call on the audio thread.
 *
 * A wider AVX path and CPUID runtime dispatch behind a `Vec8f` wrapper is a later
 * optimisation (the seam is these free functions; their call sites do not change).
 */

#include <cmath>
#include <cstddef>

#if defined(__SSE__) || defined(_M_X64) || defined(__x86_64__) || defined(_M_IX86)
    #define SUSHIENGINE_AUDIO_SIMD_SSE 1
    #include <xmmintrin.h>
#endif

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            namespace Simd
            {
                /**
                 * @brief Scales a buffer in place by a constant gain: `buf[i] *= gain`.
                 * @param buffer      The samples to scale.
                 * @param frame_count Number of samples.
                 * @param gain        The multiplier.
                 */
                inline void apply_gain(float* buffer, int frame_count, float gain) noexcept
                {
                    int i = 0;
#if defined(SUSHIENGINE_AUDIO_SIMD_SSE)
                    const __m128 g = _mm_set1_ps(gain);
                    for (; i + 4 <= frame_count; i += 4)
                    {
                        __m128 v = _mm_loadu_ps(buffer + i);
                        _mm_storeu_ps(buffer + i, _mm_mul_ps(v, g));
                    }
#endif
                    for (; i < frame_count; ++i)
                        buffer[i] *= gain;
                }

                /**
                 * @brief Scales a buffer by a gain ramped linearly across the block.
                 *
                 * The zipper-free way to change a level: instead of a per-block step that
                 * clicks, the gain sweeps from @p start_gain to @p end_gain sample by
                 * sample. Kept scalar — the per-sample increment makes a lane-parallel
                 * form more trouble than it is worth for the once-per-block cost.
                 *
                 * @param buffer      The samples to scale in place.
                 * @param frame_count Number of samples.
                 * @param start_gain  Gain applied to the first sample.
                 * @param end_gain    Gain reached at the last sample.
                 */
                inline void apply_gain_ramp(float* buffer, int frame_count,
                                            float start_gain, float end_gain) noexcept
                {
                    if (frame_count <= 0)
                        return;
                    const float step = (frame_count > 1)
                                           ? (end_gain - start_gain) / static_cast<float>(frame_count - 1)
                                           : 0.0f;
                    float g = start_gain;
                    for (int i = 0; i < frame_count; ++i)
                    {
                        buffer[i] *= g;
                        g += step;
                    }
                }

                /**
                 * @brief Accumulates a scaled source into a destination: `dst[i] += src[i] * gain`.
                 *
                 * The mixer primitive — summing a voice or bus into its parent at a send
                 * level. @p dst and @p src must not overlap.
                 *
                 * @param dst         The accumulator buffer.
                 * @param src         The source buffer.
                 * @param frame_count Number of samples.
                 * @param gain        The source multiplier before accumulation.
                 */
                inline void mix_accumulate(float* dst, const float* src, int frame_count,
                                           float gain) noexcept
                {
                    int i = 0;
#if defined(SUSHIENGINE_AUDIO_SIMD_SSE)
                    const __m128 g = _mm_set1_ps(gain);
                    for (; i + 4 <= frame_count; i += 4)
                    {
                        __m128 d = _mm_loadu_ps(dst + i);
                        __m128 s = _mm_loadu_ps(src + i);
                        _mm_storeu_ps(dst + i, _mm_add_ps(d, _mm_mul_ps(s, g)));
                    }
#endif
                    for (; i < frame_count; ++i)
                        dst[i] += src[i] * gain;
                }

                /**
                 * @brief Writes a scaled copy: `dst[i] = src[i] * gain`. @p dst and @p src may not overlap.
                 * @param dst         The destination buffer.
                 * @param src         The source buffer.
                 * @param frame_count Number of samples.
                 * @param gain        The multiplier.
                 */
                inline void copy_scaled(float* dst, const float* src, int frame_count,
                                        float gain) noexcept
                {
                    int i = 0;
#if defined(SUSHIENGINE_AUDIO_SIMD_SSE)
                    const __m128 g = _mm_set1_ps(gain);
                    for (; i + 4 <= frame_count; i += 4)
                    {
                        __m128 s = _mm_loadu_ps(src + i);
                        _mm_storeu_ps(dst + i, _mm_mul_ps(s, g));
                    }
#endif
                    for (; i < frame_count; ++i)
                        dst[i] = src[i] * gain;
                }

                /** @brief Fills a buffer with a constant. */
                inline void fill(float* buffer, int frame_count, float value) noexcept
                {
                    int i = 0;
#if defined(SUSHIENGINE_AUDIO_SIMD_SSE)
                    const __m128 v = _mm_set1_ps(value);
                    for (; i + 4 <= frame_count; i += 4)
                        _mm_storeu_ps(buffer + i, v);
#endif
                    for (; i < frame_count; ++i)
                        buffer[i] = value;
                }

                /**
                 * @brief Constant-power stereo pan gains for a mono source.
                 *
                 * Uses the sine/cosine (−3 dB centre) law: as @p pan sweeps the summed
                 * *power* `left² + right²` stays 1, so a source keeps a constant
                 * loudness as it moves across the image — the standard game pan law.
                 *
                 * @param pan       Pan position in [-1, 1] (−1 hard left, 0 centre, +1 hard right).
                 * @param out_left  Set to the left-channel gain.
                 * @param out_right Set to the right-channel gain.
                 */
                inline void equal_power_pan(float pan, float& out_left, float& out_right) noexcept
                {
                    if (pan < -1.0f)
                        pan = -1.0f;
                    else if (pan > 1.0f)
                        pan = 1.0f;
                    // Map [-1,1] -> angle [0, pi/2]; left = cos, right = sin. At centre
                    // (pan 0) the angle is pi/4, so both gains are 1/sqrt(2).
                    const float half_pi = 1.57079632679489661923f;
                    const float angle = (pan + 1.0f) * 0.5f * half_pi;
                    out_left = std::cos(angle);
                    out_right = std::sin(angle);
                }
            } // namespace Simd
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
