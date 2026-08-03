/**************************************************************************/
/* denormals.hpp                                                         */
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

#ifndef SUSHIENGINE_AUDIO_DSP_DENORMALS_HPP
#define SUSHIENGINE_AUDIO_DSP_DENORMALS_HPP

/**
 * @file denormals.hpp
 * @brief The denormal guard: flush-to-zero for the whole audio callback.
 *
 * The moment an IIR filter or a reverb tail exists, its state decays toward — but
 * never reaches — zero, and once the values fall below the smallest normal float the
 * CPU switches to *denormal* (subnormal) arithmetic, which on x86 can be one to two
 * orders of magnitude slower per operation. A single decaying tail can then blow the
 * audio thread's deadline. The fix every DSP engine uses is to set the FPU to flush
 * denormals to zero for the duration of the callback; the audio thread runs outside
 * the deterministic sim island, so the tiny bit-level change is inaudible and
 * harmless (audio never writes sim state).
 *
 * @ref ScopedNoDenormals sets flush-to-zero (FTZ) and denormals-are-zero (DAZ) on
 * construction and restores the previous control word on destruction — construct one
 * at the top of the render callback and every processor beneath it runs with
 * denormals flushed.
 */

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    #define SUSHIENGINE_AUDIO_DENORMALS_X86 1
    #include <xmmintrin.h>
#elif defined(__aarch64__) || defined(_M_ARM64)
    #define SUSHIENGINE_AUDIO_DENORMALS_ARM64 1
    #include <cstdint>
#endif

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            /**
             * @brief RAII flush-to-zero scope for the FPU control word.
             *
             * On x86 it OR-sets the MXCSR FTZ (bit 15) and DAZ (bit 6) flags; on
             * ARM64 it sets the FPCR FZ flag (bit 24). On any other target it is a
             * no-op that still compiles, so portable code needs no `#ifdef`. The
             * saved control word is restored exactly, so nesting and interleaving with
             * host code that expects the default rounding mode are both safe.
             */
            class ScopedNoDenormals
            {
                public:
                    /** @brief Saves the FPU control word and enables denormal flushing. */
                    ScopedNoDenormals() noexcept
                    {
#if defined(SUSHIENGINE_AUDIO_DENORMALS_X86)
                        saved_ = _mm_getcsr();
                        // FTZ (0x8000): flush denormal results to zero.
                        // DAZ (0x0040): treat denormal inputs as zero.
                        _mm_setcsr((saved_ | 0x8000u | 0x0040u));
#elif defined(SUSHIENGINE_AUDIO_DENORMALS_ARM64)
                        std::uint64_t fpcr = 0;
                        __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
                        saved_ = fpcr;
                        __asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr | (std::uint64_t(1) << 24)));
#endif
                    }

                    /** @brief Restores the saved FPU control word. */
                    ~ScopedNoDenormals() noexcept
                    {
#if defined(SUSHIENGINE_AUDIO_DENORMALS_X86)
                        _mm_setcsr(saved_);
#elif defined(SUSHIENGINE_AUDIO_DENORMALS_ARM64)
                        __asm__ __volatile__("msr fpcr, %0" : : "r"(saved_));
#endif
                    }

                    ScopedNoDenormals(const ScopedNoDenormals&) = delete;
                    ScopedNoDenormals& operator=(const ScopedNoDenormals&) = delete;

                    /** @brief Whether this build actually flushes denormals (false on unknown targets). */
                    static constexpr bool is_supported() noexcept
                    {
#if defined(SUSHIENGINE_AUDIO_DENORMALS_X86) || defined(SUSHIENGINE_AUDIO_DENORMALS_ARM64)
                        return true;
#else
                        return false;
#endif
                    }

                private:
#if defined(SUSHIENGINE_AUDIO_DENORMALS_X86)
                    unsigned int saved_ = 0;
#elif defined(SUSHIENGINE_AUDIO_DENORMALS_ARM64)
                    std::uint64_t saved_ = 0;
#endif
            };
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
