/**************************************************************************/
/* accelerator.hpp                                                        */
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

#ifndef SUSHIENGINE_AUDIO_ACCELERATOR_HPP
#define SUSHIENGINE_AUDIO_ACCELERATOR_HPP

/**
 * @file accelerator.hpp
 * @brief The optional GPU batch-DSP seam — declared now, implemented later.
 *
 * The real-time mix runs entirely on the CPU (on the device callback thread) and
 * deliberately sidesteps SushiRuntime, which is a block-until-quiescent throughput
 * engine with no real-time thread class (see the `sushiruntime-realtime-gaps`
 * audit). The *only* place the runtime enters the audio subsystem is here: an
 * optional path that offloads **batch**, latency-tolerant DSP — long convolution,
 * HRTF, ambisonic decode, aeroacoustic field evaluation — onto the SYCL task graph
 * with k-block lookahead, so the RT thread never waits on the GPU.
 *
 * This interface is defined from the start so the architecture stays clean and the
 * CPU mix is written against a seam rather than a concrete runtime call. The seam itself
 * stays intentionally thin — the sole query is @ref available — because the runtime's
 * fluent API is unstable and must not leak into the SDL/SYCL-free header tree. The concrete
 * S10 implementation, @ref SYCLDSPAccelerator in `accelerator_sycl.hpp` (a SYCL-only header),
 * carries the batch-submit surface (upload/submit/collect) and confines all SushiRuntime
 * coupling to that one file; it reports @ref available true once its device queue is usable
 * and offloads batch FIR convolution with k-block lookahead. The CPU path remains the
 * default everywhere no accelerator is wired.
 *
 * See `docs/slop/audio_system.md` §2, §12.2, §13.
 */

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief The optional GPU batch-DSP offload seam (placeholder until S10).
         *
         * A subsystem that could benefit from GPU offload asks @ref available and
         * falls back to its CPU path when it returns false, which is the case for
         * every build until the SushiRuntime-backed implementation exists. Keeping
         * the query behind this interface means the mix never names the runtime
         * directly and the offload can be switched on later without touching call
         * sites.
         */
        class IDSPAccelerator
        {
            public:
                virtual ~IDSPAccelerator() = default;

                /**
                 * @brief Whether GPU batch offload is available in this build/session.
                 * @return True only once a real accelerator (S10) is wired and its
                 *         device is usable; false everywhere on the CPU-only path, so
                 *         callers stay on their CPU implementation.
                 */
                virtual bool available() const noexcept = 0;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
