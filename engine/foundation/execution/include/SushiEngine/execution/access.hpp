/**************************************************************************/
/* access.hpp                                                             */
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

#include <cstdint>

namespace SushiEngine
{
    namespace Execution
    {
        /**
         * @brief How a node touches a resource — the one access algebra every domain declares in.
         *
         * The simulation domain (thread pool or SYCL) and the render domain describe
         * their hazards in the same enum so a single tracker semantic serves both: a
         * simulation backend consumes only the read/write projection, while the render
         * graph and a future compute-shader backend consume the typed values and lower
         * them to barriers. The render-only values exist here rather than in a parallel
         * render enum precisely so the two vocabularies cannot drift apart; they are
         * rejected where they are meaningless (see intent_is_render_only).
         */
        enum class AccessIntent : std::uint8_t
        {
            HostRead,             /**< Read by a host job. */
            HostWrite,            /**< Written by a host job. */
            ComputeRead,          /**< Read by a device-compute kernel. */
            ComputeWrite,         /**< Written by a device-compute kernel. */
            TransferSource,       /**< Read by a copy, either engine. */
            TransferDestination,  /**< Written by a copy, either engine. */
            SampledRead,          /**< Sampled by a shader stage. */
            StorageRead,          /**< Read as a storage image or storage buffer. */
            StorageWrite,         /**< Written as a storage image or storage buffer. */
            ColorWrite,           /**< Written as a colour attachment. */
            DepthStencilRead,     /**< Read as a depth/stencil attachment. */
            DepthStencilWrite,    /**< Written as a depth/stencil attachment. */
            IndirectRead,         /**< Consumed as indirect draw or dispatch arguments. */
            VertexRead,           /**< Consumed as vertex input. */
            IndexRead,            /**< Consumed as index input. */
            Present,              /**< Handed to the presentation engine. */
        };

        /**
         * @brief Reports whether an intent modifies the resource it names.
         *
         * The projection every tracker reduces the algebra to: hazard classification
         * needs nothing finer than "does this access write", so a backend that models
         * no GPU pipeline at all still orders the same pairs as one that does.
         *
         * @param intent The access to classify.
         * @return True when the access may modify the resource's contents.
         */
        constexpr bool intent_writes(AccessIntent intent) noexcept
        {
            return intent == AccessIntent::HostWrite ||
                   intent == AccessIntent::ComputeWrite ||
                   intent == AccessIntent::TransferDestination ||
                   intent == AccessIntent::StorageWrite ||
                   intent == AccessIntent::ColorWrite ||
                   intent == AccessIntent::DepthStencilWrite;
        }

        /**
         * @brief Reports whether an intent is issued by host code rather than a device.
         *
         * Drives visibility decisions at the domain boundary: a host access needs the
         * memory mapped and coherent on the CPU, a device access does not.
         *
         * @param intent The access to classify.
         * @return True for host-side accesses, false for device-side and fixed-function ones.
         */
        constexpr bool intent_is_host(AccessIntent intent) noexcept
        {
            return intent == AccessIntent::HostRead || intent == AccessIntent::HostWrite;
        }

        /**
         * @brief Reports whether an intent only means something inside the render domain.
         *
         * The handoff registry rejects these: cross-domain traffic is buffer-shaped and
         * flows between host and compute accesses, so an attachment or presentation
         * intent on a shared entry is a declaration error the build should catch rather
         * than a state a backend has to invent a meaning for.
         *
         * @param intent The access to classify.
         * @return True when only a rendering backend can realize the access.
         */
        constexpr bool intent_is_render_only(AccessIntent intent) noexcept
        {
            return intent == AccessIntent::SampledRead ||
                   intent == AccessIntent::StorageRead ||
                   intent == AccessIntent::StorageWrite ||
                   intent == AccessIntent::ColorWrite ||
                   intent == AccessIntent::DepthStencilRead ||
                   intent == AccessIntent::DepthStencilWrite ||
                   intent == AccessIntent::IndirectRead ||
                   intent == AccessIntent::VertexRead ||
                   intent == AccessIntent::IndexRead ||
                   intent == AccessIntent::Present;
        }

        /**
         * @brief What a node's output promises under replay.
         *
         * Rollback correctness requires the simulation to reproduce a tick bit for bit;
         * rendering must neither pay for that nor be able to contaminate it. Stamping
         * the promise on the node makes the rule mechanical instead of conventional:
         * a bitwise node reading cosmetic output is a declaration error, checkable at
         * graph build.
         */
        enum class DeterminismClass : std::uint8_t
        {
            Bitwise,   /**< Bit-identical under replay on the same binary and device. */
            Tolerant,  /**< Comparable across backends within a stated tolerance. */
            Cosmetic,  /**< No replay obligation; must never feed a bitwise node. */
        };
    } // namespace Execution
} // namespace SushiEngine
