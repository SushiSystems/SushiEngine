/**************************************************************************/
/* memory.hpp                                                             */
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

#include <cstddef>
#include <cstdint>

namespace SushiEngine
{
    namespace Execution
    {
        /**
         * @brief Who must be able to address an allocation.
         *
         * A request, not a placement: each backend satisfies it with whatever its
         * platform actually offers — shared unified memory on a device runtime, plain
         * aligned host memory where there is no device, and graphics-visible memory
         * supplied by the render backend on unified-memory hardware. Stating the
         * requirement rather than the mechanism is what lets one declaration keep the
         * zero-copy property on platforms whose memory models have nothing in common.
         */
        enum class MemoryVisibility : std::uint8_t
        {
            HostShared,     /**< Addressable by host code and by device work alike. */
            DeviceResident, /**< Device-side only; the host reaches it through transfers. */
        };

        /**
         * @brief Which device an allocation is pinned to.
         *
         * A named value rather than a bare integer so a device selector cannot be
         * mistaken for an element count at a call site. Only meaningful up to the
         * backend's reported device count; a single-device backend accepts index zero
         * and nothing else, rather than silently ignoring the argument.
         */
        struct DeviceIndex
        {
            std::uint32_t value = 0; /**< Zero-based device ordinal. */
        };

        /**
         * @brief What the selected execution backend can actually do.
         *
         * Queried rather than assumed, so a subsystem that wants a backend-specific
         * capability asks for it instead of a portable seam pretending to offer
         * something one implementation cannot deliver. The pattern matches the engine's
         * existing feature queries (`IDSPAccelerator::available()`, the renderer's
         * capability flags) rather than inventing a second convention.
         */
        struct BackendCapabilities
        {
            /** @brief Devices an allocation may target; one on a host-only backend. */
            std::size_t device_count = 1;

            /**
             * @brief Whether DeviceResident is a distinct placement from HostShared.
             *
             * False on a backend with no device at all, where both requests are served
             * by ordinary host memory and a transfer is a copy the caller need not make.
             */
            bool device_resident_memory = false;
        };
    } // namespace Execution
} // namespace SushiEngine
