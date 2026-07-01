/**************************************************************************/
/* device.hpp                                                            */
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
 * @file device.hpp
 * @brief The graphics-device abstraction the renderer is written against.
 *
 * This is the dependency-inversion seam: the engine and editor program against
 * IRenderDevice and the create_render_device() factory, never against Vulkan.
 * The only backend today is Vulkan (render/rhi/vulkan/), but the abstract surface
 * carries no Vulkan types, so a D3D12/Metal backend can be added without touching
 * a consumer. DeviceInfo exposes the device UUID from the start because that is
 * the key a later milestone matches against SushiRuntime's SYCL device to share
 * memory without a copy.
 */

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace SushiEngine
{
    namespace render
    {
        /** @brief Which physical GPU the backend should prefer when several exist. */
        enum class DevicePreference
        {
            HighPerformance, /**< Prefer a discrete GPU (default). */
            LowPower         /**< Prefer an integrated GPU. */
        };

        /**
         * @brief Parameters for bringing up a graphics device.
         *
         * Kept backend-neutral: a Vulkan or future D3D12 backend reads the same
         * request. @ref required_uuid is the interop hook — when set, the backend
         * must pick the physical device with this UUID (the one SushiRuntime runs
         * on) so their allocations can be shared; left null, the backend picks by
         * @ref preference.
         */
        struct RenderDeviceDesc
        {
            bool enable_validation = false;
            DevicePreference preference = DevicePreference::HighPerformance;
            const std::array<std::uint8_t, 16>* required_uuid = nullptr;
        };

        /**
         * @brief Identity of the selected physical device.
         *
         * @ref uuid is a stable 16-byte device identifier (Vulkan's deviceUUID),
         * comparable across APIs, and is what interop matches on.
         */
        struct DeviceInfo
        {
            std::string name;
            std::array<std::uint8_t, 16> uuid{};
            bool is_discrete = false;
            std::uint32_t api_version = 0;
        };

        /**
         * @brief A live graphics device: the root object every render resource hangs off.
         *
         * Owns the backend instance, the logical device, and its allocator; releasing
         * it tears those down. This milestone exposes only identity — resource and
         * command surfaces are added on top as the renderer grows.
         */
        class IRenderDevice
        {
            public:
                virtual ~IRenderDevice() = default;

                /**
                 * @brief The selected physical device's identity.
                 * @return Name, UUID, and capability summary of the device in use.
                 */
                virtual const DeviceInfo& info() const noexcept = 0;
        };

        /**
         * @brief Creates the default (Vulkan) graphics device.
         *
         * The one place a consumer names a backend; everything downstream is the
         * abstract IRenderDevice. Throws std::runtime_error if no suitable device is
         * found or device creation fails.
         *
         * @param desc Device-selection request (validation, preference, interop UUID).
         * @return An owning handle to the live device.
         */
        std::unique_ptr<IRenderDevice> create_render_device(const RenderDeviceDesc& desc);
    } // namespace render
} // namespace SushiEngine
