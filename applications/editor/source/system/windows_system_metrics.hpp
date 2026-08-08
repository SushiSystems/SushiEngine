/**************************************************************************/
/* windows_system_metrics.hpp                                            */
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

#include "system_metrics.hpp"

#include <chrono>
#include <cstdint>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Win32 + NVML metrics: processor times, process and system memory,
         *        and the GPU when the driver's NVML library is present.
         *
         * NVML is loaded dynamically at construction and never linked: the library
         * ships with the NVIDIA driver, so its absence is a normal configuration and
         * the GPU section simply reports invalid. All Win32 handles are process-wide;
         * the class owns only the NVML module handle.
         */
        class WindowsSystemMetricsProvider final : public ISystemMetricsProvider
        {
            public:
                WindowsSystemMetricsProvider();
                ~WindowsSystemMetricsProvider() override;

                WindowsSystemMetricsProvider(const WindowsSystemMetricsProvider&) = delete;
                WindowsSystemMetricsProvider&
                operator=(const WindowsSystemMetricsProvider&) = delete;

                void poll() override;
                const SystemMetricsSnapshot& snapshot() const noexcept override;

            private:
                void measure();

                SystemMetricsSnapshot snapshot_;
                std::chrono::steady_clock::time_point last_measure_;
                bool measured_once_ = false;
                std::uint64_t previous_idle_ = 0;
                std::uint64_t previous_kernel_ = 0;
                std::uint64_t previous_user_ = 0;
                void* nvml_module_ = nullptr; /**< HMODULE, kept as void* out of windows.h. */
                void* nvml_device_ = nullptr; /**< nvmlDevice_t of adapter 0, when valid. */

                // The four NVML entry points resolved at construction, kept per-instance
                // (not anonymous-namespace statics) since nvml_module_/nvml_device_ already
                // are; stored as void* to keep windows.h and the NVML ABI mirrors out of
                // this header, cast back to their function-pointer types at the call site.
                void* nvml_shutdown_ = nullptr;    /**< NVMLShutdown, cast at use. */
                void* nvml_utilization_ = nullptr; /**< NVMLDeviceGetUtilizationRates, cast at use. */
                void* nvml_memory_ = nullptr;      /**< NVMLDeviceGetMemoryInfo, cast at use. */
                void* nvml_temperature_ = nullptr; /**< NVMLDeviceGetTemperature, cast at use. */
        };
    } // namespace Editor
} // namespace SushiEngine
