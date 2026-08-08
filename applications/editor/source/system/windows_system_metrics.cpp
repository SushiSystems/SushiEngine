/**************************************************************************/
/* windows_system_metrics.cpp                                            */
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

#include "windows_system_metrics.hpp"

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <psapi.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            // The provider measures at most twice a second: these numbers move slowly
            // and the NVML queries are not free.
            constexpr std::chrono::milliseconds MEASURE_INTERVAL{500};

            // NVML, declared locally: the header is not vendored and the library is
            // loaded from the driver at run time, so these mirror the stable C ABI.
            using NVMLReturn = int; // 0 == NVML_SUCCESS
            struct NVMLUtilization
            {
                unsigned int gpu;
                unsigned int memory;
            };
            struct NVMLMemory
            {
                unsigned long long total;
                unsigned long long free;
                unsigned long long used;
            };
            constexpr int NVML_TEMPERATURE_GPU = 0;

            using NVMLInit = NVMLReturn (*)();
            using NVMLShutdown = NVMLReturn (*)();
            using NVMLDeviceGetHandleByIndex = NVMLReturn (*)(unsigned int, void**);
            using NVMLDeviceGetUtilizationRates = NVMLReturn (*)(void*, NVMLUtilization*);
            using NVMLDeviceGetMemoryInfo = NVMLReturn (*)(void*, NVMLMemory*);
            using NVMLDeviceGetTemperature = NVMLReturn (*)(void*, int, unsigned int*);

            std::uint64_t to_uint64(const FILETIME& time)
            {
                ULARGE_INTEGER value;
                value.LowPart = time.dwLowDateTime;
                value.HighPart = time.dwHighDateTime;
                return value.QuadPart;
            }
        } // namespace

        WindowsSystemMetricsProvider::WindowsSystemMetricsProvider()
            : last_measure_(std::chrono::steady_clock::now() - MEASURE_INTERVAL)
        {
            // The driver installs nvml.dll into System32; a machine without the
            // NVIDIA driver simply fails the load and the GPU section stays invalid.
            HMODULE module = LoadLibraryW(L"nvml.dll");
            if (module == nullptr)
                return;

            const auto init =
                reinterpret_cast<NVMLInit>(GetProcAddress(module, "nvmlInit_v2"));
            const auto by_index = reinterpret_cast<NVMLDeviceGetHandleByIndex>(
                GetProcAddress(module, "nvmlDeviceGetHandleByIndex_v2"));
            const auto shutdown =
                reinterpret_cast<NVMLShutdown>(GetProcAddress(module, "nvmlShutdown"));
            const auto utilization = reinterpret_cast<NVMLDeviceGetUtilizationRates>(
                GetProcAddress(module, "nvmlDeviceGetUtilizationRates"));
            const auto memory = reinterpret_cast<NVMLDeviceGetMemoryInfo>(
                GetProcAddress(module, "nvmlDeviceGetMemoryInfo"));
            const auto temperature = reinterpret_cast<NVMLDeviceGetTemperature>(
                GetProcAddress(module, "nvmlDeviceGetTemperature"));
            if (init == nullptr || by_index == nullptr || shutdown == nullptr ||
                utilization == nullptr || memory == nullptr || temperature == nullptr)
            {
                // Nothing was initialized yet, so there is nothing for nvmlShutdown to
                // undo: just release the module.
                FreeLibrary(module);
                return;
            }

            if (init() != 0)
            {
                // nvmlInit_v2 itself did not succeed, so calling nvmlShutdown would be
                // undoing state that was never entered.
                FreeLibrary(module);
                return;
            }

            void* device = nullptr;
            if (by_index(0, &device) != 0)
            {
                // Init succeeded before this failed, so it must be undone before the
                // module unloads or the driver-side state it created leaks.
                shutdown();
                FreeLibrary(module);
                return;
            }

            nvml_module_ = module;
            nvml_device_ = device;
            nvml_shutdown_ = reinterpret_cast<void*>(shutdown);
            nvml_utilization_ = reinterpret_cast<void*>(utilization);
            nvml_memory_ = reinterpret_cast<void*>(memory);
            nvml_temperature_ = reinterpret_cast<void*>(temperature);
        }

        WindowsSystemMetricsProvider::~WindowsSystemMetricsProvider()
        {
            if (nvml_module_ != nullptr)
            {
                reinterpret_cast<NVMLShutdown>(nvml_shutdown_)();
                FreeLibrary(static_cast<HMODULE>(nvml_module_));
            }
        }

        void WindowsSystemMetricsProvider::poll()
        {
            const std::chrono::steady_clock::time_point now =
                std::chrono::steady_clock::now();
            if (now - last_measure_ < MEASURE_INTERVAL)
                return;
            last_measure_ = now;
            measure();
        }

        const SystemMetricsSnapshot& WindowsSystemMetricsProvider::snapshot() const noexcept
        {
            return snapshot_;
        }

        void WindowsSystemMetricsProvider::measure()
        {
            FILETIME idle_time;
            FILETIME kernel_time;
            FILETIME user_time;
            if (GetSystemTimes(&idle_time, &kernel_time, &user_time) != 0)
            {
                const std::uint64_t idle = to_uint64(idle_time);
                // Kernel time includes idle, so busy = (kernel - idle) + user.
                const std::uint64_t kernel = to_uint64(kernel_time);
                const std::uint64_t user = to_uint64(user_time);
                if (measured_once_)
                {
                    const std::uint64_t idle_delta = idle - previous_idle_;
                    const std::uint64_t total_delta =
                        (kernel - previous_kernel_) + (user - previous_user_);
                    if (total_delta > 0)
                    {
                        snapshot_.cpu_utilization_percent =
                            100.0f * (1.0f - static_cast<float>(idle_delta) /
                                                 static_cast<float>(total_delta));
                        snapshot_.cpu_valid = true;
                    }
                }
                previous_idle_ = idle;
                previous_kernel_ = kernel;
                previous_user_ = user;
                measured_once_ = true;
            }

            PROCESS_MEMORY_COUNTERS process_memory{};
            process_memory.cb = sizeof(process_memory);
            if (GetProcessMemoryInfo(GetCurrentProcess(), &process_memory,
                                     sizeof(process_memory)) != 0)
                snapshot_.process_working_set_bytes = process_memory.WorkingSetSize;

            MEMORYSTATUSEX memory_status{};
            memory_status.dwLength = sizeof(memory_status);
            if (GlobalMemoryStatusEx(&memory_status) != 0)
            {
                snapshot_.system_memory_total_bytes = memory_status.ullTotalPhys;
                snapshot_.system_memory_used_bytes =
                    memory_status.ullTotalPhys - memory_status.ullAvailPhys;
            }

            snapshot_.gpu_valid = false;
            if (nvml_module_ != nullptr && nvml_device_ != nullptr)
            {
                const auto utilization_fn =
                    reinterpret_cast<NVMLDeviceGetUtilizationRates>(nvml_utilization_);
                const auto memory_fn =
                    reinterpret_cast<NVMLDeviceGetMemoryInfo>(nvml_memory_);
                const auto temperature_fn =
                    reinterpret_cast<NVMLDeviceGetTemperature>(nvml_temperature_);

                NVMLUtilization utilization{};
                NVMLMemory memory{};
                unsigned int temperature = 0;
                if (utilization_fn(nvml_device_, &utilization) == 0 &&
                    memory_fn(nvml_device_, &memory) == 0 &&
                    temperature_fn(nvml_device_, NVML_TEMPERATURE_GPU, &temperature) == 0)
                {
                    snapshot_.gpu_utilization_percent =
                        static_cast<float>(utilization.gpu);
                    snapshot_.gpu_memory_used_bytes = memory.used;
                    snapshot_.gpu_memory_total_bytes = memory.total;
                    snapshot_.gpu_temperature_celsius =
                        static_cast<float>(temperature);
                    snapshot_.gpu_valid = true;
                }
            }
        }

        std::unique_ptr<ISystemMetricsProvider> create_system_metrics_provider()
        {
            return std::make_unique<WindowsSystemMetricsProvider>();
        }
    } // namespace Editor
} // namespace SushiEngine

#else // !_WIN32

// This target is Windows-only today (CLAUDE.md), but the source list that builds it
// is not platform-gated in CMake — the project's convention (see
// applications/editor/source/project/project_panel.cpp) is to keep such files in the
// unconditional list and guard the platform-specific body in-file instead, so a
// non-Windows configure of this target still compiles. There is nothing to measure
// off Windows, so every member simply reports the default-invalid snapshot.
namespace SushiEngine
{
    namespace Editor
    {
        WindowsSystemMetricsProvider::WindowsSystemMetricsProvider()
            : last_measure_(std::chrono::steady_clock::now())
        {
        }

        WindowsSystemMetricsProvider::~WindowsSystemMetricsProvider()
        {
        }

        void WindowsSystemMetricsProvider::poll()
        {
        }

        const SystemMetricsSnapshot& WindowsSystemMetricsProvider::snapshot() const noexcept
        {
            return snapshot_;
        }

        void WindowsSystemMetricsProvider::measure()
        {
        }

        std::unique_ptr<ISystemMetricsProvider> create_system_metrics_provider()
        {
            return std::make_unique<WindowsSystemMetricsProvider>();
        }
    } // namespace Editor
} // namespace SushiEngine

#endif // _WIN32
