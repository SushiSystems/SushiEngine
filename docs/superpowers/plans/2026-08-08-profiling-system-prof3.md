# Profiling System PROF3 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** The editor-owned system metrics provider — CPU utilization, process and system memory,
and the GPU section via a dynamically loaded NVML — wired into the Profiler's System and Memory
sections with a derived GPU-busy figure, per `docs/design/profiling_system.md` §6.

**Architecture:** An `ISystemMetricsProvider` interface with a `WindowsSystemMetricsProvider`
implementation lives in the editor (`applications/editor/source/system/`), never the engine.
Windows facts come from `GetSystemTimes`, `GetProcessMemoryInfo` and `GlobalMemoryStatusEx`;
the GPU section loads `nvml.dll` (shipped by the driver) with `LoadLibrary`/`GetProcAddress`
and reports unavailable when the library or a device is absent. The provider throttles itself
to ~500 ms; `main()` polls every frame and copies the snapshot into `EditorContext`.

**Tech Stack:** C++17, Win32 (kernel32/psapi), NVML by dynamic load only (no header, no import
library), Dear ImGui.

## Global Constraints

- **The project owner runs every build.** Never run `se`, cmake, ninja, or a compiler. There is
  no test target for editor code; acceptance is the owner's editor run against Task Manager.
- **A parallel agent shares this tree.** Stage by exact path; never `git add -A`.
- C++17, Allman braces, nested namespaces written out, snake_case functions/variables, members
  trailing underscore, UPPER_SNAKE constants, no abbreviations in identifiers ("utilization",
  not "util"), acronyms upper-case (`GPU`, `CPU`, `NVML`). License header block on every new
  file (copy from `applications/editor/source/core/panel_state.hpp` lines 1-22, file name
  changed, box aligned). No separator comments. Doxygen on public declarations.
- Commits `type(scope): lowercase imperative`, past-tense body bullets, ending with
  `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Changelog bullet ≤240 chars under `## [Unreleased]` / `### Added`.
- Honesty rules (spec §8): every displayed value is a measurement, a figure labeled derived, or
  "n/a". The NVML section absent → "n/a" rows, never zeros. The derived GPU-busy figure is
  always labeled derived.
- **NVML is loaded dynamically and its header is NOT vendored**: the implementation declares
  its own minimal typedefs and function-pointer types (exact shapes given in Task 1). Never
  add an include of `nvml.h` or a link dependency on any NVIDIA library.

---

### Task 1: The provider — interface, Windows implementation, NVML dynamic load

**Files:**
- Create: `applications/editor/source/system/system_metrics.hpp` (interface + snapshot)
- Create: `applications/editor/source/system/windows_system_metrics.hpp`
- Create: `applications/editor/source/system/windows_system_metrics.cpp`
- Modify: `applications/editor/CMakeLists.txt` (sources beside line 59's list; `Psapi` beside
  the existing `Shell32` link at line 128)

**Interfaces:**
- Produces (Task 2 consumes verbatim):

```cpp
namespace SushiEngine
{
    namespace Editor
    {
        /** @brief One poll's operating-system and hardware readings. */
        struct SystemMetricsSnapshot
        {
            bool cpu_valid = false;             /**< False until two samples exist. */
            float cpu_utilization_percent = 0.0f; /**< Whole-system busy share. */
            std::uint64_t process_working_set_bytes = 0;
            std::uint64_t system_memory_used_bytes = 0;
            std::uint64_t system_memory_total_bytes = 0;
            bool gpu_valid = false; /**< True only while NVML answers for a device. */
            float gpu_utilization_percent = 0.0f;
            std::uint64_t gpu_memory_used_bytes = 0;
            std::uint64_t gpu_memory_total_bytes = 0;
            float gpu_temperature_celsius = 0.0f;
        };

        /** @brief The seam the editor reads host metrics through; one per platform. */
        class ISystemMetricsProvider
        {
            public:
                virtual ~ISystemMetricsProvider() = default;

                /**
                 * @brief Refreshes the snapshot when the provider's interval elapsed.
                 *
                 * Called every frame; the provider throttles itself, so most calls
                 * return without measuring.
                 */
                virtual void poll() = 0;

                /** @brief The most recent readings; validity flags say what answered. */
                virtual const SystemMetricsSnapshot& snapshot() const noexcept = 0;
        };
    } // namespace Editor
} // namespace SushiEngine
```

- [ ] **Step 1: Write the interface header** (`system_metrics.hpp`) exactly as above, license
  header first, includes `<cstdint>` only.

- [ ] **Step 2: Write the Windows provider header** (`windows_system_metrics.hpp`):

```cpp
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
        };
    } // namespace Editor
} // namespace SushiEngine
```

- [ ] **Step 3: Write the implementation** (`windows_system_metrics.cpp`). The complete
  mechanism — keep the shapes, spell Win32 exactly as here:

```cpp
#include "windows_system_metrics.hpp"

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

            NVMLShutdown nvml_shutdown = nullptr;
            NVMLDeviceGetUtilizationRates nvml_utilization = nullptr;
            NVMLDeviceGetMemoryInfo nvml_memory = nullptr;
            NVMLDeviceGetTemperature nvml_temperature = nullptr;

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
            nvml_shutdown =
                reinterpret_cast<NVMLShutdown>(GetProcAddress(module, "nvmlShutdown"));
            nvml_utilization = reinterpret_cast<NVMLDeviceGetUtilizationRates>(
                GetProcAddress(module, "nvmlDeviceGetUtilizationRates"));
            nvml_memory = reinterpret_cast<NVMLDeviceGetMemoryInfo>(
                GetProcAddress(module, "nvmlDeviceGetMemoryInfo"));
            nvml_temperature = reinterpret_cast<NVMLDeviceGetTemperature>(
                GetProcAddress(module, "nvmlDeviceGetTemperature"));
            if (init == nullptr || by_index == nullptr || nvml_shutdown == nullptr ||
                nvml_utilization == nullptr || nvml_memory == nullptr ||
                nvml_temperature == nullptr || init() != 0 ||
                by_index(0, &nvml_device_) != 0)
            {
                FreeLibrary(module);
                nvml_device_ = nullptr;
                return;
            }
            nvml_module_ = module;
        }

        WindowsSystemMetricsProvider::~WindowsSystemMetricsProvider()
        {
            if (nvml_module_ != nullptr)
            {
                nvml_shutdown();
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
                NVMLUtilization utilization{};
                NVMLMemory memory{};
                unsigned int temperature = 0;
                if (nvml_utilization(nvml_device_, &utilization) == 0 &&
                    nvml_memory(nvml_device_, &memory) == 0 &&
                    nvml_temperature(nvml_device_, NVML_TEMPERATURE_GPU,
                                     &temperature) == 0)
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
    } // namespace Editor
} // namespace SushiEngine
```

- [ ] **Step 4: Register in the build.** Add `source/system/windows_system_metrics.cpp` to the
  editor's source list; extend the Windows-only link line (the `Shell32` one at
  `applications/editor/CMakeLists.txt:128`) with `Psapi`. Note the source file is Windows-only
  code in a currently Windows-only editor target — if the CMake gates sources by platform
  anywhere, follow that gate; otherwise the list is fine as is (read the file and match).

- [ ] **Step 5: Commit**

```bash
git add applications/editor/source/system/system_metrics.hpp \
        applications/editor/source/system/windows_system_metrics.hpp \
        applications/editor/source/system/windows_system_metrics.cpp \
        applications/editor/CMakeLists.txt
git commit -m "feat(editor): add the Windows system metrics provider (PROF3)"
```

---

### Task 2: Wire the System and Memory rows, and the derived GPU-busy figure

**Files:**
- Modify: `applications/editor/source/core/editor_context.hpp`
- Modify: `applications/editor/source/main.cpp`
- Modify: `applications/editor/source/ui/profiler_panel.hpp`
- Modify: `applications/editor/source/ui/profiler_panel.cpp`
- Modify: `docs/reference/changelog.md`
- Modify: `docs/design/remaining_work.md`
- Modify: `docs/design/profiling_system.md` (status line: PROF0–PROF3 built, PROF4 open)

**Interfaces:**
- Consumes: Task 1's `ISystemMetricsProvider` / `SystemMetricsSnapshot` verbatim; the Profiler
  panel's held-copy pause pattern; `context.frame_profile.frame_milliseconds` and the held GPU
  pass timings (for the derived figure).
- Produces: `EditorContext::system_metrics` (`SystemMetricsSnapshot`, copied per frame).

- [ ] **Step 1: Context field.** In `editor_context.hpp`, beside `resident_texture_bytes`,
  add (include `"../system/system_metrics.hpp"` — check the include style used for other
  editor-local headers there and match it):

```cpp
            // The host's own readings, copied per frame like everything else here.
            SystemMetricsSnapshot system_metrics;
```

- [ ] **Step 2: Own and poll in main.** In `main.cpp`: construct
  `SushiEngine::Editor::WindowsSystemMetricsProvider system_metrics_provider;` beside the
  other main-owned services (before the loop); in the loop, near the other per-frame copies:
  `system_metrics_provider.poll();` then
  `context.system_metrics = system_metrics_provider.snapshot();`.

- [ ] **Step 3: Panel wiring.** `profiler_panel.hpp/.cpp`: add `SystemMetricsSnapshot
  held_system_metrics;` refreshed in the unpaused block with everything else. Then:
  - **Memory section**: "Process working set (MiB)" from
    `held_system_metrics.process_working_set_bytes` (available when > 0), "System memory used
    (MiB)" as `used / total` from the snapshot (available when total > 0). Delete those rows'
    remaining hardcoded "n/a" placeholders.
  - **System section**: CPU row available on `cpu_valid`; GPU utilization, GPU memory
    used/total (MiB) and temperature rows available on `gpu_valid`; when `gpu_valid` is
    false, keep those rows "n/a".
  - **Derived GPU busy**: compute from held data — the Scene viewport's summed GPU pass
    milliseconds (`held_gpu_statistics`) divided by `held.frame_milliseconds`, × 100,
    clamped to [0, 100]; available when both terms are > 0. Label the row exactly
    "GPU busy, derived (%)" — the derived label is a spec §8 requirement. Show it always
    (it is the fallback that works without NVML), beside the NVML utilization row.
- [ ] **Step 4: Docs.** Changelog bullet (System/Memory rows live, NVML optional).
  `remaining_work.md` PROF row → PROF4 only. `profiling_system.md` line 3 → PROF0–PROF3
  built, PROF4 open. Self-check with the two documentation checker scripts (allowed lints);
  no new failures.

- [ ] **Step 5: Owner acceptance (spec §9 PROF3 row).** Ask the owner to run `se build` +
  `se editor` and compare: CPU% and memory against Task Manager (within a few percent);
  GPU%/temperature against any monitor they trust; then rename `nvml.dll` is NOT required —
  instead they can confirm the degraded path by the plan's design review (renaming a driver
  DLL is not something to ask of a live machine; note this deviation from the spec's
  "unplugging the vendor path" clause and record that the fallback path's proof is the
  gpu_valid gating read in review).

- [ ] **Step 6: Commit**

```bash
git add applications/editor/source/core/editor_context.hpp \
        applications/editor/source/main.cpp \
        applications/editor/source/ui/profiler_panel.hpp \
        applications/editor/source/ui/profiler_panel.cpp \
        docs/reference/changelog.md docs/design/remaining_work.md \
        docs/design/profiling_system.md
git commit -m "feat(editor): wire the system metrics into the Profiler panel (PROF3)"
```

---

## After this plan

Only PROF4 (polish + the architecture chapter) remains, small enough to run without a separate
plan document if the owner agrees at the time.
