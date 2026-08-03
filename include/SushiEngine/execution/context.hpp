/**************************************************************************/
/* context.hpp                                                            */
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
 * @file context.hpp
 * @brief Selects the execution backend and publishes it under one set of names.
 *
 * Every engine subsystem builds graphs and allocates columns through
 * `Execution::Context`, `Execution::Graph`, and `Execution::Buffer` — never through a
 * backend's own types. Which implementation those names denote is a compile-time
 * policy, chosen once by the build:
 *
 * - `SUSHIENGINE_EXECUTION_BACKEND_RUNTIME` — SushiRuntime's task graph and shared USM.
 *
 * The choice is compile-time rather than a virtual interface for a concrete reason: a
 * device backend must forward each kernel into its own launch as the original callable,
 * and a type-erased one cannot be captured into device code. One binary needs one
 * backend anyway — a device translation unit already requires that compiler for the
 * whole unit — so nothing is lost by deciding at configure time.
 *
 * A translation unit compiled without the build's definition falls back to the runtime
 * backend, which keeps a stray hand-driven compile behaving as the tree always has.
 */

#if defined(SUSHIENGINE_EXECUTION_BACKEND_RUNTIME) && defined(SUSHIENGINE_EXECUTION_BACKEND_NATIVE)
#    error "Exactly one execution backend may be selected."
#endif

#if !defined(SUSHIENGINE_EXECUTION_BACKEND_RUNTIME) && !defined(SUSHIENGINE_EXECUTION_BACKEND_NATIVE)
#    define SUSHIENGINE_EXECUTION_BACKEND_RUNTIME 1
#endif

#include <SushiEngine/execution/access.hpp>
#include <SushiEngine/execution/hazard.hpp>
#include <SushiEngine/execution/interval.hpp>
#include <SushiEngine/execution/memory.hpp>
#include <SushiEngine/execution/node_descriptor.hpp>
#include <SushiEngine/execution/run_report.hpp>

#if defined(SUSHIENGINE_EXECUTION_BACKEND_RUNTIME)
#    include <SushiEngine/execution/backend/runtime_backend.hpp>
#else
#    include <SushiEngine/execution/backend/native_backend.hpp>
#endif

namespace SushiEngine
{
    namespace Execution
    {
#if defined(SUSHIENGINE_EXECUTION_BACKEND_RUNTIME)
        /** @brief The backend this build executes through. */
        namespace SelectedBackend = RuntimeBackend;
#else
        /** @brief The backend this build executes through. */
        namespace SelectedBackend = NativeBackend;
#endif

        /** @brief Owns a runtime and hands out @ref Context instances bound to it. */
        using Runtime = SelectedBackend::Runtime;

        /** @brief The device or thread pool subsystems allocate and build graphs against. */
        using Context = SelectedBackend::Context;

        /** @brief A graph of nodes compiled once and replayed. */
        using Graph = SelectedBackend::Graph;

        /** @brief A graph partitioned into regions that stream in and out cheaply. */
        using DynamicGraph = SelectedBackend::DynamicGraph;

        /** @brief One mutable partition of a @ref DynamicGraph, recorded like a @ref Graph. */
        using Region = SelectedBackend::Region;

        /**
         * @brief An allocation the execution backend owns.
         * @tparam T Element type.
         */
        template <typename T>
        using Buffer = SelectedBackend::Buffer<T>;
    } // namespace Execution
} // namespace SushiEngine
