/**************************************************************************/
/* native_backend.hpp                                                     */
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

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <new>
#include <thread>
#include <utility>
#include <vector>

#include <SushiEngine/execution/access.hpp>
#include <SushiEngine/execution/backend/native/dag_compiler.hpp>
#include <SushiEngine/execution/backend/native/thread_pool.hpp>
#include <SushiEngine/execution/interval.hpp>
#include <SushiEngine/execution/memory.hpp>
#include <SushiEngine/execution/node_descriptor.hpp>
#include <SushiEngine/execution/run_report.hpp>

/**
 * @file native_backend.hpp
 * @brief RUNTIME-PORT1: the SYCL-free implementation of the execution seam.
 *
 * Mirrors `runtime_backend.hpp`'s shape exactly — the same five names
 * (`Buffer`, `Graph`, `Region`, `DynamicGraph`, `Context`, plus `Runtime`),
 * satisfied by duck typing rather than a common base, since the templated
 * `add_parallel`/`add_host` methods could not be virtual even if the two
 * backends wanted to share one. This backend has no device to forward a
 * kernel into, so — unlike `RuntimeBackend`, which must keep a caller's raw
 * lambda intact all the way into a SYCL launch — every kernel here is
 * captured into a `std::function` inside `NativeBackend::NodeRecord`
 * (dag_compiler.hpp). That is not a compromise made to match the other
 * backend; it is what a thread pool executing an ordinary C++ callable
 * actually is.
 */

namespace SushiEngine
{
    namespace Execution
    {
        namespace NativeBackend
        {
            /**
             * @brief A plain, aligned heap allocation, named by the shared vocabulary.
             *
             * `MemoryVisibility::DeviceResident` degenerates to the same allocation
             * as `HostShared` (§4.2 Service C's documented behavior for a backend
             * with no device) — `Context::allocate` accepts the visibility argument
             * for source compatibility with `RuntimeBackend` but this class never
             * sees or branches on it. Deliberately no `native()` escape hatch: unlike
             * `RuntimeBackend::Buffer`, which needs one for the handful of call sites
             * not yet ported off SushiRuntime directly, there is nothing beneath this
             * type a caller could reach for — a call site that tried would be asking
             * for `void*` back, which is simply `data()`.
             *
             * @tparam T Element type.
             */
            template <typename T>
            class Buffer
            {
                public:
                    /** @brief Constructs an empty handle owning nothing. */
                    Buffer() noexcept = default;

                    /**
                     * @brief Allocates and value-initializes @p count elements.
                     * @param count Number of elements to allocate.
                     */
                    explicit Buffer(std::size_t count) : size_(count)
                    {
                        if (count == 0)
                            return;
                        data_ = static_cast<T*>(
                            ::operator new(count * sizeof(T), std::align_val_t{alignof(T)}));
                        std::size_t constructed = 0;
                        for (; constructed < count; ++constructed)
                            new (static_cast<void*>(data_ + constructed)) T();
                        (void)constructed;
                    }

                    ~Buffer() { destroy(); }

                    Buffer(const Buffer&) = delete;
                    Buffer& operator=(const Buffer&) = delete;

                    Buffer(Buffer&& other) noexcept : data_(other.data_), size_(other.size_)
                    {
                        other.data_ = nullptr;
                        other.size_ = 0;
                    }

                    Buffer& operator=(Buffer&& other) noexcept
                    {
                        if (this != &other)
                        {
                            destroy();
                            data_ = other.data_;
                            size_ = other.size_;
                            other.data_ = nullptr;
                            other.size_ = 0;
                        }
                        return *this;
                    }

                    /** @brief Base address of the allocation, or null when empty. */
                    T* data() const noexcept { return data_; }

                    /** @brief Number of elements the allocation holds. */
                    std::size_t size() const noexcept { return size_; }

                    /** @brief The byte interval covering the whole allocation. */
                    BufferInterval interval() const noexcept
                    {
                        return BufferInterval{static_cast<ResourceId>(data_), 0,
                                              static_cast<std::uint64_t>(size_ * sizeof(T))};
                    }

                    /** @brief The byte interval covering one window of elements. */
                    BufferInterval interval(ElementRange range) const noexcept
                    {
                        return BufferInterval{
                            static_cast<ResourceId>(data_),
                            static_cast<std::uint64_t>(range.first * sizeof(T)),
                            static_cast<std::uint64_t>(range.count * sizeof(T))};
                    }

                    /** @brief Access to one element. */
                    T& operator[](std::size_t index) const { return data_[index]; }

                    /** @brief Copies a window of elements out to host memory. */
                    std::vector<T> read_range(ElementRange range) const
                    {
                        std::vector<T> out(range.count);
                        for (std::size_t i = 0; i < range.count; ++i)
                            out[i] = data_[range.first + i];
                        return out;
                    }

                    /** @brief Copies host memory into a window of elements. */
                    void write_range(ElementRange range, const T* source)
                    {
                        for (std::size_t i = 0; i < range.count; ++i)
                            data_[range.first + i] = source[i];
                    }

                private:
                    void destroy() noexcept
                    {
                        if (data_ == nullptr)
                            return;
                        for (std::size_t i = 0; i < size_; ++i)
                            data_[i].~T();
                        ::operator delete(static_cast<void*>(data_), std::align_val_t{alignof(T)});
                        data_ = nullptr;
                        size_ = 0;
                    }

                    T* data_ = nullptr;
                    std::size_t size_ = 0;
            };

            /**
             * @brief Builds a two-access node declaring a fold's read of @p input
             * and write of @p output, then submits it as a host node.
             *
             * Shared by @ref Graph::add_reduce and @ref Region::add_reduce so the
             * access declaration — the part a conformance check actually cares
             * about — is written once. `Sink` is either a `Graph`/`Region`'s own
             * `add_host`, passed in so this stays free of either type's definition.
             */
            template <typename T, typename Combine, typename Sink>
            void submit_reduce(Sink&& sink, const Buffer<T>& input, const Buffer<T>& output,
                               std::size_t count, Combine combine, T identity)
            {
                const ResourceAccess accesses[] = {
                    ResourceAccess{input.interval(ElementRange{0, count}), AccessIntent::HostRead},
                    ResourceAccess{output.interval(ElementRange{0, 1}), AccessIntent::HostWrite}};

                NodeDescriptor node;
                node.name = "reduce";
                node.accesses = accesses;
                node.access_count = 2;
                node.kind = NodeKind::Host;
                node.capacity = 1;

                // Always sequential, one thread, regardless of worker count — the
                // fold order is the contract (Graph::add_reduce's own doc states
                // why), and a node-granular pool never splits one node's work
                // across workers in the first place, so this is simply what a
                // host node already does; no special-casing was needed to get it.
                sink(node, [&input, &output, count, combine, identity]
                     {
                         T accumulator = identity;
                         const T* data = input.data();
                         for (std::size_t i = 0; i < count; ++i)
                             accumulator = combine(accumulator, data[i]);
                         output.data()[0] = accumulator;
                     });
            }

            /**
             * @brief A graph of nodes compiled once and replayed, over a native
             * thread pool.
             */
            class Graph
            {
                public:
                    /**
                     * @brief Binds a graph to the pool it will dispatch onto.
                     * @param pool The worker pool this graph's runs execute over;
                     *             must outlive the graph.
                     */
                    explicit Graph(ThreadPool& pool) noexcept : pool_(pool) {}

                    /** @copydoc RuntimeBackend::Graph::add_parallel */
                    template <typename Kernel>
                    Graph& add_parallel(const NodeDescriptor& node, Kernel&& kernel)
                    {
                        assert(node.kind == NodeKind::Parallel &&
                               "add_parallel() on a node declared as another kind");
                        compiler_.add(node);
                        compiler_.back().parallel_kernel = std::forward<Kernel>(kernel);
                        return *this;
                    }

                    /** @copydoc RuntimeBackend::Graph::add_host */
                    template <typename Kernel>
                    Graph& add_host(const NodeDescriptor& node, Kernel&& kernel)
                    {
                        assert(node.kind == NodeKind::Host &&
                               "add_host() on a node declared as another kind");
                        compiler_.add(node);
                        compiler_.back().host_kernel = std::forward<Kernel>(kernel);
                        return *this;
                    }

                    /** @copydoc RuntimeBackend::Graph::add_reduce */
                    template <typename T, typename Combine>
                    Graph& add_reduce(const Buffer<T>& input, const Buffer<T>& output,
                                      std::size_t count, Combine combine, T identity)
                    {
                        submit_reduce(
                            [this](const NodeDescriptor& node, auto&& kernel)
                            { this->add_host(node, std::forward<decltype(kernel)>(kernel)); },
                            input, output, count, combine, identity);
                        return *this;
                    }

                    /** @brief Number of nodes registered so far. */
                    std::size_t size() const noexcept { return compiler_.size(); }

                    /** @brief Times this graph has been compiled; one after warm-up. */
                    std::size_t compile_count() const noexcept { return compiler_.compile_count(); }

                    /**
                     * @brief Compiles if needed, then executes every node once.
                     * @return What the run did, in the portable subset.
                     */
                    RunReport run() { return compiler_.run(pool_); }

                private:
                    DagCompiler compiler_;
                    ThreadPool& pool_;
            };

            /**
             * @brief One mutable partition of a DynamicGraph, recorded like a Graph.
             *
             * A thin, non-owning wrapper over the region's own pending node list —
             * `DynamicGraph` owns the storage, this only appends to it — since the
             * native backend has no per-region compiled object of its own the way
             * `RuntimeBackend::Region` wraps `SushiRuntime::API::Region`; recompose
             * (`DynamicGraph::run`) is where every live region's records actually
             * become one compiled plan (see that class's doc comment).
             */
            class Region
            {
                public:
                    /**
                     * @brief Wraps one region's pending node list.
                     * @param nodes The region's own storage, appended to in place.
                     * @param dirty The owning DynamicGraph's recompose flag, set
                     *              whenever a node is added.
                     */
                    Region(std::vector<NodeRecord>& nodes, bool& dirty) noexcept
                        : nodes_(nodes), dirty_(dirty)
                    {
                    }

                    /** @copydoc Graph::add_parallel */
                    template <typename Kernel>
                    Region& add_parallel(const NodeDescriptor& node, Kernel&& kernel)
                    {
                        assert(node.kind == NodeKind::Parallel &&
                               "add_parallel() on a node declared as another kind");
                        append(node, std::forward<Kernel>(kernel), nullptr);
                        return *this;
                    }

                    /** @copydoc Graph::add_host */
                    template <typename Kernel>
                    Region& add_host(const NodeDescriptor& node, Kernel&& kernel)
                    {
                        assert(node.kind == NodeKind::Host &&
                               "add_host() on a node declared as another kind");
                        append(node, nullptr, std::forward<Kernel>(kernel));
                        return *this;
                    }

                    /** @copydoc Graph::add_reduce */
                    template <typename T, typename Combine>
                    Region& add_reduce(const Buffer<T>& input, const Buffer<T>& output,
                                       std::size_t count, Combine combine, T identity)
                    {
                        submit_reduce(
                            [this](const NodeDescriptor& node, auto&& kernel)
                            { this->add_host(node, std::forward<decltype(kernel)>(kernel)); },
                            input, output, count, combine, identity);
                        return *this;
                    }

                    /** @brief Number of nodes recorded in this region so far. */
                    std::size_t size() const noexcept { return nodes_.size(); }

                private:
                    template <typename ParallelKernel, typename HostKernel>
                    void append(const NodeDescriptor& node, ParallelKernel&& parallel_kernel,
                               HostKernel&& host_kernel)
                    {
                        NodeRecord record;
                        record.accesses.assign(node.begin(), node.end());
                        record.kind = node.kind;
                        record.determinism = node.determinism;
                        record.capacity = node.capacity;
                        record.count = node.count;
                        record.base = node.base;
                        record.enabled = node.enabled;
                        if (node.kind == NodeKind::Parallel)
                            record.parallel_kernel = std::forward<ParallelKernel>(parallel_kernel);
                        else
                            record.host_kernel = std::forward<HostKernel>(host_kernel);
                        nodes_.push_back(std::move(record));
                        dirty_ = true;
                    }

                    std::vector<NodeRecord>& nodes_;
                    bool& dirty_;
            };

            /**
             * @brief A graph partitioned into regions that stream in and out cheaply.
             *
             * Recomposition is a full rebuild, not the O(changed region + boundary)
             * bound `SushiRuntime::API::DynamicGraph`'s own doc comment promises —
             * UHM's floors (safety, determinism) never require matching that cost
             * bound, only ordering-equivalence, and no native-backend consumer
             * exists yet whose performance depends on it (P8's soft-body regions
             * run under `RuntimeBackend` today). This is an explicit, named
             * non-goal for this milestone, not an oversight: building the
             * incremental version now, for a caller that does not exist, is
             * exactly the speculative generality this codebase's standards forbid.
             * When a real native-backend consumer needs it, the fix is local to
             * this class's `run()` — nothing about `Region`'s or `DynamicGraph`'s
             * public surface would need to change.
             *
             * Every live region's records are concatenated in ascending
             * `RegionKey` order before compiling (`std::map` iterates that way by
             * construction), satisfying the determinism floor's "cross-region
             * edges follow ascending region key" rule for free.
             */
            class DynamicGraph
            {
                public:
                    /** @brief Caller-chosen identity of a region; stable across steps. */
                    using RegionKey = std::int64_t;

                    /**
                     * @brief Binds a dynamic graph to the pool it will dispatch onto.
                     * @param pool The worker pool this graph's runs execute over;
                     *             must outlive the graph.
                     */
                    explicit DynamicGraph(ThreadPool& pool) noexcept : pool_(pool) {}

                    /**
                     * @brief Returns the region for @p key, creating it if absent.
                     * @param key Identity of the region; stable across steps.
                     * @return A wrapper valid until the region is dropped.
                     */
                    Region region(RegionKey key)
                    {
                        auto [it, inserted] = regions_.try_emplace(key);
                        if (inserted)
                            dirty_ = true;
                        return Region(it->second, dirty_);
                    }

                    /** @brief Whether a live region with @p key exists. */
                    bool has_region(RegionKey key) const noexcept
                    {
                        return regions_.find(key) != regions_.end();
                    }

                    /**
                     * @brief Removes the region for @p key from the live set.
                     * @param key Identity of the region to remove; a no-op if absent.
                     */
                    void drop(RegionKey key)
                    {
                        if (regions_.erase(key) != 0)
                            dirty_ = true;
                    }

                    /** @brief Number of live regions. */
                    std::size_t region_count() const noexcept { return regions_.size(); }

                    /** @brief Total nodes recorded across every live region. */
                    std::size_t size() const noexcept
                    {
                        std::size_t total = 0;
                        for (const auto& entry : regions_)
                            total += entry.second.size();
                        return total;
                    }

                    /** @brief Times the plan has been (re)composed; one after warm-up. */
                    std::size_t compile_count() const noexcept { return compiled_.compile_count(); }

                    /**
                     * @brief Recomposes if any region changed, then executes one step.
                     * @return What the run did, in the portable subset.
                     */
                    RunReport run()
                    {
                        if (dirty_)
                        {
                            compiled_.reset();
                            for (auto& entry : regions_)
                                for (NodeRecord& record : entry.second)
                                    compiled_.add_record(record);
                            dirty_ = false;
                        }
                        return compiled_.run(pool_);
                    }

                private:
                    std::map<RegionKey, std::vector<NodeRecord>> regions_;
                    DagCompiler compiled_;
                    bool dirty_ = true;
                    ThreadPool& pool_;
            };

            /**
             * @brief The thread pool an engine subsystem allocates and builds graphs against.
             */
            class Context
            {
                public:
                    /**
                     * @brief Binds a context to an existing pool.
                     * @param pool The pool to build graphs against; must outlive
                     *             this context and everything it hands out.
                     */
                    explicit Context(ThreadPool& pool) noexcept : pool_(pool) {}

                    /**
                     * @brief Allocates @p count elements with the requested visibility.
                     *
                     * `visibility`/`device` are accepted for source compatibility
                     * with `RuntimeBackend::Context::allocate` and otherwise
                     * ignored: every allocation this backend makes is ordinary host
                     * memory (§4.2 Service C's documented degeneration for a
                     * backend with no device).
                     *
                     * @tparam T Element type.
                     * @param count Number of elements.
                     * @return An owning handle; freed when it goes out of scope.
                     */
                    template <typename T>
                    Buffer<T> allocate(std::size_t count,
                                       MemoryVisibility visibility = MemoryVisibility::HostShared,
                                       DeviceIndex device = DeviceIndex{})
                    {
                        (void)visibility;
                        (void)device;
                        return Buffer<T>(count);
                    }

                    /**
                     * @brief Accepted and ignored.
                     *
                     * A node-granular pool never migrates a node once a worker has
                     * taken it — "do not migrate" is already true of this backend,
                     * the same reasoning `RuntimeBackend::Context`'s own doc
                     * comment states for a backend that never migrates.
                     */
                    void set_work_migration(bool enabled) noexcept { (void)enabled; }

                    /** @brief What this backend can do, for a caller that must ask. */
                    BackendCapabilities capabilities() const noexcept
                    {
                        BackendCapabilities out;
                        out.device_count = 1;
                        out.device_resident_memory = false;
                        return out;
                    }

                    /** @brief Creates an empty graph bound to this context's pool. */
                    Graph create_graph() { return Graph(pool_); }

                    /** @brief Creates an empty region-partitioned graph bound to this context's pool. */
                    DynamicGraph create_dynamic_graph() { return DynamicGraph(pool_); }

                private:
                    ThreadPool& pool_;
            };

            /**
             * @brief Owns a thread pool and hands out contexts bound to it.
             *
             * The portable "stand up a backend" factory `Execution::Runtime`
             * denotes on this build, mirroring `RuntimeBackend::Runtime`'s shape:
             * heap-allocated behind a `unique_ptr` so this type is cheaply movable,
             * with `create()` sized off `std::thread::hardware_concurrency()`
             * (falling back to 1 when the platform cannot report it, which is
             * `hardware_concurrency`'s own documented zero case).
             */
            class Runtime
            {
                public:
                    /** @brief Creates a new runtime, sized to the host's hardware concurrency. */
                    static Runtime create()
                    {
                        const unsigned int hint = std::thread::hardware_concurrency();
                        return Runtime(hint == 0 ? std::size_t{1} : std::size_t{hint});
                    }

                    Runtime(const Runtime&) = delete;
                    Runtime& operator=(const Runtime&) = delete;
                    Runtime(Runtime&&) = default;
                    Runtime& operator=(Runtime&&) = default;

                    /** @brief A new context bound to this runtime's pool. */
                    Context context() noexcept { return Context(*pool_); }

                private:
                    explicit Runtime(std::size_t worker_count)
                        : pool_(new ThreadPool(worker_count))
                    {
                    }

                    std::unique_ptr<ThreadPool> pool_;
            };
        } // namespace NativeBackend
    } // namespace Execution
} // namespace SushiEngine
