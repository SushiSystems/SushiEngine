/**************************************************************************/
/* runtime_backend.hpp                                                    */
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
#include <memory>
#include <utility>
#include <vector>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/execution/hazard.hpp>
#include <SushiEngine/execution/memory.hpp>
#include <SushiEngine/execution/node_descriptor.hpp>
#include <SushiEngine/execution/run_report.hpp>

/**
 * @file runtime_backend.hpp
 * @brief The SushiRuntime implementation of the execution seam.
 *
 * The only header in the engine that translates the execution vocabulary into
 * SushiRuntime's API, and the only one an engine subsystem must not include directly:
 * everything else names `Execution::Context`/`Execution::Graph` through
 * `execution/context.hpp`, so retargeting the engine is a build-system choice rather
 * than an edit across call sites.
 *
 * The translation is deliberately member-wise. A buffer interval already has the shape
 * of the runtime's own resource region, and a node's accesses already split into the
 * reads and writes the runtime's dependency tracker consumes, so nothing here infers,
 * caches, or reorders anything — a layer that did would be a second scheduler hiding
 * inside an adapter.
 */

namespace SushiEngine
{
    namespace Execution
    {
        namespace RuntimeBackend
        {
            namespace Detail
            {
                /**
                 * @brief Lowers a buffer interval to the runtime's resource region.
                 *
                 * A field-for-field copy, with one translation: the two types spell
                 * "to the end of the allocation" with sentinels of different widths,
                 * and mapping one onto the other explicitly keeps the meaning intact
                 * on any word size rather than relying on them coinciding.
                 *
                 * @param interval The engine-side byte interval.
                 * @return The same interval in the runtime's vocabulary.
                 */
                inline SushiRuntime::Core::ResourceRegion
                to_region(const BufferInterval& interval) noexcept
                {
                    const std::size_t length =
                        interval.size == BufferInterval::WHOLE
                            ? SushiRuntime::Core::ResourceRegion::WHOLE
                            : static_cast<std::size_t>(interval.size);
                    return SushiRuntime::Core::ResourceRegion{
                        interval.base, static_cast<std::size_t>(interval.offset), length};
                }

                /**
                 * @brief Splits a node's declared accesses into runtime read and write keys.
                 *
                 * The runtime's tracker keys on two lists rather than on intent, so the
                 * access algebra is projected here through the same predicate every
                 * other backend uses — which is what keeps the two from disagreeing
                 * about what counts as a write.
                 *
                 * Texture ranges are skipped: no simulation node declares one, and the
                 * runtime has no image concept to lower them onto.
                 *
                 * @param node   The node whose accesses are being lowered.
                 * @param reads  Receives the read regions, in declaration order.
                 * @param writes Receives the write regions, in declaration order.
                 */
                inline void split_accesses(const NodeDescriptor& node,
                                           SushiRuntime::API::Reads& reads,
                                           SushiRuntime::API::Writes& writes)
                {
                    for (const ResourceAccess& access : node)
                    {
                        if (access.interval.kind != ResourceInterval::Kind::Buffer)
                            continue;

                        const SushiRuntime::Core::ResourceRegion region =
                            to_region(access.interval.buffer);

                        if (intent_writes(access.intent))
                            writes.keys.push_back(region);
                        else
                            reads.keys.push_back(region);
                    }
                }

                /**
                 * @brief Builds the runtime's late-binding record from a node's providers.
                 *
                 * Each provider is copied into a small closure rather than invoked here:
                 * the whole point of a provider is that the value is read at run time,
                 * once per run, so a graph built this frame follows next frame's live
                 * counts without recompiling.
                 *
                 * @param node The node whose late-bound values are being lowered.
                 * @return The runtime's late-binding controls; empty when nothing is bound.
                 */
                inline SushiRuntime::API::Dynamic to_dynamic(const NodeDescriptor& node)
                {
                    SushiRuntime::API::Dynamic dynamic;
                    if (node.count.bound())
                        dynamic.and_sized([count = node.count] { return count.read(0); });
                    if (node.base.bound())
                        dynamic.and_based_at([base = node.base] { return base.read(0); });
                    if (node.enabled.bound())
                        dynamic.and_when([enabled = node.enabled] { return enabled.read(true); });
                    return dynamic;
                }

                /**
                 * @brief Projects a runtime run report onto the portable subset.
                 * @param report The runtime's full report.
                 * @return The fields every backend can answer for.
                 */
                inline RunReport to_run_report(const SushiRuntime::Core::RunReport& report) noexcept
                {
                    RunReport out;
                    out.is_successful        = report.is_successful;
                    out.cancelled            = report.cancelled;
                    out.total_tasks_executed = report.total_tasks_executed;
                    out.total_duration_ms    = report.total_duration_ms;
                    out.compile_duration_ms  = report.graph_analysis_duration_ms;
                    return out;
                }
            } // namespace Detail

            /**
             * @brief An allocation owned by the runtime, named by the shared vocabulary.
             *
             * Adds exactly one thing to the runtime's own buffer: the byte interval that
             * identifies it to a hazard tracker. Keeping that here rather than at every
             * call site is what stops a caller from computing an allocation's extent by
             * hand and getting it subtly wrong.
             *
             * @tparam T Element type.
             */
            template <typename T>
            class Buffer
            {
                public:
                    /** @brief Constructs an empty handle owning nothing. */
                    Buffer() = default;

                    /**
                     * @brief Adopts a runtime allocation.
                     * @param buffer The allocation to take ownership of.
                     */
                    explicit Buffer(SushiRuntime::API::Buffer<T> buffer) noexcept
                        : buffer_(std::move(buffer))
                    {
                    }

                    /** @brief Base address of the allocation, or null when empty. */
                    T* data() const noexcept { return buffer_.data(); }

                    /** @brief Number of elements the allocation holds. */
                    std::size_t size() const noexcept { return buffer_.size(); }

                    /**
                     * @brief The byte interval that identifies this allocation to a tracker.
                     *
                     * The exact extent rather than a whole-allocation sentinel, so a
                     * caller declaring a sub-range of it and a caller declaring all of it
                     * are comparing the same kind of value.
                     *
                     * @return An interval covering every byte of the allocation.
                     */
                    BufferInterval interval() const noexcept
                    {
                        return BufferInterval{static_cast<ResourceId>(buffer_.data()), 0,
                                              static_cast<std::uint64_t>(size() * sizeof(T))};
                    }

                    /**
                     * @brief The byte interval covering one window of elements.
                     *
                     * The element-to-byte conversion lives here rather than at the call
                     * site because this is the only place that knows the element size —
                     * and a caller doing that arithmetic by hand is how two nodes end up
                     * declaring windows that overlap in bytes but look disjoint.
                     *
                     * @param range The element window to name.
                     * @return An interval covering exactly that window's bytes.
                     */
                    BufferInterval interval(ElementRange range) const noexcept
                    {
                        return BufferInterval{
                            static_cast<ResourceId>(buffer_.data()),
                            static_cast<std::uint64_t>(range.first * sizeof(T)),
                            static_cast<std::uint64_t>(range.count * sizeof(T))};
                    }

                    /**
                     * @brief Host access to one element.
                     *
                     * Valid only on a host-shared allocation; the backend rejects it on a
                     * device-resident one, where read_range/write_range are the way in.
                     *
                     * @param index The element to address.
                     * @return A reference to that element.
                     */
                    T& operator[](std::size_t index) const { return buffer_[index]; }

                    /**
                     * @brief Copies a window of elements out to host memory.
                     * @param range The element window to read.
                     * @return The window's current contents.
                     */
                    std::vector<T> read_range(ElementRange range) const
                    {
                        return buffer_.read_range(
                            SushiRuntime::API::ElementRange{range.first, range.count});
                    }

                    /**
                     * @brief Copies host memory into a window of elements.
                     * @param range  The element window to write.
                     * @param source Host array of at least range.count elements.
                     */
                    void write_range(ElementRange range, const T* source)
                    {
                        buffer_.write_range(
                            SushiRuntime::API::ElementRange{range.first, range.count}, source);
                    }

                    /**
                     * @brief The underlying runtime allocation.
                     *
                     * Present only on this backend, for the one thing the portable surface
                     * cannot express yet — handing an allocation to a runtime call the
                     * seam has no verb for. Every use is a site the native backend will
                     * fail to compile, which is the point.
                     *
                     * @return The runtime buffer this handle owns.
                     */
                    const SushiRuntime::API::Buffer<T>& native() const noexcept { return buffer_; }

                private:
                    SushiRuntime::API::Buffer<T> buffer_;
            };

            /**
             * @brief A graph of nodes the runtime compiles once and replays.
             *
             * Node submission stays templated on the kernel type all the way through:
             * the callable reaches the runtime's launch unchanged, never wrapped in a
             * type-erasing indirection, because a device backend cannot capture one into
             * a kernel. The description beside it is a plain struct, which is what lets a
             * host backend inspect a node without instantiating anything.
             */
            class Graph
            {
                public:
                    /**
                     * @brief Wraps a freshly created runtime graph.
                     * @param graph The runtime graph to own.
                     */
                    explicit Graph(SushiRuntime::API::Graph graph) noexcept
                        : graph_(std::move(graph))
                    {
                    }

                    /**
                     * @brief Adds a per-element node over the range @p node describes.
                     *
                     * The kernel is invoked as `kernel(i)` for each live index; which
                     * indices are live is decided per run by the node's providers, so a
                     * changing entity count needs no recompile.
                     *
                     * @tparam Kernel The per-element callable's type.
                     * @param node   The node's accesses, capacity, and late-bound values.
                     * @param kernel The per-element callable.
                     * @return *this, for chaining.
                     */
                    template <typename Kernel>
                    Graph& add_parallel(const NodeDescriptor& node, Kernel&& kernel)
                    {
                        assert(node.kind == NodeKind::Parallel &&
                               "add_parallel() on a node declared as another kind");

                        SushiRuntime::API::Reads reads;
                        SushiRuntime::API::Writes writes;
                        Detail::split_accesses(node, reads, writes);

                        // node.name is nullable (§5.5's NodeDescriptor default); the
                        // runtime's own name parameter is a non-null const char* with no
                        // null check of its own, so an unlabeled node falls back here
                        // rather than handing it a null string it will copy into a
                        // std::string.
                        graph_.add(Detail::to_dynamic(node), reads, writes, node.capacity,
                                   std::forward<Kernel>(kernel),
                                   node.name != nullptr ? node.name : "unnamed_task");
                        return *this;
                    }

                    /**
                     * @brief Adds a host node ordered against the accesses @p node declares.
                     *
                     * @tparam Kernel The callable's type; invoked once with no arguments.
                     * @param node   The node's declared accesses.
                     * @param kernel The host callable.
                     * @return *this, for chaining.
                     */
                    template <typename Kernel>
                    Graph& add_host(const NodeDescriptor& node, Kernel&& kernel)
                    {
                        assert(node.kind == NodeKind::Host &&
                               "add_host() on a node declared as another kind");

                        SushiRuntime::API::Reads reads;
                        SushiRuntime::API::Writes writes;
                        Detail::split_accesses(node, reads, writes);

                        graph_.add_host(reads, writes, std::forward<Kernel>(kernel),
                                        node.name != nullptr ? node.name : "unnamed_task");
                        return *this;
                    }

                    /**
                     * @brief Adds a fixed-order fold of @p count elements into @p output.
                     *
                     * A node kind rather than something each caller builds out of ordinary
                     * nodes, because the combine order is the contract: a floating-point
                     * maximum or sum must be a function of the element layout alone, never
                     * of the worker count or the steal pattern, or every value derived
                     * from it inherits the scheduler's mood. A backend owning the kind can
                     * promise that; a hand-built fold bakes one backend's shape into the
                     * caller and promises nothing.
                     *
                     * @tparam T       Element type.
                     * @tparam Combine Binary functor; trivially copyable, so a device
                     *                 backend can capture it into a kernel.
                     * @param input    Values to fold; must hold at least @p count elements.
                     * @param output   Destination; element 0 receives the result.
                     * @param count    Number of leading elements of @p input to fold.
                     * @param combine  The combiner.
                     * @param identity Result when @p count is zero; never enters the
                     *                 arithmetic otherwise.
                     * @return *this, for chaining.
                     */
                    template <typename T, typename Combine>
                    Graph& add_reduce(const Buffer<T>& input, const Buffer<T>& output,
                                      std::size_t count, Combine combine, T identity)
                    {
                        graph_.add_reduce(input.native(), output.native(), count, combine,
                                          identity);
                        return *this;
                    }

                    /** @brief Number of nodes registered so far. */
                    std::size_t size() const noexcept { return graph_.size(); }

                    /** @brief Times this graph has been compiled; one after warm-up. */
                    std::size_t compile_count() const noexcept { return graph_.compile_count(); }

                    /**
                     * @brief Compiles if needed, then executes every node once.
                     * @return What the run did, in the portable subset.
                     */
                    RunReport run()
                    {
                        native_report_ = graph_.run();
                        return Detail::to_run_report(native_report_);
                    }

                    /**
                     * @brief The backend-native report for the most recent @ref run.
                     *
                     * `RunReport` is deliberately the cross-backend intersection (its own
                     * doc comment says so); this is the escape hatch that comment
                     * promises — SushiRuntime's own `node_timings`/`worker_timings`
                     * (§18 R8), empty unless the runtime was created with profiling on.
                     * Default-constructed (every field empty/zero) before the first
                     * @ref run.
                     */
                    const SushiRuntime::Core::RunReport& native_report() const noexcept
                    {
                        return native_report_;
                    }

                private:
                    SushiRuntime::API::Graph graph_;
                    SushiRuntime::Core::RunReport native_report_;
            };

            /**
             * @brief One mutable partition of a DynamicGraph, recorded like a Graph.
             *
             * Exposes the same three node-building calls @ref Graph does — nothing
             * else, because a region does not run itself; its owning DynamicGraph
             * composes every live region into one plan and steps them together. A
             * non-owning wrapper by necessity: the runtime's own `API::Region` lives
             * in the DynamicGraph that returned it and is neither copyable nor
             * movable, so this type is a thin reference the caller cannot outlive it.
             */
            class Region
            {
                public:
                    /**
                     * @brief Wraps a region already recorded in some DynamicGraph.
                     * @param region The runtime region to build nodes against; must
                     *               outlive this wrapper.
                     */
                    explicit Region(SushiRuntime::API::Region& region) noexcept
                        : region_(region)
                    {
                    }

                    /** @copydoc Graph::add_parallel */
                    template <typename Kernel>
                    Region& add_parallel(const NodeDescriptor& node, Kernel&& kernel)
                    {
                        assert(node.kind == NodeKind::Parallel &&
                               "add_parallel() on a node declared as another kind");

                        SushiRuntime::API::Reads reads;
                        SushiRuntime::API::Writes writes;
                        Detail::split_accesses(node, reads, writes);

                        region_.add(Detail::to_dynamic(node), reads, writes, node.capacity,
                                    std::forward<Kernel>(kernel),
                                    node.name != nullptr ? node.name : "unnamed_task");
                        return *this;
                    }

                    /** @copydoc Graph::add_host */
                    template <typename Kernel>
                    Region& add_host(const NodeDescriptor& node, Kernel&& kernel)
                    {
                        assert(node.kind == NodeKind::Host &&
                               "add_host() on a node declared as another kind");

                        SushiRuntime::API::Reads reads;
                        SushiRuntime::API::Writes writes;
                        Detail::split_accesses(node, reads, writes);

                        region_.add_host(reads, writes, std::forward<Kernel>(kernel),
                                         node.name != nullptr ? node.name : "unnamed_task");
                        return *this;
                    }

                    /** @copydoc Graph::add_reduce */
                    template <typename T, typename Combine>
                    Region& add_reduce(const Buffer<T>& input, const Buffer<T>& output,
                                       std::size_t count, Combine combine, T identity)
                    {
                        region_.add_reduce(input.native(), output.native(), count, combine,
                                           identity);
                        return *this;
                    }

                    /** @brief Number of nodes recorded in this region so far. */
                    std::size_t size() const noexcept { return region_.size(); }

                private:
                    SushiRuntime::API::Region& region_;
            };

            /**
             * @brief A graph partitioned into regions that stream in and out cheaply.
             *
             * The mutable counterpart to @ref Graph, keyed by a caller-chosen
             * identity per region (§6.6's "one region per island"). `region(key)`
             * records work with the same node-building surface a plain Graph has;
             * `drop(key)` removes one. Both only ever touch this object's own
             * bookkeeping — the actual recomposition happens lazily, inside the next
             * @ref run, so a caller never has to call a separate commit step.
             *
             * Composition is incremental in the changed region and its shared
             * boundary, not the whole world (SushiRuntime's own `DynamicGraph` doc
             * comment states the cost bound this promises: O(changed region +
             * affected boundary pins)). Cross-region edges follow ascending region
             * key, so a caller assigning keys must do so deterministically, or the
             * dependency order — and with it the result — depends on how the keys
             * happened to come out.
             */
            class DynamicGraph
            {
                public:
                    /** @brief Caller-chosen identity of a region; stable across steps. */
                    using RegionKey = SushiRuntime::API::DynamicGraph::RegionKey;

                    /**
                     * @brief Wraps a freshly created runtime dynamic graph.
                     * @param graph The runtime dynamic graph to own.
                     */
                    explicit DynamicGraph(SushiRuntime::API::DynamicGraph graph) noexcept
                        : graph_(std::move(graph))
                    {
                    }

                    /**
                     * @brief Returns the region for @p key, creating it if absent.
                     * @param key Identity of the region; stable across steps.
                     * @return A wrapper valid until the region is dropped.
                     */
                    Region region(RegionKey key) { return Region(graph_.region(key)); }

                    /** @brief Whether a live region with @p key exists. */
                    bool has_region(RegionKey key) const noexcept
                    {
                        return graph_.has_region(key);
                    }

                    /**
                     * @brief Removes the region for @p key from the live set.
                     * @param key Identity of the region to remove; a no-op if absent.
                     */
                    void drop(RegionKey key) { graph_.drop(key); }

                    /** @brief Number of live regions. */
                    std::size_t region_count() const noexcept { return graph_.region_count(); }

                    /** @brief Total nodes recorded across every live region. */
                    std::size_t size() const noexcept { return graph_.size(); }

                    /** @brief Times the plan has been (re)composed; one after warm-up. */
                    std::size_t compile_count() const noexcept { return graph_.compile_count(); }

                    /**
                     * @brief Recomposes if any region changed, then executes one step.
                     * @return What the run did, in the portable subset.
                     */
                    RunReport run()
                    {
                        native_report_ = graph_.run();
                        return Detail::to_run_report(native_report_);
                    }

                    /** @copydoc Graph::native_report */
                    const SushiRuntime::Core::RunReport& native_report() const noexcept
                    {
                        return native_report_;
                    }

                private:
                    SushiRuntime::API::DynamicGraph graph_;
                    SushiRuntime::Core::RunReport native_report_;
            };

            /**
             * @brief The runtime an engine subsystem allocates and builds graphs against.
             *
             * A borrowed reference, not an owner: the runtime's lifetime is the
             * application's concern, and a context that outlived it would hand out
             * allocations against a dead device.
             */
            class Context
            {
                public:
                    /**
                     * @brief Binds a context to an existing runtime.
                     * @param runtime The runtime to allocate and schedule against; must
                     *                outlive this context and everything it hands out.
                     */
                    explicit Context(SushiRuntime::API::Runtime& runtime) noexcept
                        : runtime_(runtime)
                    {
                    }

                    /**
                     * @brief Allocates @p count elements with the requested visibility.
                     * @tparam T     Element type; trivially copyable for device use.
                     * @param count      Number of elements.
                     * @param visibility Who must be able to address the allocation.
                     * @return An owning handle; freed when it goes out of scope.
                     */
                    template <typename T>
                    Buffer<T> allocate(std::size_t count,
                                       MemoryVisibility visibility = MemoryVisibility::HostShared,
                                       DeviceIndex device = DeviceIndex{})
                    {
                        const SushiRuntime::API::Residency residency =
                            visibility == MemoryVisibility::HostShared
                                ? SushiRuntime::API::Residency::Shared
                                : SushiRuntime::API::Residency::Device;
                        return Buffer<T>(runtime_.buffer<T>(
                            count, SushiRuntime::API::DeviceIndex{device.value}, residency));
                    }

                    /**
                     * @brief Enables or disables opportunistic migration of work between workers.
                     *
                     * A throughput/reproducibility trade every backend with a work pool
                     * eventually has to expose: migrating tasks mid-run raises throughput
                     * on long batch work and costs run-to-run reproducibility and the
                     * jitter floor, which is the wrong trade for a fixed-rate simulation
                     * tick. A backend that never migrates work accepts the call and does
                     * nothing, because "do not migrate" is already true of it.
                     *
                     * @param enabled True to allow migration, false to hold work where it was placed.
                     */
                    void set_work_migration(bool enabled) { runtime_.rebalancer(enabled); }

                    /**
                     * @brief What this backend can do, for a caller that must ask.
                     * @return The capability record; see BackendCapabilities.
                     */
                    BackendCapabilities capabilities() const
                    {
                        BackendCapabilities out;
                        out.device_count = runtime_.advanced().device_count();
                        out.device_resident_memory = true;
                        return out;
                    }

                    /** @brief Creates an empty graph bound to this context. */
                    Graph create_graph() { return Graph(runtime_.graph()); }

                    /** @brief Creates an empty region-partitioned graph bound to this context. */
                    DynamicGraph create_dynamic_graph()
                    {
                        return DynamicGraph(runtime_.dynamic_graph());
                    }

                    /**
                     * @brief The underlying runtime, for subsystems not yet on the seam.
                     *
                     * Present only on this backend, deliberately. Every remaining direct
                     * consumer of SushiRuntime therefore fails to compile the moment the
                     * engine is built against a different backend, which turns "what is
                     * left to port" from a question someone has to audit into something
                     * the build answers.
                     *
                     * @return The runtime this context was bound to.
                     */
                    SushiRuntime::API::Runtime& runtime() noexcept { return runtime_; }

                    /** @brief The underlying runtime, read-only. */
                    const SushiRuntime::API::Runtime& runtime() const noexcept { return runtime_; }

                private:
                    SushiRuntime::API::Runtime& runtime_;
            };

            /**
             * @brief Owns a runtime and hands out contexts bound to it.
             *
             * The portable "stand up a backend" factory `Execution::Runtime` denotes on
             * this build: a caller that used to do
             * `auto runtime = SushiRuntime::API::Runtime::create(); Context context(runtime);`
             * now does `auto runtime = Execution::Runtime::create(); auto context =
             * runtime.context();` unchanged across backends. Heap-allocates the runtime
             * behind a `unique_ptr` rather than storing it by value so this type is
             * cheaply movable regardless of whether `SushiRuntime::API::Runtime` itself
             * is — `native()` is the escape hatch every remaining direct SushiRuntime
             * consumer (audio's optional DSP accelerator, `Loop::App::runtime()`) still
             * needs, present only on this backend for the same reason `Context::runtime()`
             * is.
             */
            class Runtime
            {
                public:
                    /** @brief Creates a new runtime, discovering and binding a device. */
                    static Runtime create()
                    {
                        // Guaranteed copy elision (C++17): the prvalue from create()
                        // initialises the heap object directly, so SushiRuntime::API::Runtime
                        // is never required to be movable.
                        return Runtime(
                            new SushiRuntime::API::Runtime(SushiRuntime::API::Runtime::create()));
                    }

                    Runtime(const Runtime&) = delete;
                    Runtime& operator=(const Runtime&) = delete;
                    Runtime(Runtime&&) = default;
                    Runtime& operator=(Runtime&&) = default;

                    /** @brief A new context bound to this runtime. */
                    Context context() noexcept { return Context(*runtime_); }

                    /** @brief The underlying runtime, for subsystems not yet on the seam. */
                    SushiRuntime::API::Runtime& native() noexcept { return *runtime_; }

                    /** @copydoc native() */
                    const SushiRuntime::API::Runtime& native() const noexcept { return *runtime_; }

                private:
                    explicit Runtime(SushiRuntime::API::Runtime* runtime) : runtime_(runtime) {}

                    std::unique_ptr<SushiRuntime::API::Runtime> runtime_;
            };
        } // namespace RuntimeBackend
    } // namespace Execution
} // namespace SushiEngine
