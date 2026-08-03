/**************************************************************************/
/* dag_compiler.hpp                                                       */
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
#include <functional>
#include <vector>

#include <SushiEngine/execution/access.hpp>
#include <SushiEngine/execution/detail/hazard_core.hpp>
#include <SushiEngine/execution/interval.hpp>
#include <SushiEngine/execution/node_descriptor.hpp>
#include <SushiEngine/execution/run_report.hpp>

namespace SushiEngine
{
    namespace Execution
    {
        namespace NativeBackend
        {
            class ThreadPool;

            /**
             * @brief Everything @ref DagCompiler needs about one submitted node,
             * with its kernel bound.
             *
             * Unlike `RuntimeBackend`, which forwards a kernel unchanged into a SYCL
             * launch, the native backend has no device-capture constraint — a plain
             * worker thread calling a `std::function` is exactly what this backend
             * is. Erasing the kernel here is therefore not a compromise; it is what
             * lets `DagCompiler` be a single, ordinary (non-template) compiled class
             * instead of one instantiation per kernel type per node.
             *
             * Owns copies of everything `NodeDescriptor` only borrows (`accesses`,
             * `name`), since a node must survive past the `add_parallel`/`add_host`
             * call that submitted it.
             */
            struct NodeRecord
            {
                std::vector<ResourceAccess> accesses;
                NodeKind kind = NodeKind::Parallel;
                DeterminismClass determinism = DeterminismClass::Bitwise;
                std::size_t capacity = 0;
                CountProvider count{};
                BaseProvider base{};
                EnabledProvider enabled{};

                /** @brief Invoked as kernel(i) for each live index; NodeKind::Parallel only. */
                std::function<void(std::size_t)> parallel_kernel;

                /** @brief Invoked once with no arguments; NodeKind::Host only. */
                std::function<void()> host_kernel;

                std::vector<std::size_t> predecessors; /**< Node indices this node waits on. */
            };

            /**
             * @brief Compiles a flat node list into dependency order and executes it.
             *
             * The native counterpart to `SushiRuntime::API::Graph`'s internal
             * scheduler: `compile()` walks the recorded nodes in declaration order,
             * feeding each access to a fresh @ref Detail::HazardCore so the resulting
             * edge set is a pure function of declaration order and ranges (the
             * determinism floor), then `run()` executes every node once respecting
             * that order.
             *
             * `run()` here is single-threaded — every ready node executes inline, in
             * a fixed topological order derived from declaration order, which is
             * trivially both correct and deterministic. Real cross-node parallelism
             * (RUNTIME-PORT1's next checkpoint) changes how `run()` dispatches ready
             * nodes, not this class's compiled representation of the graph.
             */
            class DagCompiler
            {
                public:
                    /**
                     * @brief Records a node, copying its accesses out of the
                     * caller's (possibly stack-owned) descriptor.
                     * @param descriptor The node's accesses, capacity, and providers.
                     * @return This node's index, stable until the next @ref reset.
                     */
                    std::size_t add(const NodeDescriptor& descriptor)
                    {
                        NodeRecord record;
                        record.accesses.assign(descriptor.begin(), descriptor.end());
                        record.kind = descriptor.kind;
                        record.determinism = descriptor.determinism;
                        record.capacity = descriptor.capacity;
                        record.count = descriptor.count;
                        record.base = descriptor.base;
                        record.enabled = descriptor.enabled;
                        return add_record(std::move(record));
                    }

                    /**
                     * @brief Adopts an already-built node record (its kernel already
                     * bound), for a caller that constructed one itself.
                     *
                     * `DynamicGraph`'s recompose is this method's one caller: each
                     * live region already holds its own recorded `NodeRecord`s (built
                     * through `Region::add_parallel`/`add_host`, which have no
                     * `DagCompiler` of their own to add into), and recomposing means
                     * moving every region's records into one flattened compiler in
                     * ascending region-key order.
                     *
                     * @param record A fully-built node, kernel included.
                     * @return This node's index, stable until the next @ref reset.
                     */
                    std::size_t add_record(NodeRecord record)
                    {
                        const std::size_t index = nodes_.size();
                        nodes_.push_back(std::move(record));
                        dirty_ = true;
                        return index;
                    }

                    /** @brief The node just added by @ref add, for the kernel to be attached. */
                    NodeRecord& back() noexcept { return nodes_.back(); }

                    /** @brief Number of nodes recorded so far. */
                    std::size_t size() const noexcept { return nodes_.size(); }

                    /** @brief Times this compiler has (re)computed dependency order. */
                    std::size_t compile_count() const noexcept { return compile_count_; }

                    /**
                     * @brief Derives dependency order from every node's declared
                     * accesses, in declaration order, if anything changed since the
                     * last compile.
                     *
                     * A no-op when nothing was added since the last call — the same
                     * late-binding promise `RuntimeBackend::Graph` makes: a run that
                     * follows no new `add()` must not recompile.
                     */
                    void compile()
                    {
                        if (!dirty_)
                            return;

                        Detail::HazardCore<std::size_t> hazard;
                        for (std::size_t i = 0; i < nodes_.size(); ++i)
                        {
                            NodeRecord& node = nodes_[i];
                            node.predecessors.clear();
                            for (const ResourceAccess& access : node.accesses)
                            {
                                if (access.interval.kind != ResourceInterval::Kind::Buffer)
                                    continue; // no accepted caller in this domain declares one
                                std::vector<std::size_t> predecessors =
                                    hazard.record(i, access.interval.buffer, access.intent);
                                node.predecessors.insert(node.predecessors.end(),
                                                         predecessors.begin(), predecessors.end());
                            }
                        }

                        dirty_ = false;
                        ++compile_count_;
                    }

                    /**
                     * @brief Compiles if needed, then executes every node once.
                     * @param pool Worker pool `Parallel` nodes dispatch onto; the
                     *             caller's chosen concurrency (including single-
                     *             threaded, `ThreadPool{1}`) — this class never
                     *             constructs one of its own, so the same compiled
                     *             plan can be replayed under whichever pool
                     *             `{1,2,max}`-worker determinism testing selects.
                     * @return What the run did, in the portable subset.
                     */
                    RunReport run(ThreadPool& pool);

                    /** @brief Discards every recorded node, for a fresh recompose. */
                    void reset()
                    {
                        nodes_.clear();
                        dirty_ = true;
                    }

                private:
                    std::vector<NodeRecord> nodes_;
                    std::size_t compile_count_ = 0;
                    bool dirty_ = true;
            };
        } // namespace NativeBackend
    } // namespace Execution
} // namespace SushiEngine
