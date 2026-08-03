/**************************************************************************/
/* dag_compiler.cpp                                                       */
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

#include <SushiEngine/execution/backend/native/dag_compiler.hpp>

#include <chrono>

#include <SushiEngine/execution/backend/native/thread_pool.hpp>

namespace SushiEngine
{
    namespace Execution
    {
        namespace NativeBackend
        {
            namespace
            {
                using Clock = std::chrono::steady_clock;

                double milliseconds_since(Clock::time_point start) noexcept
                {
                    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
                }

                /** @brief Binds one node's late-bound values into a zero-argument job. */
                std::function<void()> make_job(NodeRecord& node)
                {
                    if (node.kind == NodeKind::Parallel)
                    {
                        return [&node]
                        {
                            if (node.enabled.bound() && !node.enabled.read(true))
                                return;
                            const std::size_t count =
                                node.count.bound() ? node.count.read(node.capacity) : node.capacity;
                            const std::size_t base = node.base.bound() ? node.base.read(0) : 0;
                            for (std::size_t i = 0; i < count; ++i)
                                node.parallel_kernel(base + i);
                        };
                    }
                    return [&node]
                    {
                        if (node.enabled.bound() && !node.enabled.read(true))
                            return;
                        node.host_kernel();
                    };
                }
            } // namespace

            RunReport DAGCompiler::run(ThreadPool& pool)
            {
                const Clock::time_point compile_start = Clock::now();
                compile();
                const double compile_ms = milliseconds_since(compile_start);

                const Clock::time_point run_start = Clock::now();

                // Kahn's algorithm, waved: every node in one wave has every
                // predecessor already executed, and — because compile() ordered
                // every conflicting pair — no two nodes in the same wave declare
                // overlapping accesses, so the whole wave is safe to hand to the
                // pool at once. See ThreadPool's own doc comment for why this is
                // node-granular rather than element-granular dispatch.
                std::vector<std::size_t> remaining_predecessors(nodes_.size());
                std::vector<std::vector<std::size_t>> successors(nodes_.size());
                for (std::size_t i = 0; i < nodes_.size(); ++i)
                {
                    remaining_predecessors[i] = nodes_[i].predecessors.size();
                    for (std::size_t predecessor : nodes_[i].predecessors)
                        successors[predecessor].push_back(i);
                }

                std::vector<std::size_t> ready;
                for (std::size_t i = 0; i < nodes_.size(); ++i)
                    if (remaining_predecessors[i] == 0)
                        ready.push_back(i);

                std::size_t executed = 0;
                while (!ready.empty())
                {
                    std::vector<std::function<void()>> jobs;
                    jobs.reserve(ready.size());
                    for (std::size_t index : ready)
                        jobs.push_back(make_job(nodes_[index]));

                    pool.run_batch(std::move(jobs));
                    executed += ready.size();

                    std::vector<std::size_t> next_ready;
                    for (std::size_t index : ready)
                        for (std::size_t successor : successors[index])
                            if (--remaining_predecessors[successor] == 0)
                                next_ready.push_back(successor);
                    ready = std::move(next_ready);
                }

                RunReport report;
                report.is_successful = true;
                report.cancelled = false;
                report.total_tasks_executed = executed;
                report.total_duration_ms = milliseconds_since(run_start);
                report.compile_duration_ms = compile_ms;
                return report;
            }
        } // namespace NativeBackend
    } // namespace Execution
} // namespace SushiEngine
