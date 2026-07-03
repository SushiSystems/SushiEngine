/**************************************************************************/
/* schedule.hpp                                                           */
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

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/ecs/archetype.hpp>
#include <SushiEngine/ecs/chunk.hpp>
#include <SushiEngine/ecs/component.hpp>
#include <SushiEngine/ecs/world.hpp>

namespace SushiEngine
{
    namespace Detail
    {
        /** @brief The pointer type a kernel receives for one access: const for reads. */
        template <typename Access>
        using column_ptr_t = std::conditional_t<Access::is_write,
                                                typename Access::type*,
                                                const typename Access::type*>;

        /**
         * @brief Resolves one access declaration to its column base address.
         *
         * Looks up the component's column in @p chunk and records its base address
         * as a read or write dependency key. The address is returned untyped; the
         * kernel reconstructs the typed pointer (see invoke_system), so only a plain
         * pointer array is captured into device code.
         *
         * @tparam Access A Read<T> or Write<T> declaration.
         * @param chunk  The chunk whose column is wanted.
         * @param reads  Collects read dependency keys.
         * @param writes Collects write dependency keys.
         * @return The column's base address.
         */
        template <typename Access>
        void* column_base(Chunk& chunk, std::vector<void*>& reads,
                          std::vector<void*>& writes)
        {
            void* base = chunk.column(component_id<typename Access::type>());
            if constexpr (Access::is_write)
                writes.push_back(base);
            else
                reads.push_back(base);
            return base;
        }

        /**
         * @brief Invokes a system kernel, retyping each captured column pointer.
         *
         * The kernel captures a trivially-copyable `std::array<void*, N>` (which SYCL
         * accepts as device-copyable, unlike a std::tuple) and this reinterprets each
         * entry back to the access's column type before the call, in declaration
         * order.
         *
         * @tparam Access The access declarations, in declaration order.
         * @tparam Fn     The kernel callable.
         * @tparam Is     Index sequence over the columns.
         * @param fn   The kernel to call.
         * @param i    The element index within the chunk.
         * @param cols The captured column base pointers.
         */
        template <typename... Access, typename Fn, std::size_t... Is>
        void invoke_system(const Fn& fn, std::size_t i,
                           const std::array<void*, sizeof...(Access)>& cols,
                           std::index_sequence<Is...>)
        {
            fn(i, reinterpret_cast<column_ptr_t<Access>>(cols[Is])...);
        }
    } // namespace Detail

    /**
     * @brief Registers systems and compiles them to a replayable runtime graph.
     *
     * A system is a kernel that declares which components it reads and writes; the
     * Schedule emits one graph node per matching chunk, keyed on that chunk's
     * column pointers. The runtime's dependency tracker then orders the systems by
     * their component access — conflicting systems run in order, disjoint ones (and
     * disjoint chunks of the same system) run in parallel — so no scheduler is
     * written here. Each node's iteration count is late-bound to its chunk's live
     * entity count, so spawning and destroying entities within existing chunks
     * varies the work without recompiling. The graph is rebuilt only when the
     * world's chunk set changes, which the structure version reports.
     */
    class Schedule
    {
        public:
            /**
             * @brief Creates a schedule bound to @p runtime.
             * @param runtime The runtime whose graph this builds and replays.
             */
            explicit Schedule(SushiRuntime::API::Runtime& runtime) : runtime_(runtime) {}

            /**
             * @brief Registers a system over the components named by @p Access.
             *
             * The kernel is called as `fn(i, ptr0, ptr1, ...)` where each pointer is
             * the column for the matching Access in declaration order — `const T*`
             * for Read<T>, `T*` for Write<T>. Systems run in registration order
             * where their access conflicts and in parallel where it does not.
             *
             * @tparam Access The Read<T>/Write<T> declarations defining the access set.
             * @tparam Fn     The kernel callable type.
             * @param name The system's name, for diagnostics.
             * @param fn   The per-element kernel.
             * @return *this, for chaining.
             */
            template <typename... Access, typename Fn>
            Schedule& each(std::string name, Fn fn)
            {
                Signature required = make_signature<typename Access::type...>();
                systems_.push_back(System{
                    std::move(name),
                    [required, fn](World& world, SushiRuntime::API::Graph& graph)
                    {
                        for (Archetype* a : world.query(required))
                            for (const std::unique_ptr<Chunk>& chunk : a->chunks())
                            {
                                Chunk* c = chunk.get();
                                std::vector<void*> reads, writes;
                                const std::array<void*, sizeof...(Access)> cols = {
                                    Detail::column_base<Access>(*c, reads, writes)...};

                                graph.add(
                                    SushiRuntime::API::sized([c] { return c->count(); }),
                                    reads, writes, world.chunk_capacity(),
                                    [fn, cols](std::size_t i)
                                    {
                                        Detail::invoke_system<Access...>(
                                            fn, i, cols,
                                            std::make_index_sequence<sizeof...(Access)>{});
                                    });
                            }
                    }});
                built_version_ = K_NEVER_BUILT;
                return *this;
            }

            /**
             * @brief Runs every system once, rebuilding the graph only if needed.
             *
             * Rebuilds and recompiles when the world's chunk set has changed since
             * the last build (a new archetype or chunk); otherwise replays the
             * compiled graph, which follows per-chunk count changes without a
             * recompile.
             *
             * @param world The world to step.
             * @return The run report for this step.
             */
            SushiRuntime::RunReport run(World& world)
            {
                if (!graph_ || built_version_ != world.structure_version())
                {
                    graph_.emplace(runtime_.graph());
                    for (System& s : systems_)
                        s.emit(world, *graph_);
                    built_version_ = world.structure_version();
                }
                if (graph_->size() == 0)
                    return SushiRuntime::RunReport{};
                return graph_->run();
            }

            /** @brief Number of registered systems. */
            std::size_t system_count() const noexcept { return systems_.size(); }

            /** @brief Times the current graph has been compiled (1 after warm-up). */
            std::size_t compile_count() const noexcept
            {
                return graph_ ? graph_->compile_count() : 0;
            }

        private:
            static constexpr std::uint64_t K_NEVER_BUILT = ~std::uint64_t(0);

            /** @brief A registered system: its name and its node-emitting closure. */
            struct System
            {
                std::string name;
                std::function<void(World&, SushiRuntime::API::Graph&)> emit;
            };

            SushiRuntime::API::Runtime& runtime_;
            std::vector<System> systems_;
            std::optional<SushiRuntime::API::Graph> graph_;
            std::uint64_t built_version_ = K_NEVER_BUILT;
    };
} // namespace SushiEngine
