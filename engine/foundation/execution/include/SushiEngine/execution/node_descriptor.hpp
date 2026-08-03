/**************************************************************************/
/* node_descriptor.hpp                                                    */
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
#include <cstdint>
#include <new>
#include <type_traits>

#include <SushiEngine/execution/access.hpp>
#include <SushiEngine/execution/interval.hpp>

namespace SushiEngine
{
    namespace Execution
    {
        /**
         * @brief One declared touch of one resource range by one node.
         *
         * The unit the hazard semantic operates on: a range plus what the node does to
         * it. Everything a conforming tracker needs to order two nodes is here, which is
         * why the same declaration serves a thread pool, a device task graph, and a
         * render backend without reinterpretation.
         */
        struct ResourceAccess
        {
            ResourceInterval interval{};                       /**< The range touched. */
            AccessIntent     intent = AccessIntent::HostRead;  /**< What the node does to it. */
        };

        /**
         * @brief What kind of work a node described this way performs.
         *
         * Only the kinds a descriptor can actually express. A fixed-order fold is a
         * graph verb of its own (`Graph::add_reduce`) rather than a kind listed here,
         * because it takes two buffers and a combiner instead of an access list and a
         * per-element kernel — describing it through this struct would mean carrying
         * fields every other node ignores.
         */
        enum class NodeKind : std::uint8_t
        {
            Parallel, /**< A per-element kernel over a late-bound live range. */
            Host,     /**< A single host callable. */
        };

        /**
         * @brief A late-bound value read once per run, without allocating.
         *
         * Node counts, enablement, and window bases are not known at graph-build time —
         * they follow live entity counts and solver bands that change every tick. This
         * stores the callable that answers *by value*, in fixed inline storage: a graph
         * rebuild allocates nothing, and a caller may pass a temporary lambda without
         * having to keep it alive itself — the failure a pointer-to-callable would
         * invite at exactly the call sites that emit nodes in a loop.
         *
         * The callable must be trivially copyable and fit the inline capacity. That is
         * the same requirement a device backend imposes anyway, since a provider a
         * kernel launch has to carry cannot own heap storage; both are checked at the
         * call site rather than discovered later.
         *
         * @tparam Value The type the provider yields.
         */
        template <typename Value>
        class Provider
        {
            public:
                /** @brief Bytes of inline storage available to the bound callable. */
                static constexpr std::size_t CAPACITY = 32;

                /** @brief Constructs an unbound provider, meaning "use the default". */
                Provider() = default;

                /**
                 * @brief Binds a callable, copying it into the inline storage.
                 *
                 * Implicit so a call site reads `node.count = [this] { return live_; }`
                 * instead of naming the provider type twice.
                 *
                 * @tparam Callable The callable's type; invoked with no arguments.
                 * @param callable The callable to copy in.
                 */
                template <typename Callable,
                          typename = std::enable_if_t<
                              !std::is_same<std::decay_t<Callable>, Provider>::value>>
                Provider(const Callable& callable) noexcept
                {
                    static_assert(sizeof(Callable) <= CAPACITY,
                                  "a late-bound provider must fit its inline storage; "
                                  "capture fewer values");
                    static_assert(alignof(Callable) <= alignof(std::max_align_t),
                                  "a late-bound provider must not be over-aligned");
                    static_assert(std::is_trivially_copyable<Callable>::value,
                                  "a late-bound provider must be trivially copyable, so a "
                                  "backend can carry it into a kernel launch");
                    static_assert(std::is_trivially_destructible<Callable>::value,
                                  "a late-bound provider must be trivially destructible");

                    invoke_ = [](const void* storage) -> Value
                    {
                        return (*static_cast<const Callable*>(storage))();
                    };
                    new (static_cast<void*>(storage_)) Callable(callable);
                }

                /** @brief True when a value is bound; unbound means "use the default". */
                bool bound() const noexcept { return invoke_ != nullptr; }

                /**
                 * @brief Reads the current value.
                 * @param fallback Returned when nothing is bound.
                 * @return The bound callable's current answer, or @p fallback.
                 */
                Value read(Value fallback) const
                {
                    return invoke_ ? invoke_(storage_) : fallback;
                }

                /**
                 * @brief Binds a const member function of an object that outlives the graph.
                 *
                 * The common case — a chunk's live count, a band's size — spelled without
                 * a lambda, so the stored callable is exactly one pointer.
                 *
                 * @tparam Method The member function to call; deduced including noexcept.
                 * @tparam Object The object type owning @p Method.
                 * @param object The instance to read from; must outlive the graph.
                 * @return A provider reading @p object through @p Method.
                 */
                template <auto Method, typename Object>
                static Provider bind(const Object* object) noexcept
                {
                    return Provider([object] { return (object->*Method)(); });
                }

            private:
                Value (*invoke_)(const void*) = nullptr;
                alignas(alignof(std::max_align_t)) unsigned char storage_[CAPACITY] = {};
        };

        /** @brief Yields a node's live element count for this run. */
        using CountProvider = Provider<std::size_t>;

        /** @brief Yields the first index of a node's window for this run. */
        using BaseProvider = Provider<std::size_t>;

        /** @brief Yields whether a node runs at all this run. */
        using EnabledProvider = Provider<bool>;

        /**
         * @brief Everything a graph needs to know about a node except its kernel.
         *
         * Kept separate from the kernel on purpose: the kernel stays a template
         * parameter all the way into the backend, so a device backend can forward the
         * raw callable into its own launch without type erasure, while this description
         * — which no device ever sees — stays a plain struct any backend can inspect.
         *
         * The access array is not owned. It must stay alive across the submission call
         * only; backends copy what they need out of it.
         */
        struct NodeDescriptor
        {
            const char*           name         = nullptr;               /**< Diagnostic name; not owned. */
            const ResourceAccess* accesses     = nullptr;               /**< Declared accesses; not owned. */
            std::size_t           access_count = 0;                     /**< Number of declared accesses. */
            std::size_t           capacity     = 0;                     /**< Upper bound the live count must fit. */
            NodeKind              kind         = NodeKind::Parallel;    /**< The work this node performs. */
            DeterminismClass      determinism  = DeterminismClass::Bitwise; /**< The node's replay promise. */
            CountProvider         count{};                              /**< Live count; unbound means capacity. */
            BaseProvider          base{};                               /**< Window start; unbound means zero. */
            EnabledProvider       enabled{};                            /**< Enablement; unbound means always. */

            /** @brief First declared access, for range-based iteration. */
            const ResourceAccess* begin() const noexcept { return accesses; }

            /** @brief One past the last declared access. */
            const ResourceAccess* end() const noexcept { return accesses + access_count; }
        };
    } // namespace Execution
} // namespace SushiEngine
