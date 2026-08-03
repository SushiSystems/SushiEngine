/**************************************************************************/
/* hazard.hpp                                                             */
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

#include <SushiEngine/execution/access.hpp>
#include <SushiEngine/execution/interval.hpp>
#include <SushiEngine/execution/node_descriptor.hpp>

/**
 * @file hazard.hpp
 * @brief The normative hazard semantic every execution backend implements.
 *
 * One semantic, several implementations. A backend may infer node ordering however
 * it likes — an append-only access history, a last-writer table, a colouring pass —
 * but every implementation answers to the same three obligations, and conformance is
 * defined as agreement on which pairs are ordered, never as producing the same edges.
 *
 * **Safety floor (normative, every implementation).** Every conflicting pair of
 * accesses — read-after-write, write-after-read, write-after-write over overlapping
 * ranges of one resource — is ordered. Extra, conservative edges are permitted; a
 * missing edge is not. An extra edge costs parallelism, a missing one is a data race,
 * and every simplification is required to fall on the first side.
 *
 * **Determinism floor (normative, every implementation).** The produced ordering is a
 * pure function of declaration order and declared ranges. It must never depend on
 * pointer values, hash iteration order, thread timing, worker count, or allocator
 * behaviour, because the simulation domain replays a compiled graph during rollback
 * and a schedule that varies between builds cannot reproduce a tick.
 *
 * **Quality target (the engine's own tracker only).** Ordering is inferred in time
 * linear in the declared accesses, by keeping the last writer and the reader set per
 * interval, so workloads that declare many disjoint sub-ranges of one allocation do
 * not degrade into a quadratic scan. This is a target, not a contract: a tracker that
 * orders strictly more pairs than the engine's still conforms.
 */

namespace SushiEngine
{
    namespace Execution
    {
        /**
         * @brief Why two accesses must be ordered, or that they need not be.
         *
         * Named rather than reduced to a boolean because the three hazards do not all
         * cost the same to resolve on every backend — a write-after-read can be
         * satisfied by execution ordering alone, while a read-after-write also needs the
         * write's results made visible — and a backend lowering intents to barriers has
         * to tell them apart.
         */
        enum class HazardKind : std::uint8_t
        {
            None,            /**< Disjoint ranges, or two reads: no ordering required. */
            ReadAfterWrite,  /**< The later access reads what the earlier one wrote. */
            WriteAfterRead,  /**< The later access overwrites what the earlier one read. */
            WriteAfterWrite, /**< Both write the same range; the later result must win. */
        };

        /**
         * @brief Reports whether two accesses must be ordered against each other.
         *
         * The predicate the safety floor is stated in: overlapping ranges where at least
         * one side writes. Two reads of the same bytes are not a hazard, and
         * non-overlapping ranges are never a hazard however they are accessed.
         *
         * @param first  One declared access.
         * @param second The other declared access.
         * @return True when the pair conflicts and therefore must be ordered.
         */
        constexpr bool accesses_conflict(const ResourceAccess& first,
                                         const ResourceAccess& second) noexcept
        {
            if (!intent_writes(first.intent) && !intent_writes(second.intent))
                return false;
            return first.interval.overlaps(second.interval);
        }

        /**
         * @brief Classifies the hazard between two accesses in declaration order.
         *
         * Order matters: the same pair is a read-after-write one way round and a
         * write-after-read the other, so the arguments name the earlier and later
         * declaration rather than an unordered pair.
         *
         * @param earlier The access declared first.
         * @param later   The access declared afterwards.
         * @return The hazard the pair forms, or None when they may run unordered.
         */
        constexpr HazardKind classify_hazard(const ResourceAccess& earlier,
                                             const ResourceAccess& later) noexcept
        {
            if (!accesses_conflict(earlier, later))
                return HazardKind::None;

            const bool earlier_writes = intent_writes(earlier.intent);
            const bool later_writes   = intent_writes(later.intent);

            if (earlier_writes && later_writes) return HazardKind::WriteAfterWrite;
            if (earlier_writes)                 return HazardKind::ReadAfterWrite;
            return HazardKind::WriteAfterRead;
        }

        /**
         * @brief Reports whether two nodes must be ordered against each other.
         *
         * Node-level ordering is the disjunction over their declared accesses: one
         * conflicting pair is enough, and finding it early is why this stops at the
         * first hit rather than classifying every pair.
         *
         * @param earlier The node declared first.
         * @param later   The node declared afterwards.
         * @return True when at least one pair of their accesses conflicts.
         */
        inline bool nodes_conflict(const NodeDescriptor& earlier,
                                   const NodeDescriptor& later) noexcept
        {
            for (const ResourceAccess& first : earlier)
                for (const ResourceAccess& second : later)
                    if (accesses_conflict(first, second))
                        return true;
            return false;
        }

        /**
         * @brief Reports whether a node may consume another's output under replay.
         *
         * The determinism rule made checkable: a bitwise-deterministic node reading a
         * value produced by a cosmetic one inherits that node's freedom to vary, which
         * silently breaks rollback reconciliation. Only that one pairing is rejected —
         * a tolerant producer has a stated reproducibility bound the consumer's own
         * conformance suite can hold it to, whereas a cosmetic one has none at all.
         *
         * @param producer The determinism class of the node that wrote the data.
         * @param consumer The determinism class of the node reading it.
         * @return True when the dependency is legal, false when it would taint the consumer.
         */
        constexpr bool determinism_permits(DeterminismClass producer,
                                           DeterminismClass consumer) noexcept
        {
            return !(consumer == DeterminismClass::Bitwise &&
                     producer == DeterminismClass::Cosmetic);
        }
    } // namespace Execution
} // namespace SushiEngine
