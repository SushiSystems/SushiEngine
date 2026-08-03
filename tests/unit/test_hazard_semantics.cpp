/**************************************************************************/
/* test_hazard_semantics.cpp                                              */
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

// Unit_HazardSemantics: the normative hazard semantic of SushiEngine::Execution
// (execution/hazard.hpp), which every execution backend answers to. These cases pin the
// two floors the semantic is stated in — the safety floor (every conflicting pair is
// ordered) and the determinism floor (the answer depends only on declarations, never
// on addresses or timing) — plus the interval algebra both rest on.
//
// This is a conformance suite, not an implementation test: it deliberately drives the
// vocabulary rather than any one tracker, so the same cases can be re-pointed at the
// native backend's compiler (UHM1) and at the render graph's planner (UHM2) without
// being rewritten.

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/execution/hazard.hpp>

using namespace SushiEngine::Execution;

namespace
{
    // Two distinct allocations to key intervals on. Only their addresses are used —
    // the bytes are never touched, which is exactly how a tracker treats them.
    std::uint64_t storage_a[64] = {};
    std::uint64_t storage_b[64] = {};

    ResourceId allocation_a() { return static_cast<ResourceId>(storage_a); }
    ResourceId allocation_b() { return static_cast<ResourceId>(storage_b); }

    /** @brief A read of one byte range of an allocation, as a device-compute access. */
    ResourceAccess read_of(ResourceId base, std::uint64_t offset, std::uint64_t size)
    {
        return ResourceAccess{BufferInterval{base, offset, size}, AccessIntent::ComputeRead};
    }

    /** @brief A write of one byte range of an allocation, as a device-compute access. */
    ResourceAccess write_of(ResourceId base, std::uint64_t offset, std::uint64_t size)
    {
        return ResourceAccess{BufferInterval{base, offset, size}, AccessIntent::ComputeWrite};
    }

    /**
     * @brief The conflicting pairs of an access sequence, in declaration order.
     *
     * The observable the conformance contract is written against. Two trackers agree
     * when they order the same pairs, regardless of how many extra edges either one
     * emits, so the suite compares this rather than any edge set.
     */
    std::vector<std::pair<std::size_t, std::size_t>>
    conflicting_pairs(const std::vector<ResourceAccess>& accesses)
    {
        std::vector<std::pair<std::size_t, std::size_t>> pairs;
        for (std::size_t earlier = 0; earlier < accesses.size(); ++earlier)
            for (std::size_t later = earlier + 1; later < accesses.size(); ++later)
                if (accesses_conflict(accesses[earlier], accesses[later]))
                    pairs.emplace_back(earlier, later);
        return pairs;
    }
} // namespace

TEST(Unit_HazardSemantics, DistinctAllocationsNeverConflict)
{
    EXPECT_FALSE(accesses_conflict(write_of(allocation_a(), 0, 64),
                                   write_of(allocation_b(), 0, 64)));
}

TEST(Unit_HazardSemantics, DisjointSubRangesOfOneAllocationStayParallel)
{
    // The case the quality target exists for: an island-per-node solver declares many
    // disjoint slices of one column and must not serialize on the shared base pointer.
    EXPECT_FALSE(accesses_conflict(write_of(allocation_a(), 0, 32),
                                   write_of(allocation_a(), 32, 32)));
    EXPECT_TRUE(accesses_conflict(write_of(allocation_a(), 0, 33),
                                  write_of(allocation_a(), 32, 32)));
}

TEST(Unit_HazardSemantics, TouchingRangesDoNotOverlap)
{
    // Half-open intervals: [0, 32) and [32, 64) share no byte.
    EXPECT_FALSE(accesses_conflict(write_of(allocation_a(), 0, 32),
                                   read_of(allocation_a(), 32, 32)));
}

TEST(Unit_HazardSemantics, TwoReadsAreNotAHazard)
{
    EXPECT_FALSE(accesses_conflict(read_of(allocation_a(), 0, 64),
                                   read_of(allocation_a(), 0, 64)));
    EXPECT_EQ(classify_hazard(read_of(allocation_a(), 0, 64), read_of(allocation_a(), 0, 64)),
              HazardKind::None);
}

TEST(Unit_HazardSemantics, EveryConflictingPairIsClassified)
{
    const ResourceAccess read  = read_of(allocation_a(), 0, 64);
    const ResourceAccess write = write_of(allocation_a(), 0, 64);

    EXPECT_EQ(classify_hazard(write, read), HazardKind::ReadAfterWrite);
    EXPECT_EQ(classify_hazard(read, write), HazardKind::WriteAfterRead);
    EXPECT_EQ(classify_hazard(write, write), HazardKind::WriteAfterWrite);
}

TEST(Unit_HazardSemantics, WholeIntervalCoversEverySubRange)
{
    const ResourceAccess whole = write_of(allocation_a(), 0, BufferInterval::WHOLE);

    EXPECT_TRUE(accesses_conflict(whole, read_of(allocation_a(), 0, 8)));
    EXPECT_TRUE(accesses_conflict(whole, read_of(allocation_a(), 4096, 8)));
    EXPECT_TRUE(accesses_conflict(whole, write_of(allocation_a(), 0, BufferInterval::WHOLE)));
    EXPECT_FALSE(accesses_conflict(whole, read_of(allocation_b(), 0, 8)));
}

TEST(Unit_HazardSemantics, AnEmptyIntervalKeysOnNothing)
{
    // A column that failed to allocate must not silently order against every other
    // null-keyed one, which is what a bare-pointer tracker would do.
    const ResourceAccess null_keyed = write_of(nullptr, 0, BufferInterval::WHOLE);
    EXPECT_FALSE(accesses_conflict(null_keyed, null_keyed));
    EXPECT_FALSE(null_keyed.interval.buffer.valid());
}

TEST(Unit_HazardSemantics, HostAndComputeAccessesShareOneAlgebra)
{
    // The point of one algebra: a host job writing what a kernel reads is the same
    // read-after-write as one kernel feeding another, with no domain-specific case.
    const ResourceAccess host_write{BufferInterval{allocation_a(), 0, 64}, AccessIntent::HostWrite};
    EXPECT_EQ(classify_hazard(host_write, read_of(allocation_a(), 0, 64)),
              HazardKind::ReadAfterWrite);
}

TEST(Unit_HazardSemantics, RenderIntentsProjectOntoTheSameReadWriteSplit)
{
    EXPECT_TRUE(intent_writes(AccessIntent::ColorWrite));
    EXPECT_TRUE(intent_writes(AccessIntent::StorageWrite));
    EXPECT_TRUE(intent_writes(AccessIntent::DepthStencilWrite));
    EXPECT_FALSE(intent_writes(AccessIntent::SampledRead));
    EXPECT_FALSE(intent_writes(AccessIntent::IndirectRead));

    // And the boundary knows which half of the algebra it may not carry.
    EXPECT_TRUE(intent_is_render_only(AccessIntent::ColorWrite));
    EXPECT_FALSE(intent_is_render_only(AccessIntent::ComputeWrite));
    EXPECT_FALSE(intent_is_render_only(AccessIntent::HostRead));
    EXPECT_FALSE(intent_is_render_only(AccessIntent::TransferDestination));
}

TEST(Unit_HazardSemantics, TextureSubresourcesConflictOnlyWhereTheyIntersect)
{
    const TextureInterval mip0{0, 1, 0, 1, 0x1};
    const TextureInterval mip1{1, 1, 0, 1, 0x1};
    const TextureInterval all_mips{0, TextureInterval::ALL, 0, TextureInterval::ALL, 0x1};
    const TextureInterval stencil{0, 1, 0, 1, 0x2};

    EXPECT_FALSE(mip0.overlaps(mip1));
    EXPECT_TRUE(all_mips.overlaps(mip1));
    EXPECT_FALSE(mip0.overlaps(stencil));
    EXPECT_TRUE(mip0.overlaps(mip0));
}

TEST(Unit_HazardSemantics, BufferAndTextureRangesNeverConflict)
{
    const ResourceAccess buffer_write = write_of(allocation_a(), 0, 64);
    const ResourceAccess texture_write{
        TextureInterval{0, TextureInterval::ALL, 0, TextureInterval::ALL, 0x1},
        AccessIntent::ColorWrite};

    EXPECT_FALSE(accesses_conflict(buffer_write, texture_write));
}

TEST(Unit_HazardSemantics, NodeOrderingIsTheDisjunctionOverAccesses)
{
    const ResourceAccess first_accesses[] = {read_of(allocation_a(), 0, 32),
                                             write_of(allocation_b(), 0, 32)};
    const ResourceAccess disjoint_accesses[] = {write_of(allocation_a(), 32, 32)};
    const ResourceAccess overlapping_accesses[] = {read_of(allocation_b(), 16, 32)};

    NodeDescriptor first{};
    first.name = "first";
    first.accesses = first_accesses;
    first.access_count = 2;

    NodeDescriptor disjoint{};
    disjoint.name = "disjoint";
    disjoint.accesses = disjoint_accesses;
    disjoint.access_count = 1;

    NodeDescriptor overlapping{};
    overlapping.name = "overlapping";
    overlapping.accesses = overlapping_accesses;
    overlapping.access_count = 1;

    EXPECT_FALSE(nodes_conflict(first, disjoint));
    EXPECT_TRUE(nodes_conflict(first, overlapping));
}

TEST(Unit_HazardSemantics, TheAnswerDependsOnlyOnDeclarations)
{
    // The determinism floor. The same declarations produce the same conflicting pairs
    // on every evaluation, and reversing which allocation sits at the lower address
    // must not reorder anything — a tracker that sorted by pointer would fail this.
    const std::vector<ResourceAccess> sequence = {
        write_of(allocation_b(), 0, 32),
        read_of(allocation_a(), 0, 32),
        write_of(allocation_a(), 16, 32),
        read_of(allocation_b(), 0, 32),
        read_of(allocation_a(), 48, 16),
    };

    const std::vector<std::pair<std::size_t, std::size_t>> expected = {
        {0, 3}, // b: write then read
        {1, 2}, // a: read [0,32) then write [16,48)
    };

    EXPECT_EQ(conflicting_pairs(sequence), expected);
    EXPECT_EQ(conflicting_pairs(sequence), conflicting_pairs(sequence));
}

TEST(Unit_HazardSemantics, CosmeticOutputMayNotFeedABitwiseNode)
{
    EXPECT_TRUE(determinism_permits(DeterminismClass::Bitwise, DeterminismClass::Bitwise));
    EXPECT_FALSE(determinism_permits(DeterminismClass::Cosmetic, DeterminismClass::Bitwise));
    EXPECT_TRUE(determinism_permits(DeterminismClass::Tolerant, DeterminismClass::Bitwise));
    EXPECT_TRUE(determinism_permits(DeterminismClass::Cosmetic, DeterminismClass::Cosmetic));
    EXPECT_TRUE(determinism_permits(DeterminismClass::Cosmetic, DeterminismClass::Tolerant));
}

TEST(Unit_HazardSemantics, ProvidersReadThroughWithoutAllocating)
{
    struct Counter
    {
        std::size_t value = 0;
        std::size_t count() const noexcept { return value; }
    };

    Counter counter{7};
    const CountProvider provider = CountProvider::bind<&Counter::count>(&counter);

    EXPECT_TRUE(provider.bound());
    EXPECT_EQ(provider.read(0), 7u);

    counter.value = 11;
    EXPECT_EQ(provider.read(0), 11u);

    const CountProvider unbound{};
    EXPECT_FALSE(unbound.bound());
    EXPECT_EQ(unbound.read(1024), 1024u);

    // A temporary lambda is bound by value, so a call site that emits nodes in a loop
    // is not quietly holding a dangling callable.
    CountProvider owning = [] { return std::size_t(42); };
    EXPECT_EQ(owning.read(0), 42u);

    // Trivially copyable and small: a graph rebuild copies these by value and
    // allocates nothing for them.
    static_assert(std::is_trivially_copyable<CountProvider>::value,
                  "providers must be copyable into a backend's late-binding record");
    EXPECT_LE(sizeof(CountProvider), sizeof(void*) + CountProvider::CAPACITY);
}
