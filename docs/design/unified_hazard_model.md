# Unified Hazard Model (UHM) — one execution vocabulary for sim, compute, and render

**Status:** designed, 2026-08-01.

This is the "dedicated design pass" that `cross_platform_engineering_plan.md` §3.1 defers — the
shared sim+render hazard vocabulary, the tracker discipline, and the interop contract at the domain
boundary. It resolves that document's §9 risk-register entry ("hazard vocabularies diverge before a
shared model is designed") and its open go/no-go decision on gating `RUNTIME-PORT0`/`RHI0`. Design
authority for this pass was delegated by the owner on 2026-08-01.

**Companion documents.** `cross_platform_engineering_plan.md` (the three walls; `Execution` seam §4,
RHI §5); `sushiruntime/docs/design/ENGINE_BACKBONE_REFACTOR.md` (BB-0…BB-8 — this design *composes
with* BB-1a/1b and BB-7, it does not re-plan them); `physics_system.md` §18 (the runtime request
register); `atmosphere_system.md` Phase B3 (the recorded SYCL rejection this design treats as
doctrine); `SUSHILOOP.md` (the determinism contract the sim domain answers to).

---

## 0. The verdict in one paragraph

"Unified" means **one access algebra, one tracker semantic, and one boundary contract** — it does
not mean one graph object. The sim graph (compile-once, replayed per fixed tick, bit-deterministic,
rollback-replayable) and the render graph (rebuilt per presented frame, throughput-shaped, never
re-recorded during rollback) stay two schedule domains, because their cadence and their determinism
obligations are irreconcilable in one DAG — §3.1 already suspected this, and the ground truth below
confirms it. What becomes *one* is: the vocabulary a node uses to declare an access (`AccessIntent`
over a resource interval), the hazard-inference semantic every compiler applies to those
declarations (last-writer/reader-set, RAW/WAR/WAW, deterministic tie-break), the determinism class
stamped on every node, and the epoch-published `Handoff` through which the only legal cross-domain
traffic flows. Each domain keeps its native resource currency in its hot path — pointers and byte
intervals sim-side, virtual handles render-side — and the sum type §3.1 asked about lives in exactly
one place: the boundary registry, where it is cold.

---

## 1. Ground truth this design stands on

Verified against both trees on 2026-08-01. Five facts are load-bearing; everything below follows
from them.

**G1 — The sim domain's hazard currency is byte intervals over USM columns.** A chunk column is
`SushiRuntime::API::Buffer<std::byte>` allocated with `runtime.buffer<std::byte>(capacity * size)`
(`ecs/chunk.hpp:71`); `Schedule::each` emits one node per chunk whose dependency keys are the raw
column base addresses — whole-buffer regions today (`ecs/schedule.hpp:146-164`,
`Buffer::dependency_key()`), while the runtime's `Core::ResourceRegion`
(`{ResourceId base, byte offset, byte length}`, half-open, `WHOLE` sentinel) and the R3
interval-boundary work (merged to `main` in `f801cc5`) make interval-exact sub-range tracking real
where it is declared — the fixed-order reduce's levels and `DynamicGraph`'s island regions are the
two in-tree proofs. The system kernels themselves are plain C++ over those pointers — no SYCL in any
kernel body. One forward-compatibility fact matters to the vocabulary: `Core::ResourceId` is `void*`
today but is *explicitly aliased so it can become an opaque registered id*
(`resource_region.hpp:39`, WP-2's unfinished half) — the engine's mirror type must typedef its key
for the same reason.

**G1b — The demanding sim consumer is no longer only the ECS.**
`physics/solver/runtime_graph_builder.hpp` (created 2026-07-28, after the cross-platform plan's
ground-truth pass; 42 `SushiRuntime::` usages) is the deepest coupling site in the engine and uses a
far wider graph surface than `Schedule` does: `ElementRange` sub-buffer accesses, `API::when(...)`
predicated nodes, `API::sized(...)` late-bound counts, `Residency`/`DeviceIndex` placement. Any
`Execution` vocabulary that cannot express these cannot retarget the physics graph — §4.5 addresses
each.

**G2 — The render domain's hazard currency is typed access intents over opaque virtual resources.**
A pass declares `TextureAccess`/`BufferAccess` against frame-scoped virtual handles
(`render/graph/resource_handle.hpp:96-126`), and the graph's *entire* barrier knowledge is one total
function from those intents to the Vulkan `(stage, access, layout)` triple
(`render/graph/resource_state.hpp:26-34`). Whole-resource granularity, re-declared every frame,
usage flags inferred by unioning intents. This is already an access-intent model — the render domain
needs **no semantic change** to participate in UHM, only a vocabulary alignment at RHI1.

**G3 — The boundary already has a designed direction.** The renderer *exports* allocations
(`render/interop.hpp`: UUID-matched external memory, Win32 handle / POSIX fd, "the importing half
belongs to whoever owns the other API"); SushiRuntime's import half plus completion export are
BB-1a/BB-1b — planned, unbuilt, with the engine's `vulkan_interop_buffer.cpp` export path sitting
callerless until they land. UHM's boundary contract is the engine-side orchestration of exactly
those two seams, not a third mechanism. Note the correction to the cross-platform plan's §3 wording:
on desktop, zero-copy is **renderer-exports / runtime-imports** external memory — not USM columns
handed raw to Vulkan. The USM-column-as-graphics-memory shape appears only on the native-`Execution`
UMA path (§6, tier T3).

**G4 — Rollback constrains publication, not tracking.** `Loop::App` owns the fixed-step loop and a
`RollbackBuffer` that snapshots/restores columns generically — full column copies, not deltas
(`loop/rollback.hpp:96-121`; per-write dirty tracking is a recorded follow-on, not present); a
reconciliation replays from the earliest mismatched tick through the *same compiled graph*
(`Schedule::run` rebuilds only on `structure_version` change), with the structural constraint that
no spawn/destroy may occur between capture and restore (`rollback.hpp:33-36`). The render domain
must never observe an intermediate replayed tick — which makes "when may the render domain see sim
output" a publication rule at the boundary, not a property of either tracker. Note also that today's
shipping sim→render seam is a *value snapshot*: the editor never instantiates `Loop::App` at all —
it hosts `ISimulation` and reads `render_scene()`, a copied `RenderScene` struct, once per host
frame (`editor/main.cpp:439-461`). The copy-shaped host round-trip BB-1 describes is therefore the
*current* T0 reality, not a regression risk.

**G5 — The engine already runs three compute lanes, by recorded doctrine.** SushiRuntime SYCL for
the deterministic sim domain; Vulkan compute for render-resource computation (atmosphere's recorded
rejection: *"its value is in the deterministic simulation domain, where the data does not have to
become something the rasteriser samples"* — `atmosphere_system.md` Phase B3); CPU for host systems
and fallbacks. UHM does not merge these lanes — it gives the first lane's output a typed,
epoch-fenced route into the second's inputs, and it gives the future `Execution` RHI-compute backend
(`RUNTIME-PORT8`) a vocabulary that already speaks GPU barriers.

---

## 2. Design principles

**U1 — The engine owns the vocabulary; every runtime is a backend.** The UHM types live in
`include/SushiEngine/execution/` with zero includes of SushiRuntime, SYCL, or Vulkan headers. The
SushiRuntime path consumes them through a thin adapter inside the existing SYCL TU; the render graph
consumes them through `Rhi` at RHI1; the native backend consumes them directly. This is the organic
shape of the relationship: SushiRuntime plugs into a socket the engine defines — the engine is never
wired into shapes the runtime defines. The one-way `SushiEngine → SushiRuntime` arrow
(`docs/architecture/overview.md` §1) is untouched, and the runtime's API instability stops mattering
to 51 of the 52 files it can currently reach.

**U2 — Unify algebra, not hot paths.** No hot path is retyped to a generic handle. Sim nodes keep
pointer+interval keys (trivial lowering to `Core::ResourceRegion`); render passes keep
`TextureHandle`/`BufferHandle`. The cross-domain sum type exists only in the `Handoff` registry,
which is touched at registration and publication — cold, per-resource, not per-node-per-tick.

**U3 — Cross-domain traffic is buffer-shaped, by doctrine.** BB-1 explicitly excludes importing
Vulkan *images* into the runtime, and the atmosphere rejection explains why the reverse direction is
also wrong. Therefore the boundary subset of the vocabulary is `BufferRegion` only. Image
subresources exist in the vocabulary (the render domain needs them; a future RHI-compute sim backend
needs them) but a `Handoff` registration of an image is a compile-time error. This single scoping
decision dissolves most of §3.1's "vocabulary mismatch" concern: the superset type exists, but the
shared wire format is the narrow, well-understood half.

**U4 — Determinism is a per-node class, enforced at the boundary.** `SUSHILOOP.md` locks
bit-determinism for the sim; render nodes must not pay for it and must not poison it. The class
marker is in the vocabulary from day one (§3.1's requirement), and the rules it implies (§7) are
mechanical, assertable, and cheap.

**U5 — Zero cost on today's shipping paths, measured.** Every UHM milestone carries an exit
criterion phrased as "nothing regressed": byte-identical sim output logs, golden images unchanged
(once RHI0 exists), zero steady-state allocations, capture-verified absence of staging copies. A
unification that taxes the shipping Vulkan+SYCL desktop path to subsidize a hypothetical platform is
rejected by construction.

---

## 3. The model at a glance

```
                 ┌────────────────────── SushiEngine::Execution (vocabulary, engine-owned) ───────────────────────┐
                 │  AccessIntent · ResourceInterval · DeterminismClass · NodeDescriptor · the hazard semantic     │
                 └──────┬───────────────────────────┬───────────────────────────────┬──────────────────────────┘
                        │                           │                               │
              SIM DOMAIN (one graph,       RENDER DOMAIN (one graph,        BOUNDARY (Execution::Handoff)
              compiled once, replayed      rebuilt per frame, barriers      registry of shared buffers,
              per fixed tick)              from the same intent table)      epoch publish/acquire
                        │                           │                               │
        ┌───────────────┼──────────────┐            │                    ┌──────────┴──────────┐
   RuntimeBackend   NativeBackend   [PORT8]     Vulkan RHI          desktop tier:         UMA tier:
   (SYCL adapter,   (thread pool,   (RHI        (resource_state     render exports,       RHI allocator
   lowers to        shared hazard   compute,    table relocated     runtime imports       feeds Execution
   ResourceRegion)  core)           AOT)        behind ICommandList) (BB-1a) + semaphore  columns
                                                                    export (BB-1b)       (allocator hook)
```

Two graphs, three backends on the left, one backend family on the right, one narrow waist between
them. The waist is the design.

---

## 4. The vocabulary

Header tree: `include/SushiEngine/execution/` — `access.hpp` (intents, classes), `interval.hpp`
(regions), `hazard.hpp` (the semantic, §5), `handoff.hpp` (§6). Header-only, freestanding, no engine
dependencies beyond `<cstdint>`/`<cstddef>` — these headers must compile in a SYCL TU, a stock-clang
TU, and (later) an NDK TU without friction.

### 4.1 Access intents

```cpp
namespace SushiEngine::Execution
{
    /// How a node touches a resource. The one enum every domain declares in.
    /// Sim backends consume only the read/write projection today; the render
    /// domain and the future RHI-compute backend consume the full type.
    enum class AccessIntent : std::uint8_t
    {
        HostRead, HostWrite,                    // CPU job accesses
        ComputeRead, ComputeWrite,              // device-compute accesses (SYCL or RHI compute)
        TransferSrc, TransferDst,               // copies, either engine
        // render-domain-only intents (rejected by Handoff registration, §6):
        SampledRead,                            // sampled by any shader stage
        StorageRead, StorageWrite,              // storage image/buffer by any shader stage
        ColorWrite,                             // colour attachment
        DepthStencilRead, DepthStencilWrite,    // depth/stencil attachment
        IndirectRead, VertexRead, IndexRead,    // fixed-function consumption
        Present,                                // handed to the presentation engine
    };

    constexpr bool intent_writes(AccessIntent) noexcept;   // the projection every tracker uses
    constexpr bool intent_is_host(AccessIntent) noexcept;  // boundary visibility decisions
    constexpr bool intent_is_render_only(AccessIntent) noexcept; // Handoff rejection predicate
}
```

The render graph's `TextureAccess`/`BufferAccess` are **projections** of this enum, not parallel
inventions: at RHI1, each existing value gains a documented mapping (e.g.
`TextureAccess::StorageComputeReadWrite` ⇒ `{StorageRead, StorageWrite}`, `BufferAccess::HostRead` ⇒
`HostRead`) and `resource_state.cpp`'s total function is re-keyed on the shared enum — the same
table, relocated exactly as the cross-platform plan §5.3 already requires for Metal. The
stage-qualified distinctions the current render enums carry (`SampledFragment` vs `SampledCompute`)
become a `StageMask` field beside the intent rather than enum values, which is what Metal and
console barrier models want anyway.

### 4.2 Resource intervals

```cpp
    /// The key a buffer interval is identified by. An address today — but a
    /// typedef, for the same reason SushiRuntime aliases Core::ResourceId
    /// (resource_region.hpp:39): the runtime's WP-2 plans to move identity to
    /// an opaque registered id, and the engine's mirror must be able to follow
    /// with a one-line change instead of a migration.
    using ResourceId = const void*;

    /// A byte interval — the sim domain's native currency and the only shape
    /// that crosses the domain boundary (U3). Structurally mirrors
    /// SushiRuntime::Core::ResourceRegion ({base, offset, length}, half-open,
    /// whole-buffer sentinel) so the RuntimeBackend adapter is a member-wise
    /// conversion, never a translation layer.
    struct BufferInterval
    {
        ResourceId    base;     // owning allocation = identity key
        std::uint64_t offset;   // bytes from base
        std::uint64_t size;     // interval length; WHOLE sentinel = to end
    };

    /// An image subresource rectangle — render-domain and PORT8 only.
    struct TextureInterval
    {
        std::uint16_t mip_base, mip_count;
        std::uint16_t layer_base, layer_count;
        std::uint8_t  aspects;  // projection of the RHI aspect mask
    };

    /// The §3.1 sum type. Lives in NodeDescriptor declarations and the Handoff
    /// registry; never in a kernel signature or a per-element path.
    struct ResourceInterval
    {
        enum class Kind : std::uint8_t { Buffer, Texture } kind;
        union { BufferInterval buffer; TextureInterval texture; };
    };
```

### 4.3 Determinism classes

```cpp
    /// What a node's output promises. Stamped per node at registration;
    /// defaulted per domain (sim ⇒ Bitwise, render ⇒ Cosmetic) so the
    /// common case writes nothing.
    enum class DeterminismClass : std::uint8_t
    {
        Bitwise,   // bit-identical under replay on the same binary+device (SUSHILOOP contract)
        Tolerant,  // cross-backend comparable within stated tolerance (the 1e-9 conformance tier)
        Cosmetic,  // no replay obligation; must never feed a Bitwise node (§7)
    };
```

This formalizes a split the engine already lives by — VFX's GPU-cosmetic vs CPU-deterministic
backends, skinning's Vulkan path with the SYCL kernel as a correctness oracle — rather than
inventing a new taxonomy.

### 4.4 Node declaration

```cpp
    /// What Schedule/solvers hand a graph per node. Replaces the raw
    /// (reads, writes) pointer vectors in Schedule::each at RUNTIME-PORT0
    /// with no change to kernel bodies.
    struct NodeDescriptor
    {
        const char*                     name;
        std::span<const ResourceAccess> accesses;   // {ResourceInterval, AccessIntent} pairs
        std::size_t                     capacity;    // grid size the count is late-bound under
        DeterminismClass                determinism; // defaulted by the emitting domain
        // count provider / enabled provider carried as in plan §4.3 (sized(), based_at()).
    };
```

`Schedule::each`'s emission today pushes bare `void*` into `reads`/`writes`
(`ecs/schedule.hpp:150-158`); under UHM it pushes
`{BufferInterval{base, 0, capacity * elem_size}, Read ? ComputeRead : ComputeWrite}`. The
RuntimeBackend lowers that to exactly the pointer regions SushiRuntime tracks today — byte-for-byte
the same dependency behaviour, which is what keeps RUNTIME-PORT0's "byte-identical output logs" exit
criterion honest.

### 4.5 The vocabulary must carry what `runtime_graph_builder.hpp` already uses

The physics graph builder (G1b) is the retarget stress test, and each of its constructs has a
designed home so `Execution` does not silently become "the ECS subset":

| Construct in use today | UHM/`Execution` answer |
|---|---|
| `Buffer::region(ElementRange)` sub-buffer accesses | `BufferInterval` with non-zero offset — already the vocabulary's native shape. |
| `API::sized(...)` late-bound counts | `NodeDescriptor`'s count provider (plan §4.3, kept). |
| `sized_from_device` / `based_at_device` (R1/R6 device-slot reads) | Modeled abstractly as *providers with a backend-realized source*; the SYCL backend reads the device slot, the native backend reads the same slot as host memory (on the native backend "device" slots are host-visible by construction). Not exposed as SYCL types. |
| `API::when(...)` predicated nodes | `NodeDescriptor`'s enabled provider (plan §4.3, kept). |
| `add_reduce` / `add_segmented_reduce` (R2, fixed-order) | First-class node kinds — `NodeKind::{Parallel, Host, Reduce, SegmentedReduce}` — so the engine can delete its two-node hand reduction once (BB-0 adoption) and the native backend supplies a serial/fixed-tile fold meeting the same layout-only ordering contract. |
| `Residency` / `DeviceIndex` placement | `Execution::MemoryVisibility` covers residency; explicit device indices stay behind `BackendCapabilities` (multi-device is a SushiRuntime-only capability until proven otherwise — the seam must not pretend portability it doesn't have). |

---

## 5. The tracker: one semantic, N implementations, one shared core where it pays

The hazard **semantic** is specified once, normatively, in `execution/hazard.hpp`'s documentation
and a conformance test. Stating it precisely matters, because the two shipping reference points do
*not* run the same algorithm and a naive "mirror the runtime" instruction is self-contradictory (the
cross-platform plan §4.3 asked for both "mirror `dependency_tracker.cpp`" and "linear-time" — the
runtime's tracker is an append-only per-shard access history with a backward overlap scan that
terminates only on a *fully covering* prior write: effectively O(1) amortized for whole-buffer keys,
but O(A²) per allocation exactly in the disjoint-sub-range case islands and chunk colours produce).
UHM resolves this by splitting the contract in two:

- **Safety floor (normative, all implementations):** every conflicting pair — RAW, WAR, WAW over
  overlapping intervals of one resource — is ordered. Conservative *extra* edges are permitted;
  missing edges are not. This is the runtime's own recorded doctrine
  (`dynamic_graph.hpp:582-584`: *"an extra cross edge costs parallelism; a missing one is a data
  race. Every simplification here is chosen to fall on the first side"*), adopted as the UHM
  correctness floor.
- **Determinism floor (normative):** the produced edge set and schedule are a pure function of
  declaration order and declared regions — never of pointer values, hash-iteration order, thread
  timing, or shard collisions. (The runtime satisfies this today: its shard chains are chronological
  per allocation and edges come only from interval overlap.)
- **Quality target (engine's shared core only):** last-writer-per-interval + reader-set
  bookkeeping, linear-ish in declared accesses, so disjoint sub-range workloads do not pay the
  O(A²) scan. A tighter-but-still-safe edge set than the runtime's is allowed by the safety
  floor; conformance is therefore defined as **ordering-equivalence over conflicting pairs**,
  not edge-set equality.

Implementations:

| Where | What | Code sharing |
|---|---|---|
| SushiRuntime (`dependency_tracker.cpp`) | meets the safety+determinism floors today (history-scan algorithm, interval-exact where regions are declared) | **None — knowledge dependency only.** The one-way arrow stays; the runtime is not asked to host engine types. Its own WP-2 (`resource_id` sharding) improves its constants independently. |
| `sushi_exec_native` (RUNTIME-PORT1) | the native backend's compiler | Instantiates the engine's shared `hazard core` (a small header template over `{Key, Interval, Intent}` traits) built to the quality target. |
| Render graph barrier planner | today's per-frame compile pass | Migrates onto the same shared core **during RHI2**, when the pass-recording layer is already being rebuilt behind `ICommandList` and RHI0's harness exists to prove the migration is a no-op. Not before: rewriting a shipping barrier planner with no golden images would be the exact mistake the cross-platform plan spends §5.1 warning about. |
| RHI-compute sim backend (RUNTIME-PORT8) | barrier emission for AOT compute sim | The shared core plus the same intent→barrier table the render backend uses — this is where carrying full `AccessIntent` in sim `NodeDescriptor`s from day one pays off. |

The conformance test (engine-side, `tests/functional/unit/test_hazard_semantics.cpp`) drives the
shared core and — through a recorded-order probe graph — the RuntimeBackend with identical access
sequences and asserts **every conflicting pair is ordered identically observable-side** (schedule
equivalence over conflicts, per the floors above), plus schedule determinism across repeated builds.
That is how "one semantic" stays true across implementations that deliberately share no code with
the runtime — and how a schedule-shape difference between a tighter engine core and the runtime's
conservative boundary layer stays a measured fact instead of a surprise.

---

## 6. The boundary: `Execution::Handoff`

The only legal route for data crossing domains. It is small on purpose: a registry, an epoch
counter per entry, and two verbs.

### 6.1 Contract

```cpp
    /// One shared buffer's boundary record. Registration is cold; publish and
    /// acquire are per-tick/per-frame and allocation-free.
    class Handoff
    {
    public:
        // Registration (startup / structure change only). Rejects render-only
        // intents and any TextureInterval (U3). `visibility` selects the tier (§6.3).
        HandoffId register_buffer(BufferInterval whole, DeterminismClass, Visibility visibility);

        // Sim side. Called once per *presented* simulation state — after the
        // final tick of a rollback replay, never per replayed tick (G4).
        void publish(HandoffId, Epoch tick, CompletionToken done);

        // Render side. Returns the newest published epoch and the token the
        // consumer must wait on (or fold into a queue wait, tier-dependent).
        Acquired acquire(HandoffId) const;
    };
```

- **Single-writer rule:** the publishing domain is fixed at registration. The render domain may only
  `acquire` sim-published entries. Feedback (readbacks, gameplay mirrors of GPU state — the weather
  pattern) flows through a *separate* registration published by the render/extract side with
  `DeterminismClass::Tolerant` at frame boundaries; a `Bitwise` sim node reading such an entry is a
  debug-build assertion (§7). There is no bidirectional entry, by construction.
- **Epoch = simulation tick.** `publish` after the last replayed tick only, so a rollback
  re-simulation is invisible to the render domain: it keeps rendering the last published epoch
  until a newer one appears (rate decoupling and rollback safety in one rule).
- **`CompletionToken`** is the tier-appropriate "the data is really there" object: nothing (T0), a
  host fence / `RunHandle` (T1), an exported timeline-semaphore value (T2). The render graph turns
  an acquired token into either a host wait before submit (T1) or a queue wait (T2) — in both cases
  *inferred inside the graph* from the acquire, never hand-placed by a pass. T2 has an exact in-tree
  precedent already: the atmosphere nest runs outside the render graph on its own timeline semaphore
  and the frame's submissions wait on it (`vulkan_scene_view.cpp:829-850`) — `Handoff` generalizes
  that hand-wired pattern into the registry so the *next* external producer (the runtime) doesn't
  hand-wire a second one.

### 6.2 What `Handoff` deliberately is not

Not a scheduler, not a copy engine, not a snapshot system. Whether the sim double-buffers a column
so the render can overlap with tick N+1 is the sim domain's choice, expressed as publishing a
different interval per epoch; `Handoff` records and fences, it never allocates or copies. (When
BB-7's pipelined steps land and a sim thread overlaps the render thread, the publishing pattern is
ring-buffered intervals + T2 tokens — the contract above already expresses it with no new surface.)

### 6.3 Tiers — how each platform realizes the same contract

| Tier | Where | Mechanism | Cost of the contract |
|---|---|---|---|
| **T0 — in-place, sequential** | today's desktop loop (sim and render on one timeline) | shared-USM column read directly at extract; token is empty | zero — this is the current behaviour, formalized |
| **T1 — coarse fence** | desktop, first zero-copy consumers (BB-1a only) | renderer-exported buffer imported by runtime (`import_buffer`); token = run completion; host wait before render submit | deletes the copy and staging upload; keeps one host sync |
| **T2 — semaphore overlap** | desktop, steady state (BB-1b + BB-7) | token = timeline-semaphore value signalled by the run's sentinel; render queue waits on it | zero host syncs; sim/render overlap becomes real |
| **T3 — UMA, no SushiRuntime** | Android/iOS/console with the native `Execution` backend | **the allocator hook**: `Execution` `MemoryVisibility::DeviceShared` satisfied by an RHI-supplied allocator (`HOST_VISIBLE \| DEVICE_LOCAL`), so columns *are* graphics-visible; token = the native graph's completion | zero-copy recovered exactly as cross-platform plan §3/§4.3 requires; without this hook every tick becomes a hidden staging upload |
| **T4 — fallback** | any platform/device where export/import is unavailable | staged copy at publish; token = transfer completion | correctness floor; `create_interop_buffer` already documents this fallback ("no path depends on this") |

The **allocator hook** is thereby owned: it is the T3 realization of `Handoff` visibility, designed
here once, consumed by RUNTIME-PORT1 (the native allocator accepts an injected external allocator)
and by the RHI's buffer creation (Wall 2) — the two-wall agreement point §3 of the cross-platform
plan insists on. Direction per tier is asymmetric and deliberate: on desktop the **renderer exports
and the runtime imports** (G3 — `render/interop.hpp`'s recorded doctrine); on T3 the **RHI supplies
memory to `Execution`** at allocation time. Both keep graphics ownership of graphics memory, which
is the invariant that survives every backend.

### 6.4 Cross-device scoping

`Handoff` guarantees its contract only when both sides share a physical device (`DeviceInfo::uuid`
match — the key `render/interop.hpp` already selects by). Cross-*device* zero-copy (sim on a compute
dGPU, render on another) is explicitly **out of scope**: T4 is the answer there, and the primary
GPU-compute consumer that is genuinely device-remote (the SushiLoop server) is headless — its
`Handoff` has no render domain at all and degenerates to a no-op registry. This adopts §3.1's own
skepticism as a decision instead of a worry.

---

## 7. Determinism rules (mechanical, assertable)

1. Sim-domain nodes default `Bitwise`; render-domain nodes default `Cosmetic`; either may opt to
   `Tolerant` explicitly.
2. A `Bitwise` node's access set may not include a resource whose last publisher is `Cosmetic`
   (debug assertion at graph build; the taint is per-`Handoff`-entry, so the check is O(accesses)).
3. `Bitwise` output must be replay-stable under worker-count variation `{1, 2, max}` with the
   rebalancer off — the engine's §15.5 requirement, discharged by SushiRuntime's BB-3 conformance
   suite on the RuntimeBackend and by an equivalent engine-side suite on the NativeBackend.
   Cross-backend, the contract is `Tolerant` (bit-determinism *within* a backend, tolerance *across*
   — the cross-platform plan §4.3's contract, unchanged).
4. Rollback correctness is a publication rule, not a tracker rule (G4): `publish` fires once per
   presented state. A backend that cannot express "run K ticks, signal once" composes it from K runs
   and one publish — the contract does not care.
5. The render domain never re-records in response to rollback — guaranteed structurally, because
   nothing on the render side can observe an unpublished epoch.

---

## 8. Backend mappings

| UHM element | RuntimeBackend (SYCL, desktop) | NativeBackend (all platforms) | Vulkan RHI (render domain) | Metal/console (future, per plan §5) |
|---|---|---|---|---|
| `BufferInterval` | `Core::ResourceRegion` (member-wise) | shared hazard core key | `BufferHandle` + offset/size | same as Vulkan path |
| `TextureInterval` | — (rejected at boundary) | — | subresource range | `MTLTexture` slice/level range |
| `AccessIntent` | R/W projection into tracker | R/W projection | `resource_state` table (relocated per §5.3) | intent → `MTLFence`/scope table |
| edges/barriers | tracker-inferred node ordering | tracker-inferred + atomics/futexes | inferred `VkImageMemoryBarrier2`/`VkBufferMemoryBarrier2` | inferred fences/encoder splits |
| `CompletionToken` | `RunHandle` (T1) / exported timeline semaphore (T2, BB-1b) | graph completion latch | `VkSemaphore`/fence wait folded into submit | `MTLSharedEvent` |
| `DeterminismClass` | `Bitwise` honoured per BB-3 contract | `Bitwise` via single-threaded or fixed-reduce mode | `Cosmetic` default | `Cosmetic` default |

Nothing in the right two columns requires anything of SushiRuntime, and nothing in the left column
requires anything of Vulkan — the columns only meet in the vocabulary row-headers. That is the
non-forced, organic relationship stated as a table.

---

## 9. What this changes in the standing plans

**The §9 gating decision (cross-platform plan): resolved — proceed now, no gate needed.** The
risk was vocabulary divergence *because* the shared design didn't exist; it now does, so:

- **RUNTIME-PORT0** mints its `Execution` vocabulary as §4 above (`AccessIntent`, `BufferInterval`,
  `NodeDescriptor`, `DeterminismClass`) instead of a private
  `Execution::ResourceRegion`+bare-pointer-lists shape. Cost delta versus the plan's original
  sketch: near zero — the adapter to SushiRuntime stays member-wise trivial by construction (§4.2),
  and kernel bodies are untouched.
- **RHI0** was never vocabulary-bound; unchanged.
- **RHI1** re-keys `resource_state.cpp` on `AccessIntent` + `StageMask` when it collapses
  `TextureState`/`BufferState` — work it was already doing; the only delta is *which* enum it
  lands on.
- **RHI2** migrates the render barrier planner onto the shared hazard core (§5), under the RHI0
  harness.
- **Requests to SushiRuntime** (via `physics_system.md` §18, per the two-repo convention): none new.
  UHM consumes BB-1a/1b and BB-7 exactly as specified in `ENGINE_BACKBONE_REFACTOR.md`; the only
  soft ask is that `import_buffer`'s returned `Buffer<T>` participates in interval-exact tracking
  (BB-1a already commits to this) and that BB-1b lands the semaphore-export design (i) variant,
  which keeps SYCL types out of the engine.
- **The runtime moved under the engine's feet, favourably — adopt it (BB-0 engine half).**
  Verified 2026-08-01: `feature/physics-substrate-seams` is *merged* (`f801cc5`) and a second
  wave landed on top (`cb894ff`: BB-3 determinism flags, BB-2 co-tenancy, BB-4 packaging).
  Two immediate engine-side consequences: (1) R1–R7 adoption (delete the hand-built two-node
  reduction, `sized_from_device` for the broadphase→solver chain, region-per-island) is
  unblocked *now*, and UHM0's vocabulary should land before or with it so adoption emits
  through `Execution` rather than adding fresh direct call sites; (2) **a known breakage**:
  `RuntimeConfig`'s rebalancer default flipped to off, and the engine's
  `test_runtime_graph_builder.cpp:266-277` asserts the old default — that test inverts as part
  of adoption, exactly as the BB plan predicted.
- **The runtime's WP-7 native-CPU path and the engine's NativeBackend do not merge.** They look
  similar and serve different masters: WP-7 keeps a *SushiRuntime binary* alive on a
  zero-SYCL-device desktop; the NativeBackend keeps the *engine* alive on platforms where
  SushiRuntime cannot exist at all. One is a runtime resilience feature behind `try_create`
  (BB-5b); the other is a portability backend behind a compile-time seam. Sharing code between
  them would re-couple the repos exactly where the seam exists to decouple them.

### UHM milestones

| Code | Delivers | Slots |
|---|---|---|
| **UHM0** | **LANDED 2026-08-01 (partial — see below).** `execution/access.hpp` + `execution/interval.hpp` + `NodeDescriptor`, consumed by RUNTIME-PORT0's seam extraction. Exit: PORT0's own oracle (byte-identical logs, `compile_count==1`) — **met**, `sandbox` reports `compile_count=1 mismatches=0`, suite 1108/1108 — plus the §5 conformance test's RuntimeBackend half, which is deferred to UHM1. | with RUNTIME-PORT0 |
| **UHM1** | shared hazard core (header template) + NativeBackend adoption. Exit: conformance test green on both implementations; §7.3 worker-count suite on the native side. | with RUNTIME-PORT1 |
| **UHM2** | render vocabulary alignment (`AccessIntent` re-key of `resource_state`, `StageMask`). Exit: trace byte-identical under RHI0's `TraceCommandList`. | with RHI1 |
| **UHM3** | `Handoff` T0/T1: registry + epochs + fence tokens; first zero-copy consumer (animation palettes — the BB-1a proof consumer) routed through it. Exit: capture shows no staging upload for the routed buffer; palette readback loop deleted. | after BB-1a lands runtime-side |
| **UHM4** | `Handoff` T2 (semaphore tokens) + publish-side ring pattern for the sim/render overlap. Exit: zero host syncs on the routed path; frame captures show queue-level wait. | after BB-1b/BB-7 |
| **UHM5** | T3 allocator hook realized against the native backend + RHI buffer creation. Exit: UMA test (desktop-simulated first: `HOST_VISIBLE\|DEVICE_LOCAL` heap) shows columns render-visible with no copy. | with RHI5/RUNTIME-PORT4 era |

#### UHM0 — what landed, and what did not (2026-08-01)

Landed, in `include/SushiEngine/execution/`: `access.hpp` (`AccessIntent` with its three
projections, `DeterminismClass`), `interval.hpp` (`ResourceId`, `BufferInterval` with the
`WHOLE` sentinel, `TextureInterval`, the `ResourceInterval` sum type), `node_descriptor.hpp`
(`ResourceAccess`, `NodeKind`, the non-allocating `Provider` and `NodeDescriptor`),
`hazard.hpp` (the normative semantic, `accesses_conflict`, `classify_hazard`,
`nodes_conflict`, `determinism_permits`), `memory.hpp`, `run_report.hpp`, `context.hpp` (the
compile-time backend selection), and `backend/runtime_backend.hpp` (the member-wise
SushiRuntime adapter). `ecs/{chunk,archetype,world,schedule}.hpp` name only these types;
`loop/app.hpp` owns the context. `tests/functional/unit/test_hazard_semantics.cpp` pins the
semantic. The naming follows `CONTRIBUTING.md` §4 rather than this document's original
sketch — `Execution`/`NodeDescriptor`, not `Exec`/`NodeDesc`, since neither abbreviation is
an acronym; the name is adjacent to but never ambiguous with `SushiRuntime::Execution`,
because no translation unit in the engine carries a `using namespace SushiRuntime`.

Two deviations from §4 worth recording. `ResourceId` is `void*`, not `const void*`, so the
lowering to `Core::ResourceRegion` stays a member-wise copy with no cast in the adapter.
`NodeDescriptor` carries a pointer-and-count access view rather than `std::span`, because the
codebase targets C++17.

Not yet landed, and deliberately not claimed: the conformance suite's **RuntimeBackend half**
(the recorded-order probe graph of §5) — the suite drives the vocabulary only. Its value is
real only once a second backend exists to compare against, so it is deferred to UHM1 rather
than written blind. The `Math::{sqrt,fmod,floor}` shim, held back here as speculative, landed
later the same day with the standalone-solver retarget, which gave it three real consumers.

#### The physics retarget (§4.5's stress test), 2026-08-01

`physics/solver/runtime_graph_builder.hpp` — the deepest coupling site in the engine, and the
one §4.5 was written against — **names no runtime type at all** now. Each construct landed as
that table assigns it, with two corrections worth recording:

| §4.5's construct | What it became |
|---|---|
| `Buffer::region(ElementRange)` | `Execution::Buffer<T>::interval(ElementRange)`, with the element-to-byte conversion inside the buffer rather than at the call site. |
| `API::sized(...)` / `API::when(...)` | `NodeDescriptor::count` / `::enabled`, passed to one `emit_node` helper that also carries the node's name — which the runtime form had no place for. |
| `add_reduce` (R2, fixed order) | **A graph verb, not a node kind.** A fold takes two buffers and a combiner, not an access list and a per-element kernel; expressing it through `NodeDescriptor` would have meant fields every other node ignores. `NodeKind::{Reduce, SegmentedReduce}` are therefore *removed* from the vocabulary rather than left unlowered. |
| `Residency` / `DeviceIndex` | `MemoryVisibility` plus an `Execution::DeviceIndex` argument, with `BackendCapabilities::device_count` as the honest bound — the seam does not pretend a host-only backend has devices to choose between. |
| `sized_from_device` / `based_at_device` | **Not needed**: this file uses neither, so no device-slot provider was built. It stays §4.5's answer for when one appears. |

One construct §4.5 did not anticipate: the builder disables SushiRuntime's thermal rebalancer,
because migrating work mid-run costs the jitter a fixed-rate tick cannot absorb. That became
`Context::set_work_migration(bool)` — a throughput/reproducibility trade any backend with a work
pool eventually has to expose, and one a backend that never migrates satisfies by doing nothing.

`sim/physics_simulation.hpp` and `create_physics_simulation` moved with it.

#### The rest of PORT0's surface, same day

The standalone solvers and the batch animation evaluator followed, which is what actually
collapses the blast radius: `physics/solver/pgs_solver.hpp`, `physics/solver/xpbd_solver.hpp`,
`physics/scene/physics_world.hpp`, and `animation/device_batch_evaluator.hpp`. These used a
*different* runtime shape from the graph builder — the capture-list `add(Extent, In(buffer),
InOut(buffer), …)` overload with `sycl::id<1>` kernel signatures — so each node became an
explicit access declaration over raw pointers, the same form the ECS and the graph builder use.
Four `sycl::id<1>` signatures became `std::size_t`, exactly as cross-platform plan §4.4
predicted.

That removed the last `sycl::` call in a kernel body outside the audio accelerator, so the
`Math::{sqrt, fmod, floor}` shim §4.4 asks for landed in `core/types.hpp` with the three real
consumers it was waiting for (`pgs_solver`'s distance projection, the evaluator's clip-time
wrap and frame interpolation) rather than as speculative machinery.

**Blast radius, measured 2026-08-01:** files naming SushiRuntime in code outside tests and
examples went from 38 under `include/` to **three**, all structural and none a porting task:
`execution/backend/runtime_backend.hpp` (the adapter, by definition), `loop/app.hpp` (the
composition root, which constructs the runtime and hands out the context), and
`audio/accelerator_sycl.hpp` (already correctly isolated behind `IDspAccelerator`, and
deliberately backend-specific — it is the *optional* GPU DSP path, not a portable one). Plus
`sim/runtime_simulation.cpp`, which owns the runtime for the same reason `loop/app.hpp` does.
That discharges RUNTIME-PORT0's stated exit, and with it the standing project guidance that
SushiRuntime call sites be kept thin and isolated.

A late-bound provider's storage model changed as part of this, and the reason generalizes: the
physics builder passes provider lambdas as *temporaries*, one per (kind, colour, substep), so a
provider that held a pointer to the callable would have dangled at exactly the call sites that
emit nodes in a loop. `Provider` therefore copies the callable into 32 bytes of inline storage,
requiring it to be trivially copyable — the same requirement a device backend imposes anyway.

---

## 10. The performance contract (what "SOTA" is allowed to mean)

Claims are stated as *measurable gates*, not adjectives; each is the proof obligation of the
milestone that makes it true.

1. **Nothing regresses at adoption.** UHM0/1/2 are provable no-ops (logs, traces, goldens).
2. **The copy dies.** UHM3 deletes the solver→`read_range`→mirror→extract→upload round-trip for each
   routed consumer (four are recorded waiting: palettes, soft-body vertices, VFX sim, render
   extract). Gate: RenderDoc/PIX capture contains no staging transfer for routed buffers.
3. **The sync thins.** UHM4's gate: zero host-side waits on the routed path in a steady-state
   frame; the only cross-engine ordering is one queue wait per publish batch.
4. **Steady-state ticks allocate nothing.** Composes with BB-2's `run(RunReport&)`;
   `Handoff::publish/acquire` are allocation-free by construction. Gate: counting-allocator
   assertion over N ticks.
5. **Replay holds under load.** §7.3's worker-count-varied byte-equality, both backends.
6. **Tracking overhead stays sub-measurable.** The tracker semantic is O(accesses) with
   registration-order tie-breaks; the render planner migration (UHM2/RHI2) must show frame CPU
   time no worse than the hand-rolled planner it replaces — same bar the binding-model split
   already sets (§5.4: "likely a net CPU win", now with a harness to prove it).

For calibration, not marketing: shipping engines at this scope (UE5's TaskGraph+RDG, Frostbite's
FrameGraph) run CPU-job and render-hazard tracking as two systems joined by hand-placed sync points;
render-graph automation at per-frame scope is industry standard, cross-domain *inferred* sync at an
epoch boundary with a determinism class in the vocabulary is not. That — plus the zero-copy tiers —
is the defensible differentiation. Everything else UHM does is disciplined normality, and should be,
per the engine's own non-goals.

---

## 11. Rejected alternatives (recorded so they are not re-proposed)

- **One collapsed graph object across domains.** Cadence (compile-once replay vs per-frame
  rebuild), rollback invisibility, and determinism classes all fight it; §3.1 suspected it and
  the ground truth confirms it. Two domains, one waist.
- **SushiRuntime as the unified tracker's owner.** Inverts the dependency arrow, couples the
  render graph to an API the engine's own memory records as unstable, and contradicts BB P4
  (sim domain, not render resources). The runtime remains the best implementation of the
  semantic in its domain — nothing more is asked of it.
- **Byte-interval tracking inside the render graph.** Opaque, possibly-tiled images have no
  meaningful byte intervals; subresource+intent is the correct granularity and already exists.
  Unifying granularities would be a rename pretending to be a design.
- **Image traffic across the boundary.** BB-1's non-goal, atmosphere's recorded lesson (U3).
- **A runtime-polymorphic `Execution` backend.** Already rejected by the cross-platform plan (§4.1,
  device-copyability); UHM adds no reason to revisit it.
- **Retyping kernel/pass hot paths onto a generic resource handle.** U2; the sum type is cold
  boundary metadata, not a hot-path currency.

---

## 12. Risks & open questions

| Risk | Mitigation |
|---|---|
| BB-1b's semaphore-export lands as design (ii) (raw event bridging) instead of (i) | T2 tokens are defined behind `CompletionToken`; (ii) would confine SYCL-event handling to the RuntimeBackend adapter — uglier, not architecture-breaking. UHM4 waits for the decision rather than pre-building both. |
| Interval-exact tracking of imported buffers (BB-1a) subtly differs from column regions | The §5 conformance test gains an imported-buffer case in UHM3; divergence is a test failure, not a discovery in production. |
| `Handoff` epoch pinning vs frames-in-flight (render still reading epoch N while N+1 publishes in T2) | Publish-side ring intervals (§6.2) are the designed answer; the UHM4 exit criterion includes a 3-frames-in-flight capture test. |
| Render planner migration (UHM2/RHI2) destabilizes shipping barriers | Hard-gated on RHI0's harness; migration is per-pass-diffable via `TraceCommandList`, same discipline as the rest of Wall 2. |
| The shared core template grows domain warts | Its trait surface is `{Key, Interval, Intent}` and the two instantiations land in the same milestone window (UHM1/UHM2) — divergence pressure is visible immediately, in-tree. |
| SushiRuntime's WP-2 changes `Core::ResourceId` from `void*` to an opaque registered id | `Execution::ResourceId` is a typedef from day one (§4.2), mirroring the runtime's own alias; the RuntimeBackend adapter is the only translation site either way. |
| A tighter engine tracker orders fewer pairs than the runtime's conservative scan, changing schedule *shape* across backends | Permitted by design (safety floor §5); the cross-backend contract is already `Tolerant`, and the conformance test pins conflicting-pair ordering — the only thing correctness needs. |
