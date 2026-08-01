# Cross-Platform Engineering Plan — Windows / Linux / Android / macOS / iOS / PS5 / PS4

Platform priority: **Windows + Linux first**, then **Android**, then **macOS + iOS**, then **PS5/PS4 last** — see §0.0.

Status: research complete, no implementation started. Ground-truth findings verified against the source tree on 2026-07-25; **re-verified 2026-08-01** — the counts and characterizations corrected below are marked "(re-verified 2026-08-01)". The §3.1 shared-vocabulary design pass has since landed: see `unified_hazard_model.md` (UHM). PS3 is explicitly out of scope (EOL hardware, no active toolchain, no serious studio still targets it). PS5/PS4 sections are written entirely from public knowledge — this project has no PlayStation Partners registration or devkit access as of this writing, so nothing NDA'd is claimed or guessed at anywhere in this document.

This document is the multi-platform analogue of `render_pipeline_refactor.md`: a living, phase-tracked plan. Update phase status in place as work lands, the same way that document does.

### 0.0 Platform priority (supersedes any conflicting sequencing implied elsewhere in this document)

**Windows and Linux are the primary focus and ship first.** After that, the priority order is: **Android next, then macOS and iOS, then PS5/PS4 last.** Any wording elsewhere in this document that could be read as implying a different order (e.g. macOS ahead of Android) is superseded by this statement — the milestone tables and sequencing in §8 already happen to route Android's Wall-2 proving ground (`RHI6`) ahead of Metal (`RHI7`/`RHI8`), which is consistent with this priority and should stay that way; where any future edit to this document would reorder that, check against this section first.

This priority is deliberately expressed as a **sequencing decision, not a structural one**: no wall, interface, or milestone in §4–§6 is platform-ordered in its design — `Execution`, the RHI, and `IPlatformHost`/`IDisplaySurface` are all written so a platform can be slotted in through an existing seam (compile-time backend policy, backend registry, capability-flag query) whenever its business/technical prerequisites clear, independent of where it sits in this priority list. That is intentional: the priority order can change again without requiring a re-architecture, which is the whole point of building the seams this way rather than hardcoding a platform list into any interface.

---

## 0. How to read this

The engine is **not** starting from zero on any axis. The finding that should reframe every subsequent decision: this is a mature, disciplined, AAA-scope Vulkan desktop renderer and a mature ECS/physics/audio/animation/VFX stack, built entirely on Windows. The gap to "AAA cross-platform" is not "rebuild the engine" — it is **three specific, separable structural walls**, each with a concrete engineering answer, plus a long tail of per-platform packaging/certification work that is well-understood industry practice, not engine-specific risk.

Read in this order:
1. §1 Executive summary — the three walls and the one-sentence fix for each.
2. §2 Current state — what exists today, verified.
3. §3–6 — the three walls, each with a recommended architecture and a phase-lettered milestone list (`RUNTIME-PORT*`, `RHI*`, `PLATFORM*`).
4. §7 — per-platform requirements (toolchains, packaging, certification).
5. §8 — the unified sequencing across all three walls and seven platforms.
6. §9 risk register, §10 immediate next actions, §11 explicit non-goals.

---

## 1. Executive summary

### 1.1 The three structural walls

| # | Wall | The problem | The fix | Owning section |
|---|---|---|---|---|
| 1 | **SushiRuntime / SYCL execution wall** | The ECS's `World`/`Schedule`/`Chunk` types are typed directly against `SushiRuntime::API::{Runtime,Graph,Buffer}`. SushiRuntime's SYCL toolchains (intel-llvm, AdaptiveCpp, oneAPI DPC++) have **zero path** to macOS, iOS, Android, or PlayStation — no code, no docs, no CI. (Re-verified 2026-08-01: the runtime's `ENGINE_BACKBONE_REFACTOR.md` §12/BB-8 now *states* the gap and records "no Metal/Vulkan-compute backend" as a non-goal — a plan, not yet a published support matrix.) Any TU that includes `World`/`Schedule` cannot currently be compiled for 5 of 7 target platforms. | Introduce a **compile-time execution-backend seam** (`SushiEngine::Execution`) inside SushiEngine, with the existing SushiRuntime path as one implementation and a new native thread-pool/DAG implementation as the other. This is smaller than it sounds: the actual SYCL surface in `include/SushiEngine/` is **11 symbol usages across 4 files** (re-verified 2026-08-01; the previously counted 12th/5th was an example TU) — the system bodies themselves are already plain C++. One new deep coupling site has appeared since the original survey: `physics/solver/runtime_graph_builder.hpp` (42 `SushiRuntime::` usages) — see §4.4. | §4 |
| 2 | **Vulkan-hardcoded RHI wall** | The renderer's outer seam (`IRenderDevice`, `IWindowRenderer`, `ISceneView`) is genuinely backend-neutral, but everything beneath it — the render graph, all 44 passes (re-verified 2026-08-01; was 39), the lighting/scene/resource systems — is typed directly against `Vk*`/`VmaAllocation`. 81 files include `<vulkan/vulkan.h>`; 34 include `<vk_mem_alloc.h>` (re-verified 2026-08-01; was 69/26). No second backend exists anywhere. | A layered `Rhi::{types, handles, ICommandList, IDevice}` abstraction, introduced behind a golden-image + command-trace regression harness (there are currently **zero render tests** — a two-pixel probe is the entire safety net for a ~48,700-line C++ refactor, 64,061 LOC with shaders; re-verified 2026-08-01). Metal is the real second backend; MoltenVK is a bring-up crutch and permanent differential oracle, not the shipping answer. | §5 |
| 3 | **No shippable non-editor target** | The only executable that draws a full scene to a screen is `se_editor` (SDL2 + ImGui + Vulkan desktop app). There is no player/runtime target, no scene→swapchain present path outside ImGui, no asset-mounting abstraction, and CMake has essentially zero platform-conditional logic beyond Windows-vs-POSIX. | Extract `sushi_platform` and `sushi_scene` libraries out of `editor/`, build a lean `se_player` executable driven by a `PlayerApp::frame()` callback (not a `while` loop — mandatory for iOS), and grow the build system (`cmake/Platform.cmake`, toolchain files, CI matrix) to support cross-compilation. | §6 |

### 1.2 What's better than expected

Every research agent independently found the codebase in noticeably better shape for portability than the brief assumed:

- SushiEngine's ECS **system bodies contain no SYCL at all** — `sandbox/main.cpp` (the canonical reference) has zero `sycl::` usages. The coupling is at the type level (three API types), not the semantic level.
- **Touch input is already fully implemented** (`SdlInputTranslator`, `VirtualControlSource`, `GestureRecognizer`, unit-tested) — it is simply never wired to any host. This is a wiring task, not a build task.
- **SDL2 has real, official, production-grade Android and iOS backends** already in wide industry use — not something to build from scratch.
- **miniaudio (already vendored) covers Windows, macOS, iOS, Android, and Linux natively**, with zero new third-party dependency — only build-flag/linking fixes are needed per platform.
- The renderer's use of **Vulkan 1.3 dynamic rendering** (`RenderingInfo`, no `VkRenderPass`/`VkFramebuffer` objects) maps almost 1:1 onto Metal's `MTLRenderPassDescriptor` and a console's explicit render-target binding. Had the engine used the older render-pass/subpass model, the Metal port would be substantially harder.
- The renderer's shaders use **zero subgroup/wave intrinsics** anywhere — the single hardest thing to cross-compile to MSL is simply absent from this codebase.
- Every hand-barriered compute pass (19 of them as of 2026-08-01 — grown from 13 with the atmosphere work, **and the zero-attachment invariant still holds on every one**) declares **zero attachments** — no pass mixes rendering and manual barriers in a way that would be illegal on Metal's encoder model.

### 1.3 What's worse than a casual read would suggest

- The renderer hard-requires **Vulkan 1.4** with `maintenance5`/`maintenance6`/`pushDescriptor` as *mandatory* device features (`vulkan_device.cpp`). No shipping Android device and no MoltenVK build offers Vulkan 1.4. This must be walked back to a 1.3 floor before Android or Apple-via-MoltenVK is even possible.
- There is **no scene→swapchain present path that isn't ImGui**. `IWindowRenderer::begin_frame()` returns a raw command buffer that today only the ImGui backend records into. A player that fills the screen with a rendered scene cannot be built without a new renderer entry point.
- Two absolute build-tree paths are **compiled into the shipped library** (`SUSHI_SHADER_SOURCE_DIR`, `SUSHI_PIPELINE_CACHE_DIR`) — a shipped player would try to read/write its pipeline cache into the developer's build directory.
- There are **no render tests**. The entire safety net for refactoring 39,894 lines of render code is an 82-line headless triangle that checks two pixels.

### 1.4 Scale, honestly

This is a **multi-year, multi-engineer programme**, not a quarter of work. The RHI track alone is estimated at 12–18 engineer-months to reach Metal parity, two years to also retire the shader toolchain onto Slang. Console work cannot even start until a PlayStation Partners agreement and devkit access exist — that is a business/legal prerequisite, not something derisked by engineering. Treat the milestone lists in §4–6 as the actual project plan, sized per milestone, not as a wish list.

---

## 2. Current state (verified ground truth)

### 2.1 Engine/build layer

- Repo layout: header-only ECS core in `include/SushiEngine/`; compiled libraries `sushi_render`, `sushi_sim` (the one SYCL translation unit, via `ISimulation`), `sushi_input`, `sushi_audio`; `editor/` (`se_editor`, SDL2+ImGui+Vulkan, gated behind `SE_BUILD_EDITOR`); `sandbox/` and `examples/` (~35 standalone SYCL-TU demos); `cli/` (the `se`/`ss` Python developer CLI).
- **Zero `if(APPLE)`, `if(ANDROID)`, `if(IOS)`, or `CMAKE_SYSTEM_NAME` checks exist anywhere in the CMake tree.** The only non-Windows CMake branches are two `if(NOT WIN32)` blocks (`cmake/Vcpkg.cmake`, `audio/CMakeLists.txt`) written as "else of Windows," not per-OS handling.
- `cmake/Vcpkg.cmake` is a no-op off Windows entirely (`if(NOT WIN32) return()`).
- No `vcpkg.json` manifest exists in either repo. Dependency provisioning is a custom `ss` CLI reading `cli/sushistack.deps.toml` fragments with only `linux_apt`/`windows_vcpkg` fields — **no macOS/Android/iOS field exists in the schema at all.**
- Exactly two working POSIX code paths exist **inside `editor/`** (`editor/core/preferences.cpp:53`, `editor/main.cpp:100`); two functions are explicitly Windows-only with a no-op stub and an in-code comment: *"Windows-only for now; the project targets Windows first."* (Re-verified 2026-08-01: tree-wide, three more real POSIX branches exist outside the editor — `render/material/font_atlas.cpp:51` font paths, `render/rhi/vulkan/vulkan_device.cpp:310` `VK_KHR_external_memory_fd`, and the Win32-handle-vs-fd split throughout `render/interop/vulkan_interop_buffer.cpp` — so the "Windows-only codebase" framing is editor-scoped, not global.)
- No `__APPLE__`, `__ANDROID__`, `TARGET_OS_IOS`, or any engine-defined `SE_PLATFORM_*` macro exists anywhere.
- CI (`.github/workflows/ci.yml`) has exactly 3 jobs (`functional`, `editor`, `docs`), **all `runs-on: ubuntu-latest`.** No Windows or macOS runner exists today despite Windows being the primary dev platform. No packaging/installer infrastructure exists anywhere (no CPack, NSIS, app-bundle/codesign, `.apk`/`.ipa`).
- The project has **never cut a tagged release** — the entire `CHANGELOG.md` sits under `## [Unreleased]`.

### 2.2 Renderer / RHI

- `render/` is 64,061 LOC total / 48,667 C++ (re-verified 2026-08-01; was 39,894) across `graph/` (2,383), `passes/` (19,529, 44 passes deriving `IRenderPass` + a synthetic `"present"` pass), `scene/` (4,519), `rhi/vulkan/` (4,356, the only backend), `resources/` (3,442), `shaders/` (14,915, 101 files).
- Feature set is genuinely AAA-scope: clustered Forward+ lighting (froxel grid, stochastic/MegaLights-style visibility for over-budget lights), cascaded + ray-traced sun shadows, SDF/probe-volume GI, Hillaire-2020 atmosphere, volumetric fog/clouds, TAA, GPU-driven culling with Hi-Z occlusion, mesh shaders/meshlets, bindless textures, variable rate shading, a full post stack, and a raytracing BLAS/TLAS accelerator.
- Two abstraction layers, and the abstraction stops at the outer one: `IRenderDevice`/`IWindowRenderer`/`ISceneView` carry no Vulkan types in their public signatures (by design, documented as a dependency-inversion boundary). Everything beneath — `render/graph`, all 39 `render/passes`, `render/lighting`, `render/scene`, `render/resources` — is hardcoded to `Vk*` types. **Only one RHI backend exists anywhere in the tree**, and `render/CMakeLists.txt` hard-requires Vulkan/VMA/vk-bootstrap/glslang at configure time.
- Shader pipeline is GLSL-only, compiled offline to SPIR-V via a glslang host tool that emits C++ headers with embedded SPIR-V words. Zero cross-compilation exists. This was an **explicit, deliberate, previously-deferred decision** (`docs/slop/render_pipeline_refactor.md` Phase 3.6), with a stated revisit trigger: *"when a non-SPIR-V backend is required."* That trigger has now fired.
- GPU data binding uses four simultaneous strategies (re-verified 2026-08-01): a 32-binding push-descriptor set 0 (**already documented as full** — `scene_layout.cpp:199` static-asserts the 32-entry floor; bindings 0 and 7–31 are all named frame-globals, 1–6 are the only pass-local slots), a bindless `update-after-bind` heap (set 1), a plain per-frame instance set (set 2, `INSTANCE_SET` — GPU-driven instance/compacted buffers, not in the original survey), and 128-byte push constants. All Vulkan-typed, no abstraction.
- `vulkan_device.cpp` mandates Vulkan **1.4** plus `maintenance5`/`maintenance6`/`pushDescriptor` as required (not optional) features.
- No render tests exist anywhere in `tests/`. The only harness is `render/probe/main.cpp` (82 lines, checks two pixels).

### 2.3 SushiRuntime (the sibling "battery" repo)

- Confirmed real, implemented subsystems: a SYCL task graph with a lock-free sharded dependency tracker (RAW/WAR/WAW hazard detection over byte-range `ResourceRegion`s — precisely: an append-only per-shard access history with a backward overlap scan, *not* a last-writer/reader-set map; see UHM §5 for why the distinction matters), NUMA-aware USM allocation (precision, 2026-08-01: hwloc backs *topology discovery* only; NUMA-awareness in allocation is per-node SYCL queue selection, and the engine's column path is raw `sycl::malloc_shared` under a process-global mutex — the runtime's own pool is bypassed), and hardware discovery.
- **(Added 2026-08-01)** The R1–R7 substrate seams (`add_untracked`, deterministic `add_reduce`/`add_segmented_reduce`, `sized_from_device`, interval-exact region boundaries, `run_async` + `based_at`, magazine teardown) are **merged to the runtime's `main`** (`f801cc5`), with a second wave on top (`cb894ff`: BB-3 determinism flags, BB-2 co-tenancy `RuntimeSettings`, BB-4 packaging). Two consequences for this plan: the engine-side adoption work (BB-0's engine half) is unblocked now, and the co-tenancy wave **flipped the rebalancer default to off** — the engine's `test_runtime_graph_builder.cpp:266-277`, which asserts the old default, will fail against runtime HEAD and must be inverted as part of adoption (the BB plan predicted exactly this).
- Three supported SYCL toolchains (intel-llvm, AdaptiveCpp, Intel oneAPI DPC++), all LLVM/Intel-lineage. GPU backend selection covers CUDA, ROCm, Intel Level Zero, and a CPU/OpenCL fallback — **no Metal, no mobile Vulkan-compute, no console backend exists, and none of the three toolchains has an Apple, Android, or PlayStation implementation at all.**
- **This gap was not documented anywhere** at the original survey; partially stale as of 2026-08-01 — the runtime's `ENGINE_BACKBONE_REFACTOR.md` §12 (BB-8) now states it, including "no Metal/Vulkan-compute backend" as an explicit non-goal, though the promised one-page support matrix in the runtime's public `docs/` is still unwritten. The platform branching in `include/`+`src/` is Windows-vs-POSIX only, but at **five** sites, not one (re-verified 2026-08-01): thread affinity (`topology/affinity.hpp`), dllexport (`core/export.hpp`), console handling (`core/logger.hpp`), MSVC intrinsics (`core/bit.hpp`), and Winsock-vs-BSD sockets (`distributed/tcp_transport.cpp`) — additional porting surface the "~3 files" framing elsewhere does not cover. Also (re-verified 2026-08-01): the phrase "JIT-first" appears nowhere in the runtime repo; the accurate statement is *JIT is the default on the CPU/spir64 lane, AOT exists per vendor (`spir64_gen`, `sm_*`, `gfx*`) but is opt-in and not wired for ship lanes* (that wiring is BB-5c). The load-bearing conclusion — consoles/iOS forbid runtime GPU codegen, so the shipping posture is structurally incompatible with them today — survives the correction.
- SushiRuntime's own CI runs exclusively on `ubuntu-latest`.
- **However**, the actual coupling from SushiEngine's ECS into this machinery is narrow: `Schedule::each`'s system kernels contain **no SYCL** — they are plain C++ over raw SoA column pointers. The only thing that makes a system "device code" is a wrapper inside SushiRuntime itself (`Graph::emit_dynamic_per_element`, reached transitively via `Graph::add` — the engine never names it). SushiEngine's own SYCL usage in `include/SushiEngine/` is 11 symbol usages across 4 files (re-verified 2026-08-01; the old 12/5 count included an example TU), one of which (`audio/accelerator_sycl.hpp`, 5 of the 11) is already correctly isolated behind an `IDspAccelerator` interface. **The file-count blast radius, by contrast, grew**: 66 files now mention SushiRuntime (was 52), and the physics work added `physics/solver/runtime_graph_builder.hpp` — self-described as "the one file in the physics layer that names SushiRuntime", 42 usages, consuming a far wider API surface (`ElementRange`, `when()` predicated nodes, `sized()`, `Residency`/`DeviceIndex`) than the ECS does. The `Execution` vocabulary must cover those constructs or physics cannot retarget — UHM §4.5 assigns each a home.

---

## 3. The three structural walls — why they're separable

These three walls can be worked **independently and mostly in parallel** by different engineers, with exactly two hard coupling points:

1. **The allocator/interop seam.** *(Characterization corrected 2026-08-01.)* `render/interop.hpp` is not a USM-column pass-through: it is the renderer's **export half** — the renderer allocates external-memory buffers (UUID-matched, Win32-handle/fd) for the *runtime* to import, and today it has **zero callers** because the runtime's import half (BB-1a) and completion export (BB-1b) are planned but unbuilt; the shipping sim→render path is still a host round-trip (value-snapshot `RenderScene` extract). The requirement the original wording gestured at is real and stands, tier-shaped: on desktop, zero-copy = renderer-exports/runtime-imports (BB-1a/1b); on UMA platforms with no SushiRuntime, the native `Execution` backend's allocator must accept an RHI-supplied allocator so columns *are* graphics-visible. Both tiers, the direction asymmetry, and the injectable-allocator hook are now designed once in **UHM §6** — do not re-derive them per wall.
2. **Android and iOS bring-up require all three walls to have landed their platform-specific milestone before a scene can render on-device.** Desktop (Windows/Linux) requires none of this — it works today. macOS needs Walls 1 and 2 (native Execution backend + Metal-or-MoltenVK) but not a new Wall-3 windowing backend beyond what SDL2 already provides for desktop Apple.

Everything else — the renderer's Vulkan-mobile/Metal work, the ECS's native execution backend, the player/runtime target split — can be designed, implemented, and tested against Windows/Linux long before any mobile or console hardware is involved.

### 3.1 A fourth coupling point — the shared sim+render hazard vocabulary (design deferred)

**Status: DESIGNED (2026-08-01) — see `unified_hazard_model.md` (UHM).** The dedicated design pass this subsection deferred has landed: UHM defines the vocabulary types (`AccessIntent`, `BufferInterval`/`TextureInterval`, `DeterminismClass`, `NodeDescriptor`), the tracker contract (safety floor + determinism floor + engine-core quality target), the ownership boundary (engine-owned `include/SushiEngine/execution/`; SushiRuntime remains a backend behind a member-wise adapter), and the domain-boundary interop contract (`Execution::Handoff`, epoch-published, tiered T0–T4). The paragraphs below are retained as the requirement's original statement; where they conflict with UHM, UHM governs.

Beyond the two coupling points above, there is a third, higher-leverage one: **Wall 1's execution-hazard model (`Execution::ResourceRegion`, byte-range tracking over raw memory, §4.3) and Wall 2's render-hazard model (`Graph::TextureAccess` access-intent barriers over opaque GPU subresources, §5.3) are currently designed as two independent systems.** The confirmed ambition for this engine is to unify them into a single hazard-tracked graph spanning CPU jobs, GPU compute, and GPU rendering — not just a shared allocator (§3 point 1), but a shared dependency/barrier model. Done well, this is the most architecturally differentiated part of the whole programme: most shipping engines keep the CPU job graph and the render graph as two systems synchronized by hand-placed fences, not one system with automatically inferred barriers across both domains.

Known open constraints for whoever designs this (surfaced in discussion, not yet resolved):

- **Vocabulary mismatch.** Byte-range regions on linear memory (Wall 1) vs. subresource + access-intent on opaque, possibly-tiled GPU images (Wall 2) are not the same shape of "resource." A unifying model needs a superset — e.g. a `ResourceHandle` sum type over buffer-region and image-subresource, plus a generalized `AccessIntent` covering both raw R/W and typed shader/attachment access. This is a real design task, not a rename.
- **Cadence mismatch.** `Execution::Graph` compiles once and replays once per simulation tick, including during SushiLoop's rollback re-simulation (full-snapshot restore + replay — the "delta rollback" phrasing used elsewhere is aspirational; `loop/rollback.hpp` captures full column copies and records per-write dirty tracking as a follow-on); a render graph runs once per presented frame, at a different, variable rate, and must never re-record during rollback replay. "Unified" should be read as *one shared hazard/resource model and interop contract at the domain boundary*, not one literal DAG object recompiled at a single rate — the sim graph and render graph most likely stay two schedule domains sharing one tracker/vocabulary, not one collapsed graph object.
- **Cross-device sync.** Where SushiRuntime GPU compute (SYCL/Level Zero/CUDA/ROCm) and RHI GPU rendering (Vulkan/Metal) are live in the same process on the same or different physical devices, the two APIs do not natively share a submission queue or fence. This needs explicit, narrow scoping — true zero-copy cross-API interop is driver/vendor-dependent and fragile, and may not be worth it given the primary GPU-compute consumer (the server, see below) is typically headless and non-rendering.
- **Determinism boundary.** Sim nodes need bit-determinism (rollback correctness); render nodes don't and shouldn't pay for it. The unified model needs a per-node determinism-class marker from day one, not bolted on later.

**Governance risk — RESOLVED 2026-08-01**: the shared design now exists (UHM), so the gate question dissolves: `RUNTIME-PORT0` proceeds immediately and mints its vocabulary as UHM §4 (UHM0); `RHI0` was never vocabulary-bound; `RHI1` re-keys `resource_state.cpp` on the shared `AccessIntent` (UHM2) as part of work it was already scoped to do. No dual migration remains possible because neither wall ships a private vocabulary first. See the §9 risk-register entry, now closed.

**`SR_DISTRIBUTED` / SushiAI.** SushiRuntime's cluster/distributed-offload capability (`SR_DISTRIBUTED`, `Runtime::cluster()`, `distributed/tcp_transport.cpp`) is confirmed active roadmap for SushiAI (`D:\Projects\sushiai`, sibling repo, MIT, "lightweight deep learning framework," WIP — tensor/autograd/NN/optim/loss), not the client ECS. This is distinct from `RUNTIME-PORT7b`'s mobile-viability audit item — it is a load-bearing, intentional capability to keep, not something to prune. It should not be exposed through the portable `Execution` seam (§4): `Execution`'s entire purpose is cross-platform portability, and distributed/cluster is explicitly a non-portable, Linux/server-only concern that SushiAI should consume directly from SushiRuntime's native API, outside the seam.

Ground truth as of 2026-08-01: SushiAI today is built entirely on raw CUDA + cuBLAS (`CMakeLists.txt`: `enable_language(CUDA)`, links `cudart`), with zero SushiRuntime/SYCL coupling in its current code. It is planned for a large refactor (confirmed by the user, 2026-08-01): SushiAI's tensor operations will be rebound onto **SushiBLAS** (`D:\Projects\sushiblas`, sibling repo, "mathematics layer for SushiStack"), with training run through SushiBLAS, and SushiAI formally onboarded as a SushiStack module (`ss link` + its own `sushistack.deps.toml` — neither exists yet; SushiBLAS isn't onboarded either). SushiBLAS is **already** deeply, source-level coupled to SushiRuntime — every BLAS kernel (`src/engine/blas/level1/*.cpp`) includes `<SushiRuntime/graph/task_types.hpp>` and submits via `SushiRuntime::Graph::TaskMetadata`/`TaskType::MATH_OP`, and `src/CMakeLists.txt` links `sushiruntime` directly. This resolves the integration-shape question cleanly: the stack becomes **SushiAI → SushiBLAS → SushiRuntime**, three layers sharing one portability ceiling (Windows/Linux, GPU-compute-capable) — SushiAI never touches SYCL/CUDA directly once refactored, only SushiBLAS's tensor API. SushiRuntime's `Graph::add(OpID, ...)` distributed-offload-with-local-fallback overload (§2/§4, only meaningful under `SR_DISTRIBUTED`) may already be reachable from SushiBLAS's existing `TaskMetadata`/`OpID`-tagged kernel submissions — worth confirming which overload is actually used before assuming `v0.5.x`'s distributed infrastructure needs building from scratch.

**Resolved 2026-08-01**: `v0.9.x`'s original C#/Unity3D scope is retired — the user confirms this work now happens inside SushiEngine directly instead. Concretely: a dedicated **forward-only inference evaluator**, used for both **PINN (physics-informed neural network) physics** and **RL-agent decision-making**, will be built into SushiEngine, deliberately depending on the full sushi stack (SushiEngine → SushiBLAS → SushiRuntime) rather than staying portable. This is a fourth GPU-compute-needing consumer alongside the two solvers (PGS/XPBD) and the batch animation evaluator already named in §4.2 Service B — see that table's update below. Unlike batch animation (which already has a CPU-native twin), **no portable fallback is planned for PINN-physics or RL-agent inference** — this is an explicit, accepted scope decision (Windows/Linux-only for as long as it depends on SushiBLAS/SushiRuntime), not a gap to silently discover later. If PINN physics or RL-driven NPCs are ever required on Android/iOS/consoles, that needs its own explicit re-scoping decision (a second, portable forward-evaluator path independent of SushiBLAS) — not assumed by anything in this document.

---

## 4. Wall 1 — SushiRuntime execution portability (`SushiEngine::Execution`)

### 4.1 Recommendation

Reject "grow SushiRuntime a Metal/Vulkan-compute backend" as the near-term answer — SYCL-on-Metal is dead-end territory (no production implementation exists; the only community projects are chains of incomplete translation layers), and a SushiRuntime Vulkan-compute backend means building a second, unbounded SYCL implementation (single-source C++→SPIR-V lowering) that still wouldn't solve iOS (no native Vulkan) or consoles (proprietary APIs, and both platforms generally prohibit runtime JIT compilation of GPU code — SushiRuntime is documented as "JIT-first," which is structurally incompatible with AOT-mandatory shipping targets).

Instead:

- **Adopt a compile-time execution-backend seam** inside SushiEngine (`SushiEngine::Execution`), with the existing SushiRuntime path as one implementation (`RuntimeBackend`) and a new native thread-pool/DAG implementation (`NativeBackend`) as the other. **Compile-time, not runtime-polymorphic** — a naive `virtual add_parallel(std::function<...>)` breaks the SYCL path outright, because `std::function` is not device-copyable and cannot be captured into `sycl::handler::parallel_for`. One binary only ever needs one backend anyway (a SYCL TU requires a SYCL compiler for the whole TU; project memory already records "one GPU vendor per build").
- **GPU compute for the two solvers that genuinely need it** (PGS, XPBD constraint solving) and the batch animation evaluator is a **fidelity tier**, not an architecture question — one of the three (batch animation) already has a shipping CPU twin, and the solver bodies are plain math with three `sycl::sqrt`/`fmod`/`floor` calls that need only a math-shim redirect.
- **The long-term answer for GPU compute on non-SYCL platforms is the RHI's compute-shader path (Wall 2), not SushiRuntime.** The engine will already have Vulkan/Metal/console compute shaders for rendering; writing PGS/XPBD/particles as AOT-compiled compute shaders behind an `IComputeDispatch` is bounded, known-cost work on a platform-blessed toolchain — unlike a second SYCL implementation.

### 4.2 Service decomposition (why this is smaller than it looks)

SushiRuntime provides three separable services with completely different portability profiles:

| Service | What it is | Who needs it | Portable natively? |
|---|---|---|---|
| **A — Task graph / scheduling** | CPU-side DAG: node registration by (reads, writes, capacity, live-count), compiled once, replayed per frame, RAW/WAR/WAW ordering, parallel execution of disjoint nodes | `Schedule`, `ConstraintSolver`, `XpbdSolver`, `DeviceBatchEvaluator`, `PhysicsWorld` | **Yes — mandatory on all 7 platforms, and arguably a better fit than SYCL for this workload.** The ECS emits one node per chunk (default capacity 1024) — well below GPU dispatch-saturation size; a CPU work-stealing pool executing whole-node ranges per worker, drawing parallelism from thousands of disjoint nodes, is a natural fit, not a downgrade. |
| **B — Genuine GPU compute** | Device-side kernels: PGS/XPBD solvers, batch animation, particle sim, **PINN-physics forward evaluator, RL-agent forward evaluator (both via SushiBLAS, added 2026-08-01)** | 2 physics solvers + 1 animation evaluator (audio's SYCL accelerator is already optional-by-default) + the PINN/RL evaluators | No — this is the fidelity tier. Batch animation already has a CPU twin (`animation/batch_evaluator.hpp`). **The PINN and RL evaluators have no CPU/portable twin planned — Windows/Linux-only by deliberate choice, not an oversight (§3.1).** |
| **C — USM allocator** | Host+device-visible shared memory for every ECS component column | `Chunk::Column::data` | Degenerates cleanly to plain aligned heap allocation with no GPU compute backend present — **except** for render interop, where unified-memory platforms (mobile, console) can recover the zero-copy property by having the *RHI* hand the allocator graphics-visible memory instead. |

### 4.3 The seam design

New namespace `SushiEngine::Execution`, header tree `include/SushiEngine/execution/`. Mirrors three precedents already in the codebase (`audio::IDspAccelerator`, `sim::ISimulation`, `render::rhi::IDevice`) — idiomatic, not novel.

- **Lifecycle** (`run`, `compile_count`, `node_count`, `reset`) may be virtual — called once per frame, cost is irrelevant.
- **Node submission** (`add_parallel<Fn>`, `add_host<Fn>`) **must stay templated on the kernel type** so the SYCL backend can forward the raw lambda into `parallel_for` unchanged.
- **Backend selection is a CMake-level compile-time policy** (`SE_EXECUTION_BACKEND=runtime|native`), not a vtable.

Core vocabulary types **(superseded 2026-08-01 — the vocabulary is now defined normatively in UHM §4, and it is richer than this sketch)**: `Execution::BufferInterval` (mirrors SushiRuntime's `Core::ResourceRegion` structurally — `{ResourceId base, offset, size}` with the key typedef'd for the runtime's planned `void*`→opaque-id migration), `Execution::AccessIntent` + `Execution::DeterminismClass` (the shared sim+render algebra), `Execution::NodeDescriptor` (name, intent-typed interval accesses, capacity, live-count provider, enabled provider, node kind incl. `Reduce`/`SegmentedReduce`), `Execution::RunReport`, `Execution::Graph` (duck-typed — both backends implement the same surface without a common base for the templated methods), `Execution::Context` (`create_graph()`, `allocate<T>(count, MemoryVisibility)`, `capabilities()`), and `Execution::Buffer<T>`.

**The allocator hook (do not skip this):** `MemoryVisibility::DeviceShared` on the native backend must accept an *injectable external allocator* so the RHI can supply `HOST_VISIBLE | DEVICE_LOCAL` memory on unified-memory platforms — without this hook, every frame's simulation output silently becomes a staging upload on mobile/console, invisibly regressing the zero-copy property `render/interop.hpp` exists to provide.

**Native backend internals** (ships as a compiled `sushi_exec_native` static library, not header-only — a thread pool doesn't belong in headers): DAG compilation to **UHM §5's contract** — *(corrected 2026-08-01: the original wording here asked for two incompatible things at once; "mirror `dependency_tracker.cpp`" and "linear-time" conflict, because the runtime's tracker is an append-only history scan that is O(A²) exactly in the disjoint-sub-range case. UHM resolves it: safety floor = every RAW/WAR/WAW conflict ordered, determinism floor = edge set a pure function of declaration order and regions, quality target for the engine core = last-writer-per-interval linear-ish construction; conformance = ordering-equivalence over conflicts, not edge-set equality)* — with a Chase-Lev work-stealing executor and a single-threaded determinism mode (needed because the existing deterministic-replay tests require it, and bit-identical output across SYCL-GPU and native-CPU is **not achievable** — FMA contraction and reduction-order differences mean the correct contract is bit-determinism *within* a backend, tolerance-comparison *across* backends).

### 4.4 Concrete call-site changes

`ecs/schedule.hpp` (`System::emit` retypes from `SushiRuntime::API::Graph&` to `Execution::Graph&`; the `graph.add(...)` call becomes `graph.add_parallel(NodeDescriptor{...}, kernel)` with the kernel lambda itself unchanged), `ecs/chunk.hpp` (`Column::data` becomes `Execution::Buffer<std::byte>`), `ecs/world.hpp` and `ecs/archetype.hpp` (constructor/member retype to `Execution::Context&`), `loop/app.hpp` (owns backend construction), `core/types.hpp` (gains a `Math::{sqrt,fmod,floor}` shim resolving to `sycl::*` under SYCL builds and `std::*` otherwise — this is already the file's documented job as "the single alias point"). Four `sycl::id<1>` kernel signatures convert to `std::size_t`. `audio/accelerator_sycl.hpp` needs **no change** — already correctly isolated.

**(Added 2026-08-01; retargeted 2026-08-01)** `physics/solver/runtime_graph_builder.hpp` joins this list as the fourth deep retarget site (it did not exist at the original survey): its `ElementRange` accesses map to `BufferInterval` offsets, `when()` to the enabled provider, `sized()`/`based_at*` to count/base providers, and its hand-built two-node reduction is deleted outright in favour of the `Reduce` node kind (which the RuntimeBackend lowers to the runtime's now-merged `add_reduce`).

**Net effect: SushiRuntime's blast radius across the engine collapses from 66 files touching it (re-verified 2026-08-01; was 52) to roughly 4 — the three ECS headers' replacement plus the physics graph builder's emission layer.** *(Realized 2026-08-01: three files outside tests and examples, plus `sim/runtime_simulation.cpp` which owns a runtime for the same reason `loop/app.hpp` does.)* This alone discharges the standing project guidance that "SushiRuntime API is unstable — keep call sites thin/isolated," independent of any platform work.

### 4.5 Milestones

| Code | Delivers | Size |
|---|---|---|
| **RUNTIME-PORT0** | **DONE 2026-08-01 (with UHM0); exit oracle green.** Extract the `Execution` seam behind the existing SYCL path, zero behavior change. Ships alone, standalone value. Exit: `sandbox` exits 0, `compile_count==1`, every example/test builds, byte-identical output logs — all met: `se test --suite functional` is 1108/1108 and `sandbox` reports `compile_count=1 mismatches=0 RESULT: OK`. Every §4.4 site plus the standalone solvers and the batch animation evaluator are on the seam; the `Math` shim landed with its three real consumers. Blast radius outside tests and examples: 38 files → 3, all structural (the adapter, the composition root, the optional SYCL DSP accelerator). See UHM §9's "what landed, and what did not". | Medium |
| **RUNTIME-PORT1** | Native thread-pool+DAG backend, validated on **Linux first** as a same-OS control (isolates the backend variable from the platform variable). `-DSUSHI_EXEC_BACKEND=native` builds `sandbox`/`pgs_demo` with a stock compiler, no SYCL toolchain. | Large |
| **RUNTIME-PORT2** | Native backend on Windows and macOS; SYCL-free CI matrix; first-ever stock-compiler build on macOS. Watch item: macOS has neither `SetThreadAffinityMask` nor `pthread_setaffinity_np` semantics SushiRuntime assumes — needs a no-op/QoS-class path. Also first opportunity to run ASan/UBSan/TSan on the ECS (painful-to-impossible under `-fsycl`). | Medium |
| **RUNTIME-PORT3** | Simulation fidelity tiering (`SimulationProfile::{Full,Reduced}`) — CPU paths for PGS/XPBD, route batch animation to the existing CPU evaluator, cap particle/entity budgets per tier. | Medium |
| **RUNTIME-PORT4** | Android bring-up (NDK, arm64-v8a). Extends `.deps.toml` schema with an `android_ndk` field (cross-repo, touches the `ss`/`se` CLI). First non-desktop ECS boot. | Large |
| **RUNTIME-PORT5** | iOS bring-up. Static-link only, AOT-only, no dynamic codegen anywhere in the shipped path. Reduced tier mandatory. | Medium |
| **RUNTIME-PORT6** | Console bring-up. Native backend is the sole enabler. NDA-gated — **explicitly deferred, not estimated**, until devkit access exists. | Large, blocked |
| **RUNTIME-PORT7** | Two timeboxed SushiRuntime portability spikes (1 week each): (a) AdaptiveCpp `omp.library-only` under AppleClang — would give SushiRuntime itself a CPU-only macOS path if it still works, structurally equivalent in outcome to PORT2 either way; (b) audit SushiRuntime's non-SYCL deps (hwloc, TCP transport) for mobile viability. **Explicitly out of scope: SYCL-on-Metal, a SushiRuntime Vulkan backend.** | Small each |
| **RUNTIME-PORT8** | RHI compute path for GPU sim on non-SYCL platforms (`Render::IComputeDispatch`, AOT SPIR-V/MSL/console compute shaders for PGS/XPBD/particles). This — not a SushiRuntime backend — is where mobile/console GPU-driven simulation eventually comes from. Start only after PORT4 ships and profiling proves CPU sim is the actual bottleneck. | Very Large, long horizon |

### 4.6 Critical-path statement

`ecs/world.hpp` and `ecs/schedule.hpp` both `#include <SushiRuntime/SushiRuntime.h>`, which requires a SYCL toolchain. **Today, no translation unit that touches the ECS can be compiled at all for macOS, iOS, Android, PS4, or PS5** — not the game, not a test, not a "hello triangle" spike that happens to construct a `World`. This is stronger than "the simulation is slow on mobile": a perfect Metal renderer on iOS would have nothing to draw, because nothing that owns entities can currently be compiled alongside it. **RUNTIME-PORT0 and RUNTIME-PORT1/2 gate all non-desktop bring-up across every other wall.** Start RUNTIME-PORT0 immediately, in isolation — it has a clean pass/fail oracle (`sandbox`), ships no behavior change, and nothing downstream needs anything else to be done first.

---

## 5. Wall 2 — Multi-backend RHI

### 5.1 Recommendation

Layer three tiers behind the existing outer seam, in this order: **(A) vocabulary** (enums/PODs, zero backend types) → **(B) `ICommandList`** (recording) → **(C) `IDevice`** (resource creation/pipelines/submission). Ship a golden-image regression harness and a text-tracing `TraceCommandList` **before touching any pass** — this is not optional given zero existing render tests. Use Android/Vulkan-mobile as the *second* proving ground before Metal (same backend family, better tooling — validation layers, RenderDoc — for the hard portability bugs everyone will hit anyway). Ship Metal as the real Apple backend; use MoltenVK only as a day-one bring-up crutch and permanent differential-testing oracle, never as the shipping target (it cannot do mesh shaders or meaningful ray tracing today, and it over-synchronizes barriers by design). For shaders: bridge through SPIRV-Cross (byte-identical SPIR-V still ships to the Vulkan backend — the shipping path cannot regress from shader work), migrate to Slang once a neutral binding layout exists to consume its reflection output, then **delete the bridge in the same milestone that finishes the migration.**

### 5.2 Why the abstraction is smaller than 39,894 lines suggests

A full audit of `render/` found the actual Vulkan *vocabulary* is tiny: **15 real `VkFormat` values, 8 image-usage bits, 10 buffer-usage bits, 21 distinct `vkCmd*` verbs** used anywhere in the library. A neutral vocabulary layer is a few hundred lines. The harder 60% of the problem is **resource creation** (26 files own `VmaAllocation` members directly), not command recording — size the milestones accordingly; don't declare victory after porting `vkCmd*` calls.

Also load-bearing: **every one of the 19 compute passes that hand-issue barriers (re-verified 2026-08-01; grown from 13 with the atmosphere work) declares zero attachments** — no pass mixes a rendering scope with manual barriers, which is exactly the invariant that makes a clean Metal encoder mapping possible. Enforce it as a debug-build assertion going forward, don't just rely on it holding by convention. (One notable outlier discovered in re-verification: the atmosphere nest runs *outside* the render graph entirely, synchronized by its own timeline semaphore that the frame's submissions wait on — `vulkan_scene_view.cpp:829-850`. UHM §6 adopts this as the precedent for its T2 completion tokens.)

### 5.3 The barrier design — the single highest-leverage decision

Express barriers as **access-intent transitions**, not raw Vulkan stage/access/layout masks:

```cpp
struct TextureBarrier
{
    TextureId texture;
    TextureSubresource range{};
    Graph::TextureAccess from = Graph::TextureAccess::Undefined;
    Graph::TextureAccess to   = Graph::TextureAccess::Undefined;
    Graph::PassQueue from_queue = Graph::PassQueue::Graphics;
    Graph::PassQueue to_queue   = Graph::PassQueue::Graphics;
};
```

This costs **nothing** on the shipping Vulkan backend — `render/graph/resource_state.cpp` already implements the forward mapping (`TextureAccess → {stage, access, layout}`) as a total function; the Vulkan backend's `barrier()` is just that existing table relocated. But it is the difference between a Metal/console backend being *possible* and being a reverse-engineering exercise: Metal has no image layouts at all (synchronization is `MTLFence`/`memoryBarrierWithScope:`), and a `(stage, access, layout)`-shaped barrier cannot be translated into that model without inferring intent backward. `TextureState`/`BufferState` collapse from three Vulkan-typed fields to `{access, queue}` as a direct consequence — a simplification, not just a rename.

### 5.4 The binding-model split

Today's push-descriptor set 0 conflates two unrelated things: **26 frame-global bindings** (scene uniforms, lights, shadow atlases, GI, atmosphere LUTs — identical every pass) and **6 pass-local bindings** (whatever a given pass samples). It is already documented as full at its guaranteed-minimum 32 entries — a live constraint today, independent of any porting. Split into three backend-agnostic mechanisms instead of unifying into one generic system (a generic `BindGroupLayout` would be speculative machinery for variation that doesn't exist):

1. **`FrameResourceTable`** — the 26 frame-global bindings, built once per frame, bound once per command list (one Vulkan descriptor set bind instead of ~26 pushes × 39 passes — likely a **net CPU win** on the shipping backend, not just a portability tax). Maps to one Metal argument buffer, one console descriptor table.
2. **`BindlessHeap`** — unchanged conceptually; `Resources::DescriptorHeap`'s public interface (`allocate_texture`/`free_texture`) survives verbatim across all three backends, only the body changes. Requires Metal `MTLHeap`-backed residency (`useHeap:` once per encoder) — must land in the Metal backend's `create_texture` from day one, not retrofitted.
3. **`InlineConstants`** — the existing 128-byte push-constant contract, unchanged (a safe floor under Metal's 4KB `setBytes` and typical console fast-constant registers).
4. **The 6 pass-local bindings fold into the heap** via a new `PassContext::bindless_index(TextureHandle, SamplerId)`, backed by the existing steady-state texture-pool reuse (`TexturePool::Entry` gains a persistent bindless slot allocated once and freed on retirement) — **zero additional per-frame heap writes in steady state.**

This single change is independently justified four ways: required for Metal, required for a generic console descriptor-table model, required to drop the Android version floor (push descriptors are core only in Vulkan 1.4; merely an extension on the 1.3 baseline Android needs), and a likely Vulkan CPU-time improvement.

### 5.5 Shader strategy

GLSL usage was audited feature-by-feature: **zero subgroup intrinsics anywhere** (the hardest cross-compilation problem doesn't exist in this codebase); `shared`/`barrier()`/`atomicAdd` all map cleanly to MSL; bindless (`nonuniformEXT`, 6 files) needs a generated resource-binding remap table; mesh shaders (2 files) and ray query (1 file) are the genuinely hard cases — but **both are already capability-gated** in the engine (`supports_mesh_shader()`/`supports_ray_query()` with existing fallback paths), so they can simply be switched off on the first Metal pass rather than solved immediately.

Recommendation: **SPIRV-Cross now, Slang later, delete the bridge in the same milestone that finishes the Slang migration.** SPIRV-Cross keeps the SPIR-V fed to the shipping Vulkan renderer byte-identical to today's — a hard requirement given there are no render tests. Slang is the correct destination (genuine multi-target codegen, generics that would collapse GI-tracer-tier and material-permutation duplication, reflection that generates the binding-layout artifact §5.4 needs instead of hand-maintained remap tables that can silently drift) but shouldn't be adopted until the binding-model split (§5.4) exists to consume it, and its cost (rewriting ~58 shader entry points against an unverifiable-by-tooling bit-exactness invariant) is only justified once the destination is real. One cheap early spike is worth doing regardless: Slang has a GLSL front-end that might consume the existing shaders and emit MSL directly, potentially collapsing both options into one — timebox 3 representative shaders, let the result decide.

### 5.6 Milestones

| Code | Delivers | Size |
|---|---|---|
| **RHI0** | Golden-image regression harness (per-pass hashes, deterministic N-frame render) + `TraceCommandList` (a text-diffable second `ICommandList` implementation, provable in CI with no GPU). **Zero production code changes.** This is the safety net the rest of the programme runs on. | Medium |
| **RHI1** | Neutral vocabulary (`Rhi::types.hpp`/`handles.hpp`); collapse `TextureState`/`BufferState`; one Vulkan conversion TU. Slang GLSL-front-end spike. Exit: `render/graph/` no longer includes `vulkan.h`; trace byte-identical. | Small–Medium |
| **RHI2** | `ICommandList` (21 verbs); port **recording only** across all 39 passes (95 `VkCommandBuffer` sites). Exit: zero `vkCmd*` under `render/passes/`; golden images identical. | Large |
| **RHI3** | `IDevice` + generation-tagged handle tables; port all resource-owning systems (texture/buffer pools, samplers, pipelines, descriptor heap, material/light/instance systems, all pass-owned resources). Exit: zero `vulkan.h`/`vk_mem_alloc.h` outside `render/rhi/vulkan/`; `sushi_render` links Vulkan **PRIVATE**. | Large |
| **RHI4** | Binding-model split (§5.4): `FrameResourceTable`, retire push descriptors, ~40 GLSL files move to heap indexing. Exit: no `vkCmdPushDescriptorSet` anywhere; CPU record time no worse (expect better). | Large |
| **RHI5** | Capabilities + version-floor drop: Vulkan 1.4→1.3 floor, `maintenance5`/`6`/pushDescriptor demoted to optional-with-fallback, hardcoded depth format → negotiated, BC7/ASTC texture compression, memory-budget query. Exit: renders correctly with mesh shaders/RT/VRS/GPL all absent. | Medium |
| **RHI6** | Android bring-up: NDK build, `VK_KHR_android_surface`, tiler-aware pass tuning (Load→Discard audit), on-device golden capture. **Serves as the rehearsal for Metal, with validation layers and RenderDoc still available** — every hard portability bug (version floor, format negotiation, texture compression, capability gating) gets hit here first, with better tools than Metal will offer. | Medium–Large |
| **RHI7** | Metal backend: triangle + 3 proof passes chosen to maximize coverage — `tonemap_pass` (fullscreen graphics baseline), `hiz_pass` (compute, read-write storage image format restrictions, dependent dispatch chain), `opaque_pass` classic path (argument-buffer + `MTLHeap` residency, per-instance inline-constant hot loop). SPIRV-Cross MSL path stood up. MoltenVK differential oracle alongside. | Large |
| **RHI8** | Metal pass parity — remaining ~30 passes not requiring mesh shaders/RT/VRS. | Large |
| **RHI9** | Slang migration, one shader at a time, each gated on an unchanged golden image; SPIRV-Cross + glslang deleted from the renderer and tool in this same milestone. | Large |
| **RHI10** | Metal tiered features (mesh/object shaders, acceleration structures, rasterization-rate-map VRS analogue) — opportunistic, each behind an existing capability flag. | Medium |
| **RHI11** | Console-seam proof, **no console code**: the backend factory (`create_render_device()`) becomes a registry a private CMake subdirectory can populate without the public tree ever naming the backend. This single change is the entire console-readiness budget for the renderer. | Small |

Rough calibration: RHI0–5 ≈ 6–9 engineer-months solo; RHI0–8 (Metal parity) ≈ 12–18; RHI0–9 (Slang done) approaching two years solo, roughly halved with two engineers after RHI2 since RHI3/RHI4 partially parallelize.

### 5.7 Explicit non-goals for this wall

No generic bind-group/descriptor-layout validation system (one frame table + one heap, forever, by design). No abstracted memory allocator (each backend's allocator model — VMA, `MTLHeap`, console placement — has no useful common interface; abstract only `create_texture`/`create_buffer`). No D3D12 backend "for a third data point" (Windows already works on Vulkan). No speculative console hooks beyond the four already independently justified by Metal (access-based barriers, no push-descriptor dependency, 128-byte constants, backend registry). No template-based `ICommandList` (kills runtime backend selection and compile times). Do not merge RHI2 and RHI3 into one diff.

---

## 6. Wall 3 — Editor/runtime split & platform layer

### 6.1 Recommendation

The library graph is already correctly factored — `sushi_render`/`sushi_input`/`sushi_audio` are plain static libs with no editor or SYCL knowledge; `sushi_sim` is the one isolated SYCL TU. What's misplaced lives inside `editor/`: the window abstraction, and scene/effect serialization. Extract two new libraries (`sushi_platform`, `sushi_scene`) and build a lean `se_player` executable around a `PlayerApp` that exposes `start()/frame()/suspend()/resume()/shutdown()` — **not** a `while(running)` loop, because that shape cannot run on iOS (UIKit owns the run loop) and is awkward on Android (the OS can destroy the rendering surface between frames). Doing this refactor on Windows first, before any mobile work, means every later platform-driven interface change gets designed and debugged with a real debugger instead of on a device with an hour-per-iteration edit cycle.

### 6.2 The two biggest gaps found

1. **There is no path from a rendered scene to the swapchain except through ImGui.** `ISceneView` renders offscreen and only exposes a *sampled* texture; `IWindowRenderer::begin_frame()` hands back a raw command buffer that today only `ImGuiBackend::render()` records into. `vulkan_window_renderer.cpp` contains no blit/copy/composite path. **A player cannot draw a full-screen scene without a new renderer entry point** (`IWindowRenderer::present_scene_view()`) — this is the highest-risk item in the first milestone.
2. **The renderer bakes a developer build-tree path into the shipped library.** `SUSHI_PIPELINE_CACHE_DIR` defaults to `${CMAKE_BINARY_DIR}` and is consumed unconditionally — a shipped player would try to write its driver pipeline cache into the *developer's* build directory, a guaranteed failure on any real device and a sandbox violation on iOS/Android/console.

### 6.3 Windowing: split `IPlatformWindow` into three interfaces

The current single interface conflates lifecycle, geometry, and a hardcoded Vulkan surface API. Splitting avoids widening one interface with mobile/console-only methods every implementation would have to stub:

- **`IPlatformHost`** — lifecycle + event pump, present on every target including console. Gains a typed `HostEvent` enum (`Quit`, `WillSuspend`, `DidSuspend`, `WillResume`, `DidResume`, `SurfaceLost`, `SurfaceCreated`, `Resized`, `LowMemory`, `OrientationChanged`, `FocusGained/Lost`) — this is the one genuinely new concept, and it turns out console needs **zero additional concepts** beyond it (suspend/resume/surface-loss already covers a console's "system suspend" and "user sign-out" events), which is the strongest argument for doing this split now rather than bolting mobile-only methods onto the existing interface later.
- **`IDisplaySurface`** — geometry: `drawable_size`, plus new `content_scale` (HiDPI), `safe_area` (notch/home-indicator insets — zero on desktop/console, real on phones **and** consoles' title-safe margins), `orientation`, and `surface_generation()` (a generation counter the renderer compares against to know when to rebuild the swapchain — the cheapest possible fix for Android's surface-lost/recreate cycle).
- **`IVulkanSurfaceProvider`** — the Vulkan-specific half, *queried for*, not inherited by everyone. A future Metal or console backend asks for a different provider interface instead.

Android and iOS specifics: **SDL2's official backends do the heavy lifting** — a Java `SDLActivity` shim over `NativeActivity`/`ANativeWindow` for Android (ships with SDL2, is the standard integration path, not custom glue), and `UIApplicationMain`/`SDL_iPhoneSetAnimationCallback` integration for iOS (which is *why* the `frame()`-callback shape in §6.1 is a hard prerequisite, not a nicety). Concrete platform requirements: Android needs the player built as `libmain.so` (a shared library loaded by the Java activity, not an executable) plus a Gradle wrapper project; both need `HostEvent::WillSuspend` to tear down the swapchain and `SurfaceCreated`/`DidResume` to rebuild it, driven by an `SDL_AddEventWatch` for the `SDL_APP_*` events (which arrive synchronously, outside the normal poll loop, and must be handled promptly or the OS kills the process).

### 6.4 Input and audio extension — smaller than expected

**Touch is already fully implemented and simply unwired.** `SdlInputTranslator` already translates `SDL_FINGER*` events; `VirtualControlSource` (on-screen sticks/buttons) and `GestureRecognizer` (tap/long-press/drag/pinch) both exist and are unit-tested — but `set_display_size()` is never called by any host, and neither object is ever instantiated. The action layer needs **zero new event types** for touch. It needs exactly two new event types (`GamepadSensor` for gyro/accel, `TouchpadDown/Move/Up` for console-pad capacitive touchpads — deliberately *not* reusing the screen-touch event types, since a controller touchpad must never feed the on-screen pointer table) plus one new, separately-queried `IExtendedHapticsSink` interface for console-class haptics (adaptive-trigger-style effects, LED) that must **not** widen the existing minimal `IHapticsSink::rumble()` seam — that would force every implementation, including test doubles, to implement effects they can't play.

**Audio is already covered natively on 5 of 7 platforms with zero new dependencies** — the vendored miniaudio documents native backends for WASAPI (Windows), CoreAudio (macOS **and** iOS), AAudio/OpenSL|ES (Android), and ALSA/PulseAudio (Linux). What's missing is build-flag plumbing, not code: macOS/iOS need `MA_NO_RUNTIME_LINKING` (miniaudio's runtime framework linking **fails Apple notarization** outright) plus the CoreAudio/AudioToolbox framework links, and the iOS TU specifically must compile as Objective-C. Console audio has no third-party coverage at all and needs a vendor-specific `IAudioDevice` implementation — structurally identical in shape to how `render/rhi/vulkan/` is today's only RHI backend.

### 6.5 Build system rework

- **`cmake/Platform.cmake`** (new): derives `SE_PLATFORM_{WINDOWS,LINUX,MACOS,IOS,ANDROID,...}` plus rollups (`SE_PLATFORM_DESKTOP/MOBILE/CONSOLE/APPLE/POSIX`) from `CMAKE_SYSTEM_NAME`/`ANDROID`/`IOS`, and defines the `SE_PLATFORM_*` C++ macros the codebase currently lacks entirely (code today reaches for raw `_WIN32`).
- **Toolchain files**: Android NDK's `android.toolchain.cmake` (standard, NDK-provided); iOS via native `-G Xcode -DCMAKE_SYSTEM_NAME=iOS` (CMake has first-class support since 3.14 — prefer this over a third-party toolchain file); Windows/Linux unchanged; console via a vendor-provided CMake module plugged in through `CMAKE_TOOLCHAIN_FILE`, kept in a private, non-public overlay directory so nothing NDA-covered ever enters this repo.
- **`cmake/Vcpkg.cmake`** needs to stop being a Windows-only no-op and instead chainload correctly for cross-compiles (`VCPKG_CHAINLOAD_TOOLCHAIN_FILE` pointing at the NDK/iOS toolchain) — this is vcpkg's documented cross-compilation mechanism. Add a root `vcpkg.json` manifest (none exists today) to stop the CI job's hand-rolled package list from drifting against `sushistack.deps.toml`.
- **`.deps.toml` schema evolves additively** (`schema = 2`, new `[pkg.platforms.<os>]` tables) so the existing `linux_apt`/`windows_vcpkg` fields — and the external `ss` CLI that reads them — never break.
- **Target-helper decomposition**: don't conflate "does this target compile SYCL" with "what OS-level artifact shape does it produce" (`.exe` vs `.app` bundle vs `libmain.so` vs console package) into one macro name — keep them as two orthogonal, composable layers (`add_sushi_app()` for artifact shape, `sushi_apply_sycl()` for the compute axis).
- **`show_in_explorer`/`open_with_default_app`** (corrected 2026-08-01: both live in `editor/project/project_panel.cpp`, not `editor_panels.cpp`, and only *one* of the two is hazardous — `open_with_default_app` already uses `ShellExecuteW` with no shell round-trip, its own comment saying why) are the representative case for the whole "Windows-only for now" pattern: replace `show_in_explorer`'s `std::system()` call (`project_panel.cpp:147` — the command-injection/quoting hazard is real, and confined to that one function) with `SDL_OpenURL()` where "open" semantics suffice and `posix_spawn`-based platform branches for "reveal in file manager" (no cross-platform primitive exists for this one). The same fix collapses the scattered `#ifdef _WIN32` path-resolution forks onto `SDL_GetPrefPath()`.

### 6.6 CI matrix expansion

Refactor first (extract composite actions for the SYCL/vcpkg bootstrap, currently duplicated verbatim between jobs, and add binary caching) before adding platforms — with a 7-platform matrix, an uncached SYCL nightly download multiplied across jobs burns runner budget fast. Then add, in order, with each new lane starting `continue-on-error: true` until proven stable: `functional-windows`/`editor-windows` (Windows has **zero CI today** despite being the primary dev platform — highest-value addition), `player-*` jobs (need the headless mode from §6.7), `libs-macos` (compile-only — no SYCL target exists there), `android-build` (cross-compile only; GitHub-hosted emulators have flaky Vulkan support, don't run render smoke tests there — that needs a self-hosted device runner), `ios-build` (macOS runner + simulator). **Console CI must never touch GitHub-hosted infrastructure** — self-hosted runners only, in a separate workflow file gated behind a protected environment with required reviewers, artifact upload disabled.

### 6.7 Milestones

| Code | Delivers | Size |
|---|---|---|
| **PLATFORM0** | `sushi_platform`/`sushi_scene` extracted from `editor/`; `se_player` with the `start/frame/suspend/resume/shutdown` shape; `IWindowRenderer::present_scene_view()` (the highest-risk item); boot manifest; runtime-resolved pipeline-cache path; `--headless` mode (enables CI without a GPU runner); `se_editor` unaffected, now linking the extracted libraries. Windows only — no new OS support. Proves the editor/runtime boundary is real: `se_player` never links `sushi_imgui`, and that one link failure *is* the enforcement mechanism. | Large |
| **PLATFORM1** | Linux parity: `cmake/Platform.cmake`, the shell/path fixes (§6.5), `install()`+CPack rules, CI refactor, root `vcpkg.json`, `.deps.toml` schema v2. First shippable artifact on two platforms. | Medium |
| **PLATFORM2** | Windowing/lifecycle interface split (§6.3) — exercised on **desktop** first (minimize/restore, alt-tab as stand-ins for suspend/resume) before any mobile hardware is involved; touch wiring validated with mouse-as-pointer. | Medium |
| **PLATFORM3** | *(= RHI5 in §5.6 — renderer-side prerequisite, listed here only because Android/iOS bring-up cannot start without it.)* | — |
| **PLATFORM4** | Android: NDK/Gradle integration, `SDLActivity`-based windowing, asset mounting via `AAssetManager`, suspend/resume proven on a real device, on-screen controls rendered, miniaudio AAudio wiring. **Flag explicitly at kickoff**: `se_player` links `sushi_sim` → SushiRuntime → SYCL, which almost certainly has no Android target — either RUNTIME-PORT4 must land first, or PLATFORM4 ships a render/input/audio-only Android host with simulation stubbed, and full gameplay follows later. This is the single biggest scheduling unknown in the whole plan; decide it explicitly, don't let it default. | Large |
| **PLATFORM5** | Asset packing (`sushipak` format, a cook tool) + `SE_SHIPPING` config (hot-reload/glslang compiled out, mount-table-only paths). Benefits every platform; sequenced here because Android is what makes it unavoidable. | Medium |
| **PLATFORM6** | macOS: libraries + CI (compile-only — no player without RHI7/8 or RUNTIME-PORT2), Apple miniaudio fixes, `.app` bundle + codesign/notarize scaffolding. | Medium |
| **PLATFORM7** | *(= RHI7/8 in §5.6 — Metal backend, separate rendering effort, noted as a dependency for a first-class Apple player.)* | — |
| **PLATFORM8** | iOS: Xcode-generator CMake lane, `SDL_iPhoneSetAnimationCallback`-driven frame loop, `Info.plist`/signing/launch-screen, safe-area-driven UI layout, `AVAudioSession` configuration, simulator CI smoke test. | Large |
| **PLATFORM9** | Console bring-up: private CMake overlay, `FixedOutputSurface` implementing `IPlatformHost`/`IDisplaySurface`, vendor `IAudioDevice`, `IExtendedHapticsSink` implementation, self-hosted environment-protected CI. Every piece plugs into a seam this plan already establishes on open platforms — the entire point of sequencing console last. NDA-gated, cannot start without devkit access. | XL, blocked |

---

## 7. Per-platform engineering dossier

Everything below is public, vendor-documented, or widely-reported knowledge. Where real detail sits behind an NDA'd SDK (exact PS5/PS4 compiler flags, TRC checklist item numbers, GNM API surface), that boundary is called out explicitly rather than guessed at. Sections are ordered by platform priority (§0.0): Windows/Linux, then Android, then macOS/iOS, then PS5/PS4 last.

### 7.1 Windows (existing home platform)

Already works — no graphics-backend, no audio-backend work needed. MSVC is the reference compiler; `clang-cl` is a fully viable, ABI-compatible alternative already effectively in use since intel/llvm's `-fsycl` runs in a clang-cl-compatible mode on Windows. Vulkan is fully first-class here (all three IHVs ship conformant, maintained ICDs) — the tradeoff already accepted by standardizing on Vulkan is giving up PIX in exchange for the shared Linux/Android/(MoltenVK)/macOS codebase. Packaging: NSIS/Inno Setup for a standalone or Steam-style build (Steam needs no special packaging format, just a runnable tree), or MSIX for Microsoft Store distribution. No console-grade certification for direct/Steam distribution.

### 7.2 Linux

Graphics/audio need no backend work (Vulkan and ALSA/PulseAudio are both already the engine's native targets). The real complexity is **which Linux** — glibc/libstdc++ ABI fragmentation across distros, not compiler choice (Clang is already required for SYCL anyway). The industry-standard answer is Valve's **Steam Runtime** containers (currently "sniper," Steam Runtime 3, built from a stable Debian/Ubuntu LTS base) — build inside the published container image rather than targeting a floating `ubuntu-latest`. Windowing needs both X11 and Wayland paths tested (SDL2 supports both, but fullscreen/HDR/cursor-confinement semantics differ enough to need real per-backend testing). Packaging: AppImage or Flatpak for direct distribution; Steam itself needs no separate installer, just the Steam-Runtime-built binary tree. Steam Deck (fixed-spec SteamOS, ~16GB shared LPDDR5) has its own "Verified" compatibility program, functionally closer to a console cert process than general desktop Linux.

### 7.3 Android

Vulkan is the modern native API — **Vulkan 1.1 has been required for new 64-bit devices since Android 10**, tightened further with Android 15 — but real fragmentation exists in driver quality/extension support across Adreno/Mali/PowerVR, and mobile GPUs are, like iOS, overwhelmingly TBDR with lower descriptor-set limits than the desktop-oriented design currently assumes. Toolchain: NDK-provided Clang (GCC was fully removed from the NDK years ago), driven via `android.toolchain.cmake`, with Gradle as the unavoidable outer build orchestrator (it's what produces an installable package and mediates the JNI boundary) — a thin Gradle wrapper is required even for an otherwise CMake-driven engine. Input/windowing: SDL2's official Android backend (`SDLActivity`, `ANativeWindow`/`SurfaceHolder.Callback2` lifecycle, JNI-bridged) is production-shipping, not custom work; the player builds as `libmain.so`, not an executable. Audio: AAudio (modern, 8.0+) with OpenSL|ES fallback, both natively supported by the already-vendored miniaudio. Packaging: `.aab` (Android App Bundle) has been **required by Google Play since August 2021**, built via Gradle; Play App Signing (Google holds the final signing key) is the standard model since the same date. Certification: policy-based/largely automated (content rating via IARC, permissions declarations), substantially lighter than console TRC. Memory: unified, the widest device spread of any platform in this list (roughly 3GB budget devices to 12GB+ flagships), with the LMK (low-memory killer) aggressively reclaiming backgrounded processes.

### 7.4 macOS

Metal is the only fully native low-level graphics API — no native Vulkan or D3D. **MoltenVK** (Khronos, open source) translates a CTS-conformant Vulkan 1.0–1.4 core subset onto Metal, but as of current public status, `VK_KHR_ray_tracing_pipeline` is **not implemented** and `VK_EXT_mesh_shader` support is an **open, unmerged PR** — so anything the engine does with ray tracing or mesh shaders will not work through MoltenVK today, independent of the native-Metal-backend timeline (§5). Toolchain: Xcode + Apple Clang (a separate LLVM fork/cadence from the intel-llvm SYCL toolchain — two different compiler installs even though both descend from LLVM). Audio (CoreAudio via miniaudio) and input (MFi/`GCController`, which covers DualShock/DualSense/Xbox pads over Bluetooth) are both well-covered. Packaging: `.app` bundle, Developer ID signing, **mandatory notarization** (unsigned/un-notarized apps are hard-blocked by Gatekeeper since Catalina) — this is also why miniaudio's runtime framework linking must be disabled (`MA_NO_RUNTIME_LINKING`). Apple Silicon uses unified memory (no discrete VRAM budget concept).

### 7.5 iOS

Metal-only, same MoltenVK caveats as macOS. GPUs are **tile-based deferred renderers (TBDR)** — render-pass load/store discipline has real performance impact regardless of graphics backend. UIKit lifecycle: the OS can suspend/background/terminate at any time; general render/game logic must **not** run in the background; SDL2's official iOS backend (`SDL_iPhoneSetAnimationCallback` driven off `CADisplayLink`) is production-shipping, not custom work — which is why the `PlayerApp::frame()` callback shape (§6.1) is a hard prerequisite, not a preference. Touch via UIKit→SDL2; physical controllers via `GCController` (officially covers DualShock 4/DualSense/Xbox over Bluetooth since iOS 13, with haptics). Audio: CoreAudio via miniaudio, but session category/interruption handling (`AVAudioSession`) is the app's own responsibility since the engine doesn't route through SDL audio. Packaging: `.ipa` via Xcode Archive → App Store Connect / TestFlight, paid Apple Developer Program membership ($99/yr) required, provisioning profiles + signing certificates mandatory. Certification: public App Review Guidelines (stability is an automatic-rejection category; private-API usage is statically checked). Memory: unified, 3–8GB device totals with an app-available budget well below that and enforced by the jetsam killer — treat any specific number as a device/OS-version-dependent rule of thumb, not a constant.

### 7.6 PlayStation 5

Everything here is stated at the public-knowledge level; the project currently has no PlayStation Partners registration or devkit access, which blocks **all** technical work on this platform as a business/legal prerequisite, not something engineering can derisk in advance. Graphics: **GNM**, a low-level API in the Vulkan/D3D12 philosophical family but entirely PlayStation-specific (not Khronos/Microsoft-spec), shaders in **PSSL** (HLSL-derived) — no public Vulkan/D3D12 driver exists for PS5. Input: **DualSense** with adaptive triggers and multi-band haptics well beyond generic controller rumble, reachable only through Sony's proprietary SDK (no public/SDL2 exposure of the advanced features). Audio: proprietary stack including the publicly-named **Tempest 3D AudioTech** engine — no third-party library, including miniaudio, covers it; a dedicated first-party backend is required once devkit access exists. Lifecycle: system-level **Rest Mode** suspend/fast-resume, which titles must correctly save/restore state around (part of certification). Memory: a fixed, publicly-known **16GB unified GDDR6** total (OS reserve and exact app-available figure are NDA'd). Certification: Sony's **Technical Requirements Checklist (TRC)** — publicly known category areas include suspend/resume behavior, save-data handling, trophy implementation, and standard system-UX conventions, without inventing specific checklist item numbers. Packaging: a `.pkg`-style format via devkit tooling, submitted through a registered-publisher-gated pipeline.

### 7.7 PlayStation 4 (no PS3)

Same NDA boundary as PS5, older SDK generation (internally referred to by developers as "Orbis," analogous to PS5's "Prospero"). Graphics: **GNM** and **GNMX** (a higher-level, more-overhead convenience wrapper Sony provided over GNM), sharing PSSL lineage with PS5. Hardware is materially weaker (8-core AMD Jaguar CPU, GCN-generation GPU — no hardware ray tracing, no mesh shading), which matters both for available renderer features and for CPU-side task-scheduling budget relevant to the `Execution` native backend (§4). Input: **DualShock 4** — touchpad, light bar, dual-motor rumble, no adaptive triggers (PS5/DualSense-only). Audio: proprietary Orbis-generation libraries, no Tempest 3D AudioTech. Memory: a fixed, publicly-known **8GB unified GDDR5** total. Certification/packaging: same TRC concept and `.pkg`-style pipeline as PS5, predating it, same registered-publisher gate.

### 7.8 Cross-cutting notes

Windows and Linux need no graphics-backend work at all — the real gap on both is packaging/CI infrastructure that doesn't exist yet. macOS/iOS are where "keep Vulkan everywhere via MoltenVK" has a real, currently-unresolved cost (RT/mesh-shader gaps) — any renderer feature depending on those needs a native Metal path regardless of the desktop design. Android is architecturally the closest fit to the existing Vulkan-first, miniaudio-covered design, but carries the most severe hardware/driver fragmentation of any platform here. SDL2's Android and iOS backends are both real and production-proven — the work is *integration* (Gradle wrapper, Xcode project/UIKit delegate), not backend authorship. PS5/PS4 are categorically different from the other five: essentially none of the required engineering work can begin without first securing a PlayStation Partners/publisher agreement — a business prerequisite that blocks all technical work, not an engineering task.

---

## 8. Unified sequencing

### 8.1 Platform readiness matrix — what must land before a platform is "real"

Rows are ordered by platform priority (§0.0): Windows/Linux, then Android, then macOS/iOS, then PS5/PS4.

| Platform | Wall 1 (`RUNTIME-PORT*`) | Wall 2 (`RHI*`) | Wall 3 (`PLATFORM*`) | External blocker |
|---|---|---|---|---|
| **Windows** | works today; PORT0 recommended for hygiene | works today | PLATFORM0 (player split) | none |
| **Linux** | works today; PORT1 (native backend as a control) optional but recommended | works today | PLATFORM1 | none |
| **Android** | **PORT4 required, or explicit stub-sim decision at kickoff** | RHI5 (1.3 floor — mandatory, not optional) + RHI6 | PLATFORM2 + PLATFORM4 | none |
| **macOS** | **PORT2 required** (no SYCL toolchain exists) | RHI5 (version floor) + RHI7/8 (Metal) or MoltenVK interim | PLATFORM6 | none |
| **iOS** | **PORT5 required** | RHI5 + RHI7 (Metal/MoltenVK) | PLATFORM2 (frame-callback shape) + PLATFORM6 + PLATFORM8 | Apple Developer Program enrollment |
| **PS5** | PORT6 | RHI11 (registry seam) + a private GNM backend | PLATFORM9 | **PlayStation Partners registration + devkit** |
| **PS4** | PORT6 | RHI11 + a private GNM/GNMX backend | PLATFORM9 | **PlayStation Partners registration + devkit** |

### 8.2 Recommended overall order

1. **Immediately, in parallel, on Windows/Linux only:** `RUNTIME-PORT0` (Execution seam extraction) and `RHI0` (golden-image + trace harness). Neither touches behavior; both are prerequisites for everything else and have clean pass/fail oracles.
2. **Next:** `RUNTIME-PORT1` (native Execution backend, Linux-first control) and `RHI1–RHI3` (vocabulary → command list → device) can proceed in parallel — they don't share files. `PLATFORM0` (player split) can also start in parallel; its main technical risk (`present_scene_view()`) is renderer-adjacent but not RHI-abstraction-dependent, so it doesn't need to wait for RHI1–3.
3. **Once RHI3 lands:** `RHI4` (binding split) and `RUNTIME-PORT2` (macOS native backend) proceed in parallel. `PLATFORM1` (Linux parity/CI) is independent and can land anytime after PLATFORM0.
4. **Once RHI4–5 land:** `RHI6` (Android Vulkan) and `PLATFORM2` (windowing/lifecycle split, desktop-validated) proceed; then `PLATFORM4` (Android) — with the RUNTIME-PORT4-vs-stub-sim decision made explicitly before this milestone starts, not during it.
5. **Once RHI6 (Android) has proven the hard portability bugs with good tooling:** `RHI7–8` (Metal). In parallel, `RUNTIME-PORT5` (iOS Execution) and `PLATFORM6` (macOS libs/CI) can proceed since they don't depend on Metal being finished, only on the version-floor work (RHI5) and the native Execution backend (RUNTIME-PORT2/5).
6. **iOS (`PLATFORM8`)** lands after Metal-or-MoltenVK is usable (RHI7) and the frame-callback shape (PLATFORM2) exists.
7. **`RHI9` (Slang)** is independent of platform bring-up and can land whenever bandwidth allows after RHI4.
8. **Console (`RUNTIME-PORT6`, `RHI11`, `PLATFORM9`)** stays blocked on the business/legal prerequisite throughout. The only actionable console work today is `RHI11` (the backend-registry seam) — cheap, backend-agnostic, and worth doing regardless of when/whether console ships, since it costs nothing and forecloses nothing.

---

## 9. Risk register

| Risk | Wall | Severity | Mitigation |
|---|---|---|---|
| `std::function` in a naive execution seam silently breaks SYCL device codegen | 1 | Critical | Compile-time backend policy, not a vtable at node granularity (§4.3) |
| Zero render tests today; a 39,894-line refactor with no safety net | 2 | Critical | `RHI0` (golden images + `TraceCommandList`) ships *before* any pass is touched, non-negotiably |
| Cross-backend float divergence breaks existing deterministic-replay tests | 1 | High | Explicit tolerance contract: bit-determinism within a backend, tolerance-comparison across backends — renegotiate the test assumption directly, don't paper over it |
| O(N²) DAG compile stalls the native Execution backend at scale | 1 | High | Last-writer/reader-set linear construction (mirrors SushiRuntime's own tracker design) |
| Android/PLATFORM4 silently assumes RUNTIME-PORT4 is done when it isn't (or vice versa) | 1+3 | High | Make the stub-sim-vs-full-Execution decision explicit at PLATFORM4 kickoff, in writing, not by default |
| USM→native-heap silently loses render zero-copy on unified-memory platforms | 1+2 | Medium | Injectable allocator hook designed once, explicitly, before any mobile/console RHI work (§3, §4.3) |
| ~~Wall 1 and Wall 2 hazard vocabularies diverge before a shared sim+render model is designed~~ **CLOSED 2026-08-01** | 1+2 | — | The shared design landed (`unified_hazard_model.md`); RUNTIME-PORT0 mints UHM's vocabulary (UHM0), RHI1 re-keys `resource_state` on it (UHM2). No gate, no dual migration. |
| `.deps.toml` schema break against the external `ss` CLI | 3 | Medium | Additive-only schema evolution (`schema=2`, new nested tables, existing fields untouched) |
| Seam ossifies into lowest-common-denominator, blocking SushiRuntime-only features (distributed offload, `State<T>`) | 1 | Medium | `BackendCapabilities` flags, feature-query pattern already used by `IDspAccelerator::available()` |
| `component_id<T>()`'s static-local-counter registration order breaks when a new compiled library (`sushi_exec_native`) is introduced | 1 | Medium | Keep `component_id` header-inline; add a debug-build id-agreement check at `World` construction |
| macOS has neither of SushiRuntime's two known thread-affinity APIs | 1 | Low | No-op/QoS-class path in `RUNTIME-PORT2` |
| Console access blocked indefinitely on a business decision outside engineering's control | 1+2+3 | Deferred | `RUNTIME-PORT6`/`RHI11`/`PLATFORM9` explicitly unscheduled rather than estimated; only `RHI11`'s registry seam is actionable today |

---

## 10. Immediate next actions

These three can start today, in parallel, without waiting on any decision this document doesn't already make:

1. **`RUNTIME-PORT0`** — *code complete 2026-08-01; the remaining exit criterion is running the suite, not writing code.* Extract the `Execution` seam behind the existing SYCL path, zero behavior change, **minting the vocabulary defined in `unified_hazard_model.md` §4 (milestone UHM0)**. Self-contained, has a pass/fail oracle (`sandbox`), and immediately collapses SushiRuntime's engine-wide blast radius from 66 files to ~4. The runtime-side R1–R7 merge (2026-08-01) also unblocks the engine's adoption pass (delete the hand reduction, `sized_from_device` chains, region-per-island) — route that adoption *through* the new seam rather than adding direct call sites the seam must then chase.
2. **`RHI0`** — extend `render/probe/main.cpp` into a deterministic, per-pass-hashed golden-image harness, and build `TraceCommandList`. Zero production code changes; makes every subsequent RHI milestone provably a no-op or not.
3. **`PLATFORM0`**, starting with the `IWindowRenderer::present_scene_view()` gap specifically — it's the highest-risk, most architecturally-uncertain piece of the whole player-split effort, and resolving it early de-risks everything else in that milestone.

None of these three requires a decision on Metal-vs-MoltenVK, Slang-vs-SPIRV-Cross, or the Android stub-sim question — those decisions can be made later, closer to when they're actually load-bearing.

---

## 11. Explicit non-goals

Stated once here rather than repeated per section; applies engine-wide, consistent with this project's standing engineering values (SOLID without speculative generality; no designing for hypothetical requirements):

- No D3D12 backend without a specific requirement driving it (Windows already works on Vulkan).
- No generic, WebGPU-style bind-group/layout-validation system — the engine has exactly one frame-global table and one bindless heap, by design, on every backend.
- No abstracted GPU memory allocator — each backend's model (VMA, `MTLHeap`, console placement) has no useful common interface beyond `create_texture`/`create_buffer`.
- No speculative `#ifdef SUSHI_CONSOLE` hooks beyond the four already independently justified by the Metal work (access-based barriers, no push-descriptor dependency, 128-byte inline constants, backend registry).
- No runtime-polymorphic execution backend selection — one binary, one backend, chosen at compile time.
- No permanent second shader toolchain — SPIRV-Cross is a bridge with a scheduled deletion date (`RHI9`), not a parallel long-term path.
- No PS3, and no other legacy/EOL platform, without an explicit, separate decision to re-open that scope.
