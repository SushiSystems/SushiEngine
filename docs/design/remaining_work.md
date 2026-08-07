# Remaining work — the design corpus's open backlog in one view

**Status:** living — derived from the other documents in this directory, and rewritten whenever any
of them changes.

Seventeen design documents each record what their own subsystem plans, and no one of them records
what the project has left, because that answer only exists by reading seventeen roadmap sections at
once. This file is that reading. It holds no facts of its own: every row names the document that
owns the phase, and a reader who wants the detail goes there. Where a row and its owning document
disagree, the owning document wins and this file is the one that is wrong.

Rows are sorted by document, not by priority. Priority is the project owner's and is proposed
separately, so this file does not go stale the moment the owner reorders it.

The State column takes one of four values and nothing else, so a row maps back to the audit term
behind it:

- `open` — the owning document is accurate and the work is not built. Most of the backlog.
- `blocked` — open, and something outside the phase has to land before it can start.
- `withdrawn` — the document claimed the work as delivered, the tree did not carry it, and the
  claim has been withdrawn. The work itself is still owed.
- `unmeasured` — the code exists and is reachable from a build target, and a numeric or visual
  acceptance criterion has never been checked because no machine here has the hardware. These
  close with a measurement, not with a commit, which is why they are a separate table.

A phase the audit found claimed and true appears nowhere below, so absence of a coded phase from
this file means that phase is done. Coverage therefore matters more here than concision: a row
that goes missing reads as finished.

The tables carry phases, because a phase is what a roadmap section numbers and what the audit
issued a verdict on. Scope a document defers without giving it a phase code stays in that
document — [model_import.md](model_import.md) §12's asset inspector, rigged models inside an
imported hierarchy and the glTF feature-coverage audit are the largest example, and §12 is where
they are tracked.

## Open

| Phase | Document | State | What it still needs |
| --- | --- | --- | --- |
| §12.5 device evaluator parity | [animation_system.md](animation_system.md) | open | A test that runs the device batch evaluator's SYCL kernel and asserts the device result matches the host's. No such test exists, and writing it is the whole of the work: the kernel runs on this machine's CPU and OpenCL backends, so the run is not waiting on a GPU. |
| D | [atmosphere_system.md](atmosphere_system.md) | blocked | Terrain and surface coupling: orographic lift, rain shadow, föhn, valley fog, sea and lake breeze, terrain-driven turbulence, land-cover convective initiation. §15 records a blocker that needs restating before the phase can be scoped. |
| E | [atmosphere_system.md](atmosphere_system.md) | open | `AtmosphereProfile` and `AtmosphereDiagnostics` replacing `WeatherColumn`, the query mirror and deterministic summary, the five existing weather headers retargeted onto them, a skew-T readout, and GRIB ingest beside the retained METAR path. Roughly half the surface area exists and needs retargeting rather than writing. |
| F | [atmosphere_system.md](atmosphere_system.md) | open | A microscale large-eddy-simulation domain of roughly 20 kilometres at 100 metres. The document says "possibly never"; unscoped until someone asks for it. |
| CV5 + CV9 | [atmosphere_system.md](atmosphere_system.md) | open | The generation model. One 2,400-metre noise scale decides where every cloud is, so there is no mesoscale organisation, and water is per column, so every cloud in a column carries the same density. The two meet in the same code and land together. §11's handover names this the priority. |
| S8 codecs | [audio_system.md](audio_system.md) | open | Opus, Vorbis and SOFA are demo-verified only. Needs a test that exercises all three; none exists today. |
| RHI1–RHI11 | [cross_platform_engineering_plan.md](cross_platform_engineering_plan.md) | open | The neutral `Rhi::` vocabulary through Metal parity and the Slang migration. RHI1 is next per §10 and is unblocked: RHI0's golden-image harness is closed. |
| RUNTIME-PORT1–8 | [cross_platform_engineering_plan.md](cross_platform_engineering_plan.md) | open | The native execution backend beyond its current Windows and SushiRuntime seam, and every non-Windows bring-up. RUNTIME-PORT1, the native backend on Linux first, is next. |
| PLATFORM1–9 | [cross_platform_engineering_plan.md](cross_platform_engineering_plan.md) | open | Linux parity, then mobile and console bring-up. PLATFORM0 gave all of them the editor and runtime split they depend on. |
| Deferred items | [editor_feature_sync_gaps.md](editor_feature_sync_gaps.md) | open | Six deferred items transferred to [editor_ux_overhaul.md](editor_ux_overhaul.md) §2.4, and four are discharged: the two Material Inspector checkboxes and the GPU Culling statistics checkbox were removed, and the GPU Culling freeze-frustum control is wired at `engine/presentation/render/shaders/cull.comp`. Two are not. VFX `SimulationDomain` still has no authoring widget — `SortMode` and `BeamModule` gained Sort and Alignment combo entries in `applications/editor/source/vfx/particle_panel.cpp` and it did not. Audio acoustic geometry has no editor surface at all: nothing under `applications/editor/source/` names it. |
| §6.1 | [editor_ux_overhaul.md](editor_ux_overhaul.md) | open | The clean-profile first-run check. Needs a reset-profile command-line flag, or an automated run of the editor against an empty configuration directory, or that run wired into continuous integration, which builds the editor target today and never executes it. |
| §9 | [model_import.md](model_import.md) | open | The naming correction: `has_character_extension` to `is_model_extension`, `open_character_in_preview` to `open_model_asset`, and a branch on the glTF scene's skin count so an unrigged file stops opening in the animated-character preview. Drafted already in the document; small and scoped. |
| P8 | [physics_system.md](physics_system.md) | open | Five pieces in §16.42's order: a barrier-reduction primitive blocked on a SushiRuntime API recorded as ask R9; per-island substepping, additionally blocked on §16.43's design question; sleeping-parking for beams and elements, over the island-connectivity bug beneath it; structure-of-arrays state columns, roughly 60 files; and device-resident broadphase, narrowphase and contact detection, sequenced last. |
| P9 | [physics_system.md](physics_system.md) | open | Kinematic bodies, a character controller, `IPhysicsEventSink` and its fan-out to gameplay, audio and VFX, rollback snapshot integration, and the networking validation harness with its 10,000-tick byte-equality test. |
| §8 | [prefab_system.md](prefab_system.md) | open | The preserve-on-missing mechanism exists; none of the reporting built on it does. Thread the returned unreadable-path list from the load call site through to the editor log, add an unlinked flag to `PrefabInstanceParameters`, state an entity count before a rebuild runs, and report a flattened nested instance. Editor-only work. |
| P2 | [prefab_system.md](prefab_system.md) | open | Override resolution. `prefab_entity_id` is written by P1 and has no consumer anywhere; giving it one is the next design-and-build task in this line. |
| P3 | [prefab_system.md](prefab_system.md) | open | Nested prefabs and a prefab edit mode. Designed only as far as their boundary. |
| P4 | [prefab_system.md](prefab_system.md) | open | A runtime instantiation API. Prefab instantiation is editor-only today; nothing in the deterministic loop references it. |
| 4 | [render_pipeline_refactor.md](render_pipeline_refactor.md) | open | Area and IES lights, projected normal-map decals, shadow caching, screen-coverage quadtree tiles, and adaptive per-light percentage-closer soft shadows. The clustered core is shipped; these are the tier-scalable remainder. |
| 5 | [render_pipeline_refactor.md](render_pipeline_refactor.md) | open | Local reflection probes (§5.4), deferred pending the editor's scene-capture visual loop. |
| 6 | [render_pipeline_refactor.md](render_pipeline_refactor.md) | open | Multiple cascades, emissive injection, toroidal amortization, and the Tier B ray-query tracer, the last of which needs ray-tracing hardware. |
| 7 | [render_pipeline_refactor.md](render_pipeline_refactor.md) | open | Cascaded-shadow-map god-ray shafts, punctual-light fog, per-tier lookup-table resolutions, and temporal amortization. |
| 9 | [render_pipeline_refactor.md](render_pipeline_refactor.md) | open | HDR10 and scRGB output. No swapchain colour-space code exists; verifying it afterwards also needs an HDR display. |
| 10 | [render_pipeline_refactor.md](render_pipeline_refactor.md) | open | Sparse virtual texturing, deferred with a stated rationale and absent from the tree. |
| 11 | [render_pipeline_refactor.md](render_pipeline_refactor.md) | blocked | The runtime-side interop import, blocked on a SushiRuntime API that does not exist. The same blocker is why `create_interop_buffer` has no caller in the restructure's phase-3 inventory. |
| 12 | [render_pipeline_refactor.md](render_pipeline_refactor.md) | blocked | 12.1, 12.2 and 12.3 Tier B all need `VK_KHR_ray_query`, which the Pascal development device does not have. 12.2 additionally needs the NRD software development kit and the licensing decision that goes with it. |
| 4 | [repository_restructure.md](repository_restructure.md) | open | The document's own §10 records the structural half done and the citation repair landed behind `tools/documentation/check_design_citations.py`. What phase 4 still owes is the four defects in "Backlog this audit created" below, which that repair measured and did not close. |
| P2c | [solar_system_overhaul.md](solar_system_overhaul.md) | open | §20.1's punch list. The camera starts inside the terrain shell because nothing resolves the observer's altitude against the height field before the scene origin anchors to the reference ellipsoid, and the terrain pass then draws the shell's interior in every direction. Implementation against a written diagnosis, not new design. |
| P3–P11 | [solar_system_overhaul.md](solar_system_overhaul.md) | open | Residency streaming, collision, the editable layer stack, Earth, material synthesis, detail synthesis, the `Mesh` LOD rung, the atmosphere seam and the GPU quadtree tier. Sized and sequenced already; P3 to P5 come first, because after P5 there is a walkable, editable planetary surface. |
| UHM1–UHM5 | [unified_hazard_model.md](unified_hazard_model.md) | blocked | The shared hazard core beyond UHM0's vocabulary headers, and the whole `Handoff` boundary type across its T0 to T3 realizations. UHM1 needs RUNTIME-PORT1 to have something to adopt against; UHM3 waits on SushiRuntime's own BB-1a, outside this repository's control. |

## Withdrawn claims

The document claimed each of these as delivered, the tree did not carry it, and the claim has been
withdrawn from the document. The work is still owed; only the claim is gone.

| Phase | Document | State | What it still needs |
| --- | --- | --- | --- |
| §5 rule 11 / §13 | [model_import.md](model_import.md) | withdrawn | Collider generation on import. The setting flows into the plan as a per-entity flag and `engine/world/model_import/source/prefab_output.cpp` never reads it, so no collider is ever created. Needs a case in that switch calling the physics cooking path, and a test proving a model imported with the setting on carries a collider. |
| M1 | [SUSHILOOP.md](SUSHILOOP.md) | withdrawn | Continuous integration on both platforms. `.github/workflows/ci.yml` defines Linux jobs only. Needs a Windows job, or the claim stays withdrawn and Windows stays a local build. |
| M3 | [SUSHILOOP.md](SUSHILOOP.md) | withdrawn | Delta snapshot recording. `engine/world/loop/include/SushiEngine/loop/rollback.hpp` copies every live chunk in full every tick, as its own header comment says. Needs the per-write dirty tracking that comment names as the follow-on. |
| M4 | [SUSHILOOP.md](SUSHILOOP.md) | withdrawn | Real sockets. `engine/world/loop/include/SushiEngine/loop/net.hpp` carries a loopback transport and states it has no sockets, threads or serialization. Needs a real transport behind the same seam the loopback already implements. |

## Unmeasured — built, waiting on hardware

Every row below is code that exists and is reachable from a build target, under an acceptance
criterion nothing has checked because the machine the engine is developed on has no suitable GPU,
no HDR display and no ray-tracing device. These close with a run and a number, not with a commit.
Mixing them into the open backlog above would misprice both.

| Phase | Document | State | What it still needs |
| --- | --- | --- | --- |
| §12.1 evaluator bridge | [animation_system.md](animation_system.md) | unmeasured | A GPU display session: load a rigged character in the editor and confirm the skinned result draws. The wiring is built and the document does not claim more. |
| §12.1 GPU morph blending | [animation_system.md](animation_system.md) | unmeasured | The same session, looking at a morphed character. Descriptor bindings and push constants are in place. |
| §12.3 launch check | [animation_system.md](animation_system.md) | unmeasured | An interactive editor launch confirming the device-loss fix holds. |
| §12.4 dual-quaternion wiring | [animation_system.md](animation_system.md) | unmeasured | The same session, with dual-quaternion skinning selected. The algorithm and its tests are separate and pass. |
| B3e | [atmosphere_system.md](atmosphere_system.md) | unmeasured | The third of B3e's three acceptance clauses. Two pass; the third needs a GPU run. |
| §11 handover | [atmosphere_system.md](atmosphere_system.md) | unmeasured | CV3, PL1 and the rest of the handover's fixed table, written 2026-08-02 and unverified by eye. They need a screenshot, not a code change; the handover's own first rule is to ask for one before forming a hypothesis. |
| S10 SYCL execution | [audio_system.md](audio_system.md) | unmeasured | The SYCL accelerator compiles and has never executed on a device. Needs a GPU session. |
| S0 device seam | [audio_system.md](audio_system.md) | unmeasured | The SDL and miniaudio backends are real device-callback code. Audible correctness needs real audio hardware and someone listening. |
| P1 | [physics_system.md](physics_system.md) | unmeasured | The §13.1 clause "contact cost drops measurably against the P0 baseline". A GPU-timed number. |
| P2 | [physics_system.md](physics_system.md) | unmeasured | The §13.1 clauses for 1,000 mixed-shape bodies and 10,000 mostly-sleeping bodies. GPU-timed numbers. |
| P6 | [physics_system.md](physics_system.md) | unmeasured | The 20,250-tetrahedron, 3-millisecond §13.1 target. 29.4 milliseconds is on record from the CPU backend because SushiRuntime found no device. One run of `samples/physics/soft_body_budget.cpp` on a machine with a SYCL-visible GPU closes it. |
| P7 | [physics_system.md](physics_system.md) | unmeasured | The §13.1 vehicle target, deterministic under replay. A GPU-timed number. |
| 0–12 visual claims | [render_pipeline_refactor.md](render_pipeline_refactor.md) | unmeasured | Every "shipped in core form" in this document was verified as code that exists and is wired, never as an image that looks right. Closing it means a GPU and an eye, phase by phase. |
| M2 | [SUSHILOOP.md](SUSHILOOP.md) | unmeasured | The XPBD solver builds and runs an execution graph; whether that graph lands on a GPU device depends on the selected backend, which needs GPU hardware to confirm. |
| §13 | [vfx_particle_system.md](vfx_particle_system.md) | unmeasured | VFX2c, VFX2d and VFX7 are built and await the GPU visual check. §13 names the exact scene to look at. This is the document's only remaining item; §12 carries none. |

## Backlog this audit created

The corpus-wide citation audit that produced this file repaired 1,074 citation sites and left four
classes of defect behind it, all measured, none fixed. They belong to
[repository_restructure.md](repository_restructure.md)'s phase 4, which owns the corpus's citation
standard and records them in its §10, and they are listed here at row granularity because an
unmeasured population that nobody wrote down is exactly the failure this audit exists to correct.
This table is scoped to defects rather than phases, so a row here carries no phase code.

| Item | Size | What it still needs |
| --- | --- | --- |
| Subject-and-file agreement | 1,064 repaired sites, unswept | A unique filename match proves a path resolves, not that it is the file the sentence means. Two wrong-file citations were found by hand, both in one paragraph, and one of the two arrived through the unique-match route rather than the ambiguous one. The population is unmeasured, not clean, and needs a sweep that reads each sentence against the file it now points at. |
| Unverified line numbers | 33 of 34 citations | Citations that already resolved were exempted from every check, so their line numbers were never verified. The one checked by hand was wrong: `docs/design/static_mesh_authoring.md` cited `applications/editor/source/main.cpp:489-501` for a copy that runs 484 to 499. It is corrected; the other 33 are unchecked. Needs a pass that checks a line locator against the file it names. |
| Bare continuation references | 58 references | A bare `:466-467` sitting beside an anchor whose line suffix the repair pass stripped. The checker matches a backticked path and cannot see these at all, so they are neither verified nor verifiable today. Needs either the suffixes restored on their anchors or the checker taught to read a continuation. |
| Stale de-abbreviated identifiers | 9 sites corrected, the sweep still owed | All nine `QualityParams` sites now name the real `QualityParameters`, and four of them named fields the type no longer carries at all — `atmosphere_nest`, `max_particles` and the three animation budgets, deleted on 2026-07-29 as resolved values no pass read — so those four sentences were corrected rather than renamed. The two changelog occurrences are historical and stay. The row's scope is not discharged: nothing has swept the corpus for the other identifiers the de-abbreviation pass renamed. No check sees this — the citation checker matches a path, and an identifier is not a path. |
