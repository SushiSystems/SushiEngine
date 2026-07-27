# Changelog

All notable changes to SushiEngine are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) — versions follow [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Added
- 2026-07-27 — Added atmosphere phase B1: baked the cloudscape into two camera-centred, non-wrapping windows (near/far) instead of a periodic tile, and derived cloud genus per column from the simulated field.
- 2026-07-27 — Added atmosphere phase A: coupled the weather simulation to the sky — the cloud march samples the simulation's own horizontal grid (`Render::WeatherField`), applies coverage as a re-threshold, and samples the driving column at the camera instead of the scene anchor.
- 2026-07-26 — Added editor UI for GTAO, SSR, ray-traced shadows, particle bursts, and blend shapes — engine features that were already implemented but unreachable from the Inspector.
- 2026-07-26 — Added physics broadphase/collision, animation authoring tools, particle mesh rendering, and UI subsystem expansions.
- 2026-07-25 — Added VFX particle material features: sprite textures through the bindless heap, flipbook atlases, soft particles, and per-emitter lit shading.
- 2026-07-25 — Added audio codec/HRTF/acoustics hardening: Vorbis and streaming-compressed codecs, MagLS/anthropometric HRTF, ray-traced acoustics, and an authoring DAW panel.
- 2026-07-25 — Added a Game view no-camera placeholder with an aspect/orientation/fullscreen toolbar.
- 2026-07-25 — Added audio ECS integration (phase S6): emitter/listener/reverb-zone components and a wall-clock snapshot extract bridging the ECS to the audio engine.
- 2026-07-25 — Added a reverb system and GPU-driven per-particle punctual lighting.
- 2026-07-24 — Added animation, audio, input, and VFX systems with full editor integration, including headless S0 vertical slices for audio/input and an action-mapped input layer.
- 2026-07-24 — Added an upscaler seam, scene-interop groundwork, and shader enhancements (Phase 11).
- 2026-07-23 — Added Saturn's rings to the analytic celestial-body sky.
- 2026-07-23 — Added Phase 6 lighting: three-cascade probe-volume GI (4/8/16 m spacing) and emissive materials that light the scene through GI.
- 2026-07-23 — Added lunar/solar eclipse light-loss dimming the whole scene.
- 2026-07-23 — Added the post-processing stack (Phase 9) and GPU-driven geometry (Phase 10): per-mesh-bucket instancing, indirect draws, and single-phase GPU occlusion culling.
- 2026-07-22 — Added a Vulkan 1.4 baseline: a background PSO optimizer and a descriptor-writer seam.
- 2026-07-22 — Added Phase 3 advanced passes: GTAO, Hi-Z, clustered lighting, and SSR.
- 2026-07-22 — Added Phase 7 atmosphere: the Hillaire LUT stack and volumetric fog.
- 2026-07-21 — Added the Phase 3 foundation: the quality-tier contract, SH-9 IBL, and `ViewResources`.
- 2026-07-21 — Added the render graph architecture, PBR materials, IBL, and TAA; added astro reference frames and gravity; added shader hot-reload, a disk-backed pipeline cache with pipeline-library linking, and a bindless descriptor heap.
- 2026-07-04 — Added PBR materials, a WGS84 planet, atmosphere, clouds, and stars.
- 2026-07-04 — Added collision shapes (Sphere/Plane/Box/OBB via SAT), soft bodies, a UI framework, two-way cloth↔rigid contact coupling, and a sweep-and-prune broadphase.
- 2026-07-03 — Added selection outlining and Hierarchy reordering.
- 2026-07-03 — Added primitive objects (Box/Sphere/Cylinder/Terrain), colliders, and cloth rendering.
- 2026-07-03 — Added SushiLoop M4 (loopback network layer) and M5 (cloth grids over the XPBD solver).
- 2026-07-03 — Added a scene-less editor start — the live world now starts empty instead of seeding demo cubes — plus unsaved-scene tracking and a confirm-before-close prompt.
- 2026-07-03 — Added Hierarchy multi-select; fixed the Project panel's double-click handling.
- 2026-07-03 — Added undo/redo and New Scene.
- 2026-07-03 — Added `.sushiscene` save/open.
- 2026-07-03 — Added hierarchy parenting via drag-and-drop.
- 2026-07-03 — Added a Unity-style Project window with double-click open and a relocated project root.
- 2026-07-03 — Added rotate/scale gizmos, Local/World space, wheel-zoom, and middle-mouse pan.
- 2026-07-03 — Added a Camera component with per-display selection; the Game view no longer picks.
- 2026-07-03 — Added a Preferences window with persisted settings.
- 2026-07-03 — Added net reconciliation wired into a live client/server demo.
- 2026-07-03 — Added cloth grids wired into `RuntimeSimulation` and the Inspector.
- 2026-07-02 — Added a translate gizmo for the selected entity.
- 2026-07-02 — Added GPU id-buffer picking with a selection highlight.
- 2026-07-02 — Added an entity-aware editor over the live world, with the world as the single source of truth.
- 2026-07-02 — Added the Game view as a second viewport driven from the world camera.
- 2026-07-02 — Added a live ECS world on SushiRuntime behind an `ISimulation` seam.
- 2026-07-02 — Added a Scene viewport with a Unity-style fly camera.
- 2026-07-02 — Added Vulkan presentation for the editor.
- 2026-07-02 — Added a Unity-style editor panel set.
- 2026-06-28 — Added the `se` developer CLI and the functional test suite.
- 2026-06-26 — Added a graph-coloured Projected Gauss-Seidel physics solver.
- 2026-06-26 — Added the WP-3 archetype ECS layer.
- 2026-06-26 — Added the milestone-A headless core.

### Changed
- 2026-07-04 — Changed `Scalar` precision from a build-selectable option (`SE_SCALAR_DOUBLE`, added 2026-07-02) to always `double`, with no switch; moved rendering to camera-relative space.
- 2026-07-03 — Changed editor/engine naming to PascalCase namespaces and no-abbreviation identifiers; reorganized the editor layout.
- 2026-07-02 — Changed cloud noise generation to run on the GPU (Perlin-Worley shape, erosion detail).

### Fixed
- 2026-07-26 — Fixed remaining device-loss paths (LUTs, MRT blend, IBL descriptors).
- 2026-07-26 — Fixed functional CI by installing POCL, and fixed push-constant/image-layout validation errors.
- 2026-07-26 — Fixed CI flakiness by pinning the intel/llvm nightly to an older, more POCL-5.0-compatible build.
