# Design documents

This directory holds SushiEngine's engineering plans and the audits behind them. It is a record of
intent rather than a description of the tree, which is why a document here may describe work that
does not exist: it says why a subsystem is shaped the way it is and what is planned for it, and that
plan outlives any one commit. Everything else under `docs/` describes what is built today. Each
document opens with a status line, and the roadmap section that line names is the single place its
per-phase progress is recorded.

| Document | Status | Covers |
| --- | --- | --- |
| [animation_system.md](animation_system.md) | shipped | Skeletal animation: clip assets and compression, an Animator controller with layered state machines and blend trees, the IK and pose-modifier stack, and GPU skinning. |
| [atmosphere_system.md](atmosphere_system.md) | in progress | GPU meteorology and a planet-scale cloudscape: the global core, the regional nest, the summary the deterministic world consumes, and phases A to F. |
| [audio_system.md](audio_system.md) | shipped | A from-scratch AAA game-audio runtime: the real-time DSP core, ambisonic and binaural spatialization, propagation, occlusion, reverb, voice management, and the ECS integration. |
| [cross_platform_engineering_plan.md](cross_platform_engineering_plan.md) | designed | The three structural walls between today's Windows engine and Windows, Linux, Android, macOS, iOS, PS5 and PS4, and the milestones that take each one down. |
| [editor_feature_sync_gaps.md](editor_feature_sync_gaps.md) | superseded | An audit of editor controls against the engine features they claim to expose: what was reconnected in that pass, and what was deferred. |
| [editor_ux_overhaul.md](editor_ux_overhaul.md) | shipped | The rebuild of the editor into a Unity-class layout: docking, per-domain quality tiers, a wire-or-remove verdict for every unbound control, and visual identity. |
| [model_import.md](model_import.md) | designed | A glTF file as an entity hierarchy: per-asset import settings in a `.meta` sidecar, the node graph mirrored into parented entities with their materials, lights and cameras, and the reimport link back to the asset. |
| [physics_system.md](physics_system.md) | in progress | One unified XPBD solver carrying rigid bodies, articulations, cloth and soft bodies, the offline cooking pipeline, and the road to deformable vehicles. |
| [project_selection.md](project_selection.md) | shipped | New/Load Project in the editor's File menu: picking a project directory at runtime instead of one fixed at startup, reusing the already cross-platform preferences store. |
| [render_pipeline_refactor.md](render_pipeline_refactor.md) | in progress | The phased path to a performance-first renderer: clustered Forward+, global illumination, the atmosphere LUT stack, post-processing, GPU-driven geometry, and delivery. |
| [repository_restructure.md](repository_restructure.md) | in progress | Moving every engine module under `engine/` in named tiers with per-module include roots, so a dependency the tier order forbids fails at configure time. |
| [solar_system_overhaul.md](solar_system_overhaul.md) | designed | Real planetary terrain from a metre to orbit: baked cube-sphere height tiles, run-time-editable layer stacks, CDLOD geometry, and the authoritative height path physics reads. |
| [static_mesh_authoring.md](static_mesh_authoring.md) | shipped | Placing an imported glTF as a plain visual prop in the scene: an "Imported" mesh kind on the existing Renderer component, wired to render and import machinery already built. |
| [SUSHILOOP.md](SUSHILOOP.md) | shipped | The deterministic, network-ready fixed-step game loop: ECS systems, GPU XPBD in double precision, delta snapshots and rollback, and server-authoritative networking. |
| [unified_hazard_model.md](unified_hazard_model.md) | designed | One access algebra, one tracker semantic and one boundary contract shared by the simulation and render schedule domains, joined at an epoch handoff. |
| [vfx_particle_system.md](vfx_particle_system.md) | in progress | One authored effect asset behind two backends — a GPU cosmetic simulator and a CPU deterministic one — plus the render-graph integration and the editor authoring surface. |
