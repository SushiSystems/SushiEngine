# Design documents

This directory holds SushiEngine's engineering plans and the audits behind them. It is a record of
intent rather than a description of the tree, which is why a document here may describe work that
does not exist: it says why a subsystem is shaped the way it is and what is planned for it, and that
plan outlives any one commit. Everything else under `docs/` describes what is built today. Each
document opens with a status line, and the roadmap section that line names is the single place its
per-phase progress is recorded.

One file here is neither a plan nor an audit. [remaining_work.md](remaining_work.md) is a derived
view: it collects what every roadmap section above has left, holds no facts of its own, and changes
whenever one of those sections does. Where it and an owning document disagree, the owning document
is right.

| Document | Status | Covers |
| --- | --- | --- |
| [animation_system.md](animation_system.md) | shipped | Skeletal animation: clip assets and compression, an Animator controller with layered state machines and blend trees, the IK and pose-modifier stack, and GPU skinning. |
| [atmosphere_system.md](atmosphere_system.md) | in progress | GPU meteorology and a planet-scale cloudscape: the global core, the regional nest, the summary the deterministic world consumes, and phases A to F. |
| [audio_system.md](audio_system.md) | shipped | A from-scratch AAA game-audio runtime: the real-time DSP core, ambisonic and binaural spatialization, propagation, occlusion, reverb, voice management, and the ECS integration. |
| [cross_platform_engineering_plan.md](cross_platform_engineering_plan.md) | in progress | The three structural walls between today's Windows engine and Windows, Linux, Android, macOS, iOS, PS5 and PS4, and the milestones that take each one down. |
| [editor_feature_sync_gaps.md](editor_feature_sync_gaps.md) | superseded | An audit of editor controls against the engine features they claim to expose: what was reconnected in that pass, and what was deferred. |
| [editor_ux_overhaul.md](editor_ux_overhaul.md) | shipped | The rebuild of the editor into a Unity-class layout: docking, per-domain quality tiers, a wire-or-remove verdict for every unbound control, and visual identity. |
| [entity_lifecycle_system.md](entity_lifecycle_system.md) | designed | Real enabled/disabled state gating physics, audio and render (not just render); runtime instantiate/destroy on the deferred ECS discipline; native OnEnable/OnDisable/OnSpawn/OnDestroy hooks; inter-object messaging. |
| [model_import.md](model_import.md) | in progress | A glTF file as an entity hierarchy: per-asset import settings in a `.meta` sidecar, the node graph mirrored into parented entities with their materials, lights and cameras, and the reimport link back to the asset. |
| [physics_system.md](physics_system.md) | in progress | One unified XPBD solver carrying rigid bodies, articulations, cloth and soft bodies, the offline cooking pipeline, and the road to deformable vehicles. |
| [prefab_system.md](prefab_system.md) | in progress | An authored entity subtree as a reusable asset: the `.sushiprefab` file, the instance component, refresh on scene load, and the link that makes an imported model a prefab. |
| [project_selection.md](project_selection.md) | shipped | New/Load Project in the editor's File menu: picking a project directory at runtime instead of one fixed at startup, reusing the already cross-platform preferences store. |
| [remaining_work.md](remaining_work.md) | living | Every phase the corpus audit found incomplete, in one table: open work, claims withdrawn from a document, and phases whose acceptance number is unmeasured for want of hardware. |
| [render_pipeline_refactor.md](render_pipeline_refactor.md) | in progress | The phased path to a performance-first renderer: clustered Forward+, global illumination, the atmosphere LUT stack, post-processing, GPU-driven geometry, and delivery. |
| [repository_restructure.md](repository_restructure.md) | in progress | Moving every engine module under `engine/` in named tiers with per-module include roots, so a dependency the tier order forbids fails at configure time. |
| [solar_system_overhaul.md](solar_system_overhaul.md) | in progress | Real planetary terrain from a metre to orbit: baked cube-sphere height tiles, run-time-editable layer stacks, CDLOD geometry, and the authoritative height path physics reads. |
| [static_mesh_authoring.md](static_mesh_authoring.md) | shipped | Placing an imported glTF as a plain visual prop in the scene: an "Imported" mesh kind on the existing Renderer component, wired to render and import machinery already built. |
| [SUSHILOOP.md](SUSHILOOP.md) | shipped, three clauses narrower | The deterministic, network-ready fixed-step game loop: ECS systems, GPU XPBD in double precision, whole-chunk snapshots and rollback, and a server-authoritative loopback transport with client prediction and reconciliation. Continuous integration builds Linux only, rollback records whole chunks rather than deltas, and there are no sockets. |
| [unified_hazard_model.md](unified_hazard_model.md) | designed | One access algebra, one tracker semantic and one boundary contract shared by the simulation and render schedule domains, joined at an epoch handoff. |
| [vfx_particle_system.md](vfx_particle_system.md) | in progress | One authored effect asset behind two backends — a GPU cosmetic simulator and a CPU deterministic one — plus the render-graph integration and the editor authoring surface. |
