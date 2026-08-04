# SushiEngine architecture

This directory explains how SushiEngine is put together: the relationship to SushiRuntime, the
tier layering the modules are arranged into, and the seams a non-trivial change will touch. It is
one file per subject, grouped the way `cmake/EngineLayers.cmake` groups the code —
`foundation → domain → asset → presentation → world` — so the file you need is the tier the change
lands in. Read [`overview.md`](overview.md) before anything else; read the rest when a change
crosses into them.

| File | What it covers |
| --- | --- |
| [`overview.md`](overview.md) | The head/battery split with SushiRuntime, and the layer table every module sits in. |
| [`foundation.md`](foundation.md) | The ECS, archetype-chunk storage and the system graph, and the value-type seam all precision rests on. |
| [`domain-physics.md`](domain-physics.md) | The graph-coloured PGS and XPBD solvers, cloth and soft bodies, colliders and contacts, and their editor authoring. |
| [`domain-astro.md`](domain-astro.md) | The ephemeris, celestial lighting, gravity field, reference frames, and frame-local authoring. |
| [`domain-ui.md`](domain-ui.md) | The retained ECS canvas, its anchor solver and façade, and the render pass that composites it. |
| [`domain-animation.md`](domain-animation.md) | Skeletons, clips and controllers, the deterministic animator tick, blend trees, layers, IK, morphs and retargeting. |
| [`domain-input.md`](domain-input.md) | Device-abstracted actions, bindings as data, the SDL backend, rebinding, touch, and the tick boundary. |
| [`domain-audio.md`](domain-audio.md) | The two-plane audio engine: DSP core, mixer and voices, propagation, spatialization, reverb, the ECS bridge, and the bank pipeline. |
| [`domain-vfx.md`](domain-vfx.md) | The particle authoring model, its deterministic CPU and cosmetic GPU backends, every render alignment, and the particle material. |
| [`domain-atmosphere.md`](domain-atmosphere.md) | Weather providers and the cloudscape they compile to, the spatial weather field and window, and the GPU regional nest. |
| [`domain-terrain.md`](domain-terrain.md) | The cube-sphere quadtree, the editable layer stack, the height-source and pack formats, and the terrain draw. |
| [`presentation-render.md`](presentation-render.md) | The RHI and scene-view seams, the frame graph, materials and IBL, the temporal core, shadows, and the lighting and sky passes. |
| [`world.md`](world.md) | SushiLoop's snapshot/rollback buffer, the loopback network reconciliation, and the `Loop::App` host loop. |
| [`tooling.md`](tooling.md) | The functional test suite's shape and what it pins, and the `se` developer CLI. |
| [`roadmap.md`](roadmap.md) | The milestones — what has landed, what is in progress, and the editor and player host shells. |
