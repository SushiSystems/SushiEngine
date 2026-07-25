# Editor/feature sync gaps — backlog (2026-07-26)

An audit of the editor UI against the engine features it's supposed to expose turned up several
disconnects. This doc tracks what was fixed in the same pass and what's deferred for later, so the
deferred items don't get rediscovered from scratch.

## Fixed this pass

- **GTAO** (`RenderSettings::gtao`) had no editor UI despite being fully wired through
  `gtao_pass.cpp` and round-tripped in `preferences.cpp`. Added a section to the Rendering panel
  (`editor/ui/editor_panels.cpp`).
- **SSR** (`RenderSettings::ssr`) — same gap, same fix. Note the doc comment on `SsrSettings`: only
  the hi-Z pyramid build is live today, the reflection trace itself lands in a later increment.
- **Ray-traced sun shadows** (`ShadowSettings::ray_traced`) and **secondary directional shadow
  casters** (`max_directional_shadow_casters`) were consumed by `ray_traced_shadow_pass.cpp` /
  `quality.cpp` but had no toggle in the Shadows section. Added both.
- **VFX particle bursts** (`Vfx::ParticleBurst`) were drawn on the effect timeline but had no
  authoring UI — an author could see bursts that already existed but not add/remove/edit one. Added
  a burst list (time/count + Add/Remove) under Emission in `draw_particle_system_component`.
- **Blend shapes / morph targets**: `AnimatedMeshPreview::set_morph_weights` was a dead seam — glTF
  morph-target import already existed in `gltf_importer.cpp` (`primitive.targets[]` →
  `MeshRegistry::set_morph_targets`), but nothing queried the target count or exposed sliders.
  Added `IAssetLibrary::morph_target_count(MeshId)`, had `AnimatedMeshPreview::load_gltf` size
  `morph_weights_` from it, and added a "Blend Shapes" slider section to the Animator panel
  (`editor/animation/animator_preview_panel.cpp`). Still manual per-target weights, not clip-driven
  (design `animation_system.md` §12.1/§12.2 — glTF `WEIGHTS` animation-channel import doesn't exist
  yet; that's a separate, larger piece of work).

## Deferred — no-op controls (UI exists, nothing reads it)

- **Material Inspector**: "Receive Shadows" and "GPU Instancing" checkboxes
  (`editor/ui/material_inspector.cpp`) toggle `Material::receive_shadows` / `gpu_instancing`
  (`include/SushiEngine/render/material.hpp`), but no render pass reads either field — every
  material appears to support per-object shadow-receiving and instancing control, and toggling
  either does nothing. Fixing this for real means either:
  - wiring `receive_shadows` into whichever shadow-sampling shader/pass sees this material
    (`pbr.frag` and friends), and `gpu_instancing` into an actual instancing batcher, or
  - removing the checkboxes until those exist, so the UI stops promising something it can't do.
  Deferred because both are shader/pass-level engine changes, not editor-only, and need a GPU to
  verify visually.
- **GPU Culling panel**: "Freeze frustum (debug)" and "Show statistics"
  (`editor/ui/editor_panels.cpp`, `draw_gpu_culling_panel`) toggle
  `GpuCullingSettings::freeze` / `show_statistics`, which `cull_pass.cpp` never reads. The struct's
  own doc comment already says these are "reserved for the editor's cull debug view" — i.e. this
  was always meant to be finished later, not a regression. Needs: a frozen-frustum debug mode in
  `cull_pass.cpp`, and a readback path for per-frame cull counts (the "Show statistics" branch
  currently just points at the Profiler HUD instead of producing anything itself).

## Deferred — engine feature has no editor UI at all

- **Audio acoustic geometry**: `include/SushiEngine/audio/acoustic_geometry.hpp`,
  `acoustic_raytracer.hpp`, `occlusion.hpp`, `portals.hpp`, `propagation.hpp`,
  `early_reflections.hpp`, `convolution_reverb.hpp` are all real, but `editor/audio/audio_panels.cpp`
  / `audio_authoring_panel.cpp` only expose the mixer, per-entity emitter/listener/reverb-zone
  params (hand-set "Room/Hall/Cave/Generic" presets), and a simple container-graph tool. There's no
  way to mark level geometry as occluding, place a portal, or preview a ray-traced early-reflection
  response from the editor — reverb is entirely manual rather than derived from the scene. This may
  be intentional (baked/automatic at runtime) rather than a bug; needs a scoping decision before any
  UI work starts, and it's the largest single gap found (a new authoring surface, not a
  slider-in-an-existing-panel fix).
- **VFX `SortMode`, `SimulationDomain`, `BeamModule`** (`include/SushiEngine/vfx/modules.hpp`) have
  no widgets in `draw_particle_system_component` — only Spawn/Shape/Init/Gravity/Drag/
  Turbulence/Collision/ForceFields/SizeOverLife/ColorOverLife/Render are exposed via the "Graph"
  node list. Beams in particular are a distinct render mode with no authoring path at all. Lower
  priority than the audio gap — these are more specialized modules — but still real gaps against
  what `modules.hpp` defines.
