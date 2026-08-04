# VFX particles

This file covers the particle system: the authoring model both backends compile from, the
deterministic CPU backend and the cosmetic GPU path, every render alignment and material feature
layered on them, and the component-shaped authoring surface in the editor.

## 1. VFX particle system (authoring model, dual backends, GPU render — phase VFX1)

Design: `docs/design/vfx_particle_system.md`. One authored effect asset feeds **two** simulation
backends behind one seam — a GPU-cosmetic path (millions of particles, render-side, *outside* the
deterministic sim island, like skinning/audio) and a CPU-deterministic path (bounded,
byte-reproducible, rollback-safe, like `AnimatorInstance`). Both consume the same compiled POD.

**Authoring model (`engine/domain/vfx/include/SushiEngine/vfx/`, header-only, C++17, depends only
on `engine/foundation/core/include/SushiEngine/core/types.hpp`).** An emitter is a stack of
modules across four stages (spawn / shape / init / update) plus a render module; each module is
its own trivially-copyable descriptor struct (the Open/Closed seam — a new behaviour adds a
descriptor, a compiler handler, and a shader/integrator branch, touching no existing module).
`AnimationCurve` and `ColorGradient` are keyframed authoring types bakeable to fixed-width LUTs.
`EmitterCompiler` flattens a `ParticleEffect` (a list of `EmitterDescriptor`) into a
`CompiledEffect`: an array of POD `CompiledEmitter` records plus two baked LUT atlases — the
single artifact both backends and the GPU consume, the particle equivalent of resolving authored
`RenderSettings` into a POD `QualityParameters`. `EffectDatabase` is the AssetId registry (lazy
compilation), mirroring `AnimationDatabase`. `GPUParticle` is the shared 80-byte, five-`vec4`
std430 record used by the CPU backend, the GPU pools, and the shaders.

**Deterministic backend
(`engine/domain/vfx/include/SushiEngine/vfx/deterministic_backend.hpp` +
`Simulation::ParticleEmitter`).** A fixed-pool integrator run as an ECS system inside the fixed
step. Its whole per-emitter state is a `DeterministicEmitterState` — a capped `GPUParticle` pool,
a count, a `Pcg32`, and a few scalars, pointer-free — so a tick is byte-snapshottable and a
rolled-back-then-replayed tick reproduces it exactly (`Integration_ParticleDeterminism`,
`tests/integration/test_particle_determinism.cpp`). Shapes, forces (gravity/drag/curl-noise
turbulence), and the size/colour-over-life LUTs all run here too, sampling the same baked LUTs
the GPU does.

**GPU render path.** `Render::Scene::ParticleSystem`
(`engine/presentation/render/source/scene/particle_system.*`) owns the shared, persistent,
device-local particle pool (zero-cleared once), the per-slot host-visible emitter table
(`GPUEmitter`, a std430 mirror of the compute-visible `CompiledEmitter` subset with the emitter's
world transform and ring cursor baked in), and the uploaded LUT atlases.

`ParticleSimPass` (`engine/presentation/render/source/passes/particle_sim_pass.*`, a compute
`IRenderPass`, after `skinning_pass_`) sweeps the pool: a simulate dispatch advances/ages/retires
and appends survivors to a compacted draw list, then a per-emitter emit dispatch allocates ring
slots and initialises new particles — both atomically build a `VkDrawIndirectCommand`.
`ParticlePass` (`engine/presentation/render/source/passes/particle_pass.*`, a graphics
`IRenderPass`, between `ssr_pass_` and `taa_pass_`, writing `scene_final`) draws vertex-lessly:
six vertices per alive particle, expanded into a camera-facing billboard pulled from the draw
list, sampling the scene depth (never attaching it) to discard fragments behind geometry,
additively blended.

The compacted draw list and indirect args are graph transients (the graph derives the
compute→draw barriers); the pool is system-owned. `ISceneView::render` gained a
`ParticleEmitterView` extract channel (opaque `CompiledEmitter` bytes + LUT pointers +
host-computed spawn count), the same shape as `SkinnedInstance`. Additive-only for the slice;
alpha depth-sorting, lit particles, ribbons, mesh particles, and GPU collision are phases
VFX2–VFX5.

**Enabling seams.** `Resources::GraphicsPipelineDescription`
(`engine/presentation/render/source/resources/`) gained a `ColorBlend` member folded into the
fragment-output pipeline-library key (defaults reproduce opaque, so transparent draws are now
expressible without disturbing existing passes). `QualityParameters` gained `gpu_particles` /
`max_particles` / `particle_sim_substeps`, scaled per tier (Low drops cosmetic particles). Build:
the four shaders (`particle_emit.comp`, `particle_simulate.comp`, `particle.vert`,
`particle.frag`, sharing `particle_common.glsl`, all under
`engine/presentation/render/shaders/`) go through `sushiengine_compile_shader` + the shader
catalogue; the two systems and two passes into `sushiengine_render`.

### 1.1. Deterministic emitter entities + alpha particles (Bağla + VFX2a)

**Deterministic emitter entities.** A gameplay entity carries a particle emitter through the
sim's `IWorldEditor` (the emitter quartet `create_particle_emitter` / `has_particle_emitter` /
`particle_emitter_parameters` / `set_particle_emitter_parameters` / `set_has_particle_emitter` +
`particle_effect_count`/`particle_effect_name`). Like cloth, this is host bookkeeping — no ECS
migration: `RuntimeSimulation::Record` gains the fixed `VFX::DeterministicEmitterState pool`
(~80 KB, off the ECS chunk), plus the effect handle and play head.

`step_particle_emitters()` runs inside `step_once()` (after the schedule, before extract),
advancing every playing pool one fixed step via `CPUDeterministicBackend::step`; `extract()`
emits one `RenderScene::particle_billboard` per live particle. A built-in effect library
(Fire/Sparks/Smoke, Deterministic domain) lives on the sim's `EffectDatabase`. The renderer draws
these through a new `ParticleBillboard` extract channel on `ISceneView::render` —
already-simulated world-space particles billboarded directly (the particle analogue of
`DeformableMeshView`), uploaded to a host-visible `GPUParticle` buffer by
`ParticleSystem::prepare_billboards` and drawn by a `vkCmdDraw` in `ParticlePass`. The editor
adds "GameObject ▸ Particle Emitter", an Add-Component entry, an Inspector section
(effect/seed/playing), and `.sushiscene` persistence.

**Alpha particles (VFX2a).** `GPUEmitter` now carries the emitter's blend and sort modes. During
the compute compaction, `particle_simulate.comp`/`particle_emit.comp` bucket each particle by
blend — additive/premultiplied into `particle_draw`, true-alpha into `particle_alpha` — each with
its own `VkDrawIndirectCommand` in a single `particle_args` buffer (additive at offset 0, alpha
at 16). `ParticlePass` draws the two buckets with two pipelines: the additive glow
(`source+destination`, order-independent) and a premultiplied "over" for true alpha
(`source + destination*(1-a)`), both on the premultiplied fragment output. Smoke and dust
composite correctly instead of only glowing. Deferred to the next VFX2 slice:
clustered-punctual-light particles (the froxel version — needs camera-relative conversion).

**Alpha depth-sort (VFX2a).** The alpha bucket is bitonic-sorted back-to-front on the GPU. A new
`ParticleSortPass` (`engine/presentation/render/source/passes/particle_sort_pass.*`, compute,
between the sim and draw passes) runs `particle_sort.comp`: mode 0 seeds one
`{-distance², index}` key per pool slot from the alpha list's camera distance (padding slots sink
to the end); mode 1 is one bitonic compare-exchange stage, dispatched `log2(N)*(log2(N)+1)/2`
times by the host over the power-of-two pool capacity — the engine's "dispatch a host-known max,
read the alive count on the GPU" idiom, so no indirect dispatch is needed (the tree's first GPU
sort).

The seed stage always runs so the key buffer has a producer; the bitonic stages run only when
`ParticleSystem::needs_alpha_sort()` — that is, when at least one active emitter both blends
true-alpha and asks for `VFX::SortMode::ViewDistance`, which is the authored `RenderModule::sort`
field and defaults to sorting. `SortMode::None` is the opt-out, and it is a whole-pass one: the
alpha bucket is shared across emitters and the bitonic pass costs the padded pool whatever
fraction of it is occupied, so excluding one emitter's particles from the bucket would save
nothing and cost their blending (the only other bucket composites additively). The alpha draw
uses `particle_sorted.vert`, which indexes the alpha list through the sorted keys (binding 2 on
the draw pass's set), so `gl_InstanceIndex` walks particles far-to-near; the additive and
billboard draws keep the direct `particle.vert`. The pool capacity was set to 2^16 to keep the
sort tractable.

**Lit particles (VFX2b).** The true-alpha bucket (smoke, dust) is lit by the sun; the additive
bucket (fire, sparks) stays emissive. The sun is a **world-space** directional light
(`Environment::sun`), so lit particles need no camera-relative conversion — sidestepping the
biggest hazard of clustered-light particles. The billboard fragment shades the sprite as a
camera-facing hemisphere (a spherical normal from the sprite offset), takes
`max(dot(n, sun_dir), 0)`, and adds a flat ambient; the sun direction, radiance, and a per-draw
lit flag ride the 128-byte push constant (which replaced the unused viewport lanes). The
clustered-punctual-light and IBL-SH version (camera-relative, set-0 b13-17) is the deferred
refinement.

### 1.2. Clustered lights and shadows on particles (VFX2c)

**Clustered punctual lights.** The lit bucket also receives the scene's point and spot lights.
The fragment maps the sprite to a froxel and accumulates that cluster's lights with the same
windowed inverse-square and spot-cone falloff the meshes use, without the BRDF — a puff is a
diffuse blob, not a surface — and the flat VFX2b ambient became `gi_sh_irradiance` from the
environment's SH.

The hazard is that the froxel lights live in **camera-relative** space while particles are
absolute world space, so `particle.vert` carries the sprite's `centre − eye` and its view depth
(`gl_Position.w`, the same quantity the mesh path's z-slice is keyed on) down to the fragment in
one varying. Rather than re-architecting `SceneLayout`, the pass declares the light buffer,
cluster grid, index list, froxel config, and IBL SH on **its own set** (bindings 3–7), and the
binding-free froxel primitives were extracted from `clustered_lighting.glsl` into
`clustered_lighting_common.glsl`, now shared with `pbr.frag`.

**Shadowed particles.** The sun term is multiplied by the sun's cascade visibility, so a smoke
column standing in the shadow of geometry goes dark instead of reading as uniformly sunlit. The
cascade matrices are already camera-relative, so the carried centre serves the shadow projection
as well as the cluster lookup. `particle_shadow.glsl` is a **particle-specific** sampler, not the
mesh path's: a puff has no surface normal for the normal-bias acne fix to work with, and the mesh
path's blocker search plus twenty-odd filter taps would be paid once per overlapping sprite layer
under a transparency pass's overdraw. It picks the cascade, projects once, takes a four-tap
fixed-radius Vogel disc through the comparison sampler, and fades out over the last cascade — a
soft, chunky shadow, which is what a volumetric medium wants.

The cascade block and atlas are bound on the pass's own set at the **scene set's own binding
numbers** (10 and 11, free here), so the shared `shadow_common.glsl` declaration is reused
verbatim rather than copied; the sampler-free cascade arithmetic (cascade select, atlas tiling,
the Vogel disc, the atlas texel) moved into that file from `shadow_sampling.glsl`, which keeps
only what needs a sampler — the split the file's own header already described.

### 1.3. Velocity-stretched particles (VFX3a)

`RenderAlignment::VelocityStretched` had been in the authoring model since VFX1 with no renderer
behind it. It now works: the vertex stage aims the quad's long axis down the particle's
screen-projected velocity and lengthens it by `size + speed * velocity_stretch`, so a spark reads
as a streak. `RenderModule::velocity_stretch` (streak metres per m/s) is the authored scale,
carried through `CompiledEmitter` into `GPUEmitter`'s spare lanes — the record's size did not
change. Two degenerate cases fall back to camera-facing, which is what a zero-length streak *is*:
a particle barely moving, and one flying straight at the eye (whose screen-projected velocity is
null).

The alignment is a property of the **emitter** while the draw list is per-**particle**, so the
two GPU draws bind this frame's emitter table to the vertex stage (binding 12) and index it by
the particle's `emitter_index`. Deterministic billboards belong to no GPU emitter — index 0 of
the table is an unrelated cosmetic emitter, and on a billboard-only frame there is no table at
all — so they got their own `particle_billboard.vert` and pipeline instead of a flag on the
shared one: the two shaders differ in what they may *read*, which is a pipeline property. The
quad expansion itself is not duplicated across the three: it moved into `particle_quad_offset` in
`particle_common.glsl`, which was already the shared home of the record layouts, so a future
alignment mode (ribbon, mesh) is added in one function.

### 1.4. Ribbons and trails (VFX3b)

`RenderAlignment::Ribbon` draws a particle as a tapered strip through its own recent positions. A
trail is state that has to outlive the frame that recorded it, so it lives where the pool lives:
`ParticleSystem` owns a second device-local buffer of `capacity * TRAIL_POINTS` `vec4`s (xyz
sample position, w its size), zero-cleared on the pool's one-time clear path. The simulate step
shifts every sample one place down and writes the new position at index 0 — a ring would save
seven `vec4` moves but would need a head index in the vertex stage, and the 128-byte draw push
has no room for one. The emit step collapses the whole history onto the birth point, because a
recycled slot still holds the previous occupant's trail and would otherwise streak from wherever
that particle died.

Ribbons form a **third indirect draw bucket**, keyed on geometry rather than blend: a strip is a
different vertex count per instance than a quad, which is a property of the draw and cannot be a
branch inside a shader. `particle_args` grew to three `VkDrawIndirectCommand`s (ribbons at offset
32) and `particle_ribbon.vert` expands each instance into `(TRAIL_POINTS - 1) * 6` vertices. Each
sample's side offset is perpendicular to both the local trail direction — taken from the
*neighbouring* samples, so a corner shared by two segments resolves identically from both and the
band has no crease — and the eye ray, so the strip presents its width to the camera and twists
with the path. Width and alpha taper to zero at the tail, the width coming from the sample's own
recorded size so a size-over-life curve is already in the strip's profile. No ribbon fragment
shader was needed: the shared sprite fragment's radial falloff becomes a soft-edged band when the
vertex stage emits `v = 0.5`, collapsing it to `1 - u²`.

Because the bucket is keyed on geometry it mixes authored blends, so ribbons composite with the
premultiplied "over" whatever the emitter asked for; splitting the bucket by blend is the
increment that would lift that. Shading is no longer part of the limitation —
[the particle material](#111-the-particle-material) moved the lit flag onto the emitter, so a
trail is lit if its author said so. Ribbons remain outside the alpha sort.

`RenderAlignment::Beam` shares that bucket outright rather than opening a fourth one. A beam is a
strip between two authored endpoints instead of through a particle's own recent positions, and
that is the only difference — the strip expansion, the texturing and the draw are the ribbon's,
and the two differ by which function `particle_ribbon.vert` samples and by the tail taper, which
a beam does not apply because both its ends are authored.

The endpoints, width, sag and lateral jitter live in `VFX::BeamModule` on the emitter descriptor,
emitter-local so the span travels with its emitter; `ParticleSystem::prepare` bakes them to world
space by the same transform it applies to a force field's centre, and `particle_beam_sample`
evaluates a point along the span for a given `t`. Sag bends the line under an imagined gravity
and the jitter — seeded from the drawing particle, so several live particles draw several
distinct arcs between the same two points — displaces it laterally; with both at zero the beam is
exactly straight. It is cosmetic only: the deterministic CPU backend draws nothing, so a beam
never affects simulation state.

### 1.5. Mesh particles (VFX4)

`RenderAlignment::Mesh` draws each particle as a solid mesh instance. It is the one particle path
that is not transparent, and that decides where it runs: `ParticleMeshPass`
(`engine/presentation/render/source/passes/particle_mesh_pass.*`) sits immediately after the
opaque pass, loads the scene's HDR colour and depth, and depth-tests and depth-writes like any
other solid surface. In the transparency pass, overlapping debris would composite in list order,
which no blend mode fixes — so this is a pass rather than a fourth bucket in `ParticlePass`.

A draw binds one mesh, so mesh particles cannot share a single indirect command the way sprites
do. Up to `MAX_MESH_EMITTERS` (4) mesh-aligned emitters each claim an equal slice of one shared
list and one `VkDrawIndexedIndirectCommand`; the emitter carries its slice in
`GPUEmitter::mesh_slot`, and an emitter past the last slice draws nothing rather than borrowing
another's mesh. The command's index count is a host fact and its instance count a GPU fact, so
the sim pass seeds the whole command from `ParticleSystem`'s mesh-draw table — which is why
`prepare` now takes the `MeshRegistry`. The converse constrains the shader: the instance count is
a GPU atomic the host cannot clamp, so `particle_mesh.vert` clamps its index to the slice and an
overflowing instance redraws the last particle instead of reading into the next emitter's slice.

The mesh supplies geometry; the particle supplies placement — position, uniform scale from its
size, a tumble about its direction of travel (Rodrigues about the velocity axis, angle from the
roll the update step already integrates), and a colour tint over the mesh's vertex colour. Faces
are not culled, since debris turns. Shading is deliberately not `pbr.frag`: a mesh particle
carries no material, and reaching the bindless material heap would tie the pass to the whole
scene set, so it is a diffuse surface lit by the sun (through `particle_shadow.glsl`, its second
consumer) plus a flat ambient. A mesh particle that needs a real material is a mesh instance, not
a particle.

The design originally sketched wiring this onto the GPU-driven instance path; that is not
possible as written, because `InstanceSystem` builds its records host-side each frame and a
GPU-simulated particle's transform never reaches the host. Consequences of the pass writing only
colour and depth: mesh particles are not pickable, contribute no motion vectors, and are absent
from SSR, GTAO, and the GPU cull.

### 1.6. Force fields (VFX5a)

Gravity, drag, and turbulence act everywhere alike; a force field has a *place*, which is what
makes it the tool for authored motion. `ForceFieldModule` places up to `MAX_FORCE_FIELDS` (4) of
them per emitter: `Point` pulls toward or pushes from a centre, `Vortex` swirls about an axis
through it, `Drag` damps velocity inside it. Point and Vortex contribute acceleration; Drag
returns a factor applied to velocity after integration, alongside the emitter's own drag.

The weight is `pow(1 - distance/radius, falloff)` — one at the centre, zero at the rim. That
bound is the point: a field touches nothing outside its radius, so it costs nothing to particles
elsewhere, and it never spikes to infinity at the centre the way a raw inverse-square does. The
count is fixed because `CompiledEmitter` is the byte-comparable POD both backends and the GPU
read, so a variable-length list cannot live in it; the compiler takes the first four *enabled*
entries.

Fields are authored in the emitter's local frame so they travel with their emitter, and placed
into world space once per step rather than per particle — `ParticleSystem` bakes the emitter
matrix into `GPUEmitter` when it flattens the frame, and `CPUDeterministicBackend::integrate`
(which now takes the emitter pose for exactly this) applies the pose at the top of the step. The
GLSL `particle_force_fields` and the deterministic backend's field loop are line-for-line
counterparts, as `curl_noise` already is: they read the same authored record, so they must agree
on what it means. They are not required to be bit-identical to each other — they are different
simulation domains — only each deterministic in itself.

### 1.7. Depth collision (VFX5b)

Cosmetic particles bounce off whatever the camera can see, tested against depth the renderer
already produced — no collision geometry, no broadphase. The apparent obstacle was pass order:
`ParticleSimPass` runs *before* `depth_prepass`, so this frame's depth does not exist when the
particles move. But `HiZPass` owns a **persistent** image rather than a graph transient, so at
sim time its level 0 still holds last frame's linearised depth (`near / depth` — exactly the
linear view distance the test needs). One frame of lag on a spark's bounce is invisible; this is
the same cross-frame read `cull_pass` already makes of the occlusion pyramid.

`HiZPass::has_history()` reports whether there is anything to read: false before the first build
and while the pass is off (it follows SSR), because an image never written holds garbage and has
never left `UNDEFINED`. The sim pass then binds a 1×1 stand-in it owns and clears the collision
bit in its push constant — a combined-image-sampler binding needs a real view even on a frame the
shader will not read it.

The test projects the particle, reads the pyramid at its pixel, and compares view depths: in
front is free, further behind than the authored thickness is also free (the depth buffer records
a surface, not a solid), and between the two is contact. The normal comes from the depth gradient
— two neighbouring taps turned back into camera-relative positions and crossed — falling back to
camera-facing where a depth jump says the neighbours are on a different surface, since a gradient
across a silhouette points nowhere real. The response keeps `restitution` of the normal velocity,
sheds `friction` of the tangential, and lifts the particle out along the normal by its
penetration. The sim push block reached exactly its 128-byte budget doing this: a
view-projection, the camera basis with the half-fov tangents in the spare `w` lanes, and two
vec4s of counts.

What it cannot do follows from the same design: particles off screen, behind the viewer, or
hidden behind something nearer pass straight through. That is the right trade for sparks
skittering off a visible floor and the wrong tool for gameplay, so it is a **cosmetic-path
feature** — the deterministic backend has no counterpart, and the authoring UI says so.

### 1.8. Effect assets, library, and timeline (VFX6)

`engine/world/serialization/include/SushiEngine/serialization/effect_serializer.hpp` and
`engine/world/serialization/source/effect_serializer.cpp` read and write `.sushieffect` files:
JSON in the same shape and spirit as `.sushiscene`, one object per emitter with a sub-object per
module. What is persisted is the **descriptor** tree, never the compiled record — the compiled
form is a build product whose layout has changed with every phase of this subsystem, so writing
it would tie a saved asset to a moving target. Curves and gradients go out as their authored key
lists rather than baked LUTs for the same reason. Reads default rather than fail: a missing key
keeps the module's default, so a file written by an older build loads with the newer defaults,
and a numeric enum from a newer build that is out of range falls back instead of becoming an
out-of-range enum the consumers would later switch on.

The particle panel gained a "Library" section listing `assets/effects/*.sushieffect` with load
and save, re-reading the directory on demand rather than watching it — an author saves far less
often than the panel redraws. It also gained a timeline: the emitter's cycle as a bar with its
burst times marked and a draggable play head, calling `EffectPreview::seek`
(`applications/editor/source/vfx/effect_preview.*`).

What that seek means depends on which backend is previewing, and the split is the honest one. In
the GPU preview it moves the **emission schedule**: the rate and burst evaluation jump, and the
fractional-particle accumulators are cleared so their debt does not spill as a burst at the new
time, but the particles already alive do not wind back and cannot — the cosmetic pool lives on
the GPU and advances one step per rendered frame, so there is no host copy to rewind.

A **"CPU (scrubbable)"** mode previews through `CPUDeterministicBackend` instead, and there the
scrub is exact: that backend is a pure function of (state, emitter, dt), so seeking resets the
pools and replays from zero at a fixed step, reproducing precisely the frame that time would have
shown. The replay is capped at 4096 steps so dragging to the end of a long cycle cannot stall the
editor. The CPU preview steps *every* emitter whatever domain it declares — the domain says which
backend ships the effect, while the preview's job is to show the author the same asset through
the other one — and its particles come out as `Render::ParticleBillboard`s, the channel the sim's
own deterministic emitters already use, which the viewport concatenates with the sim's rather
than adding a channel for.

The panel also draws the module stack as a left-to-right node graph with click-to-toggle. That is
a **presentation** of the authoring model, not a second one: the stack order (spawn → shape →
init → update → render) is the pipeline a particle actually goes through, so the graph is that
pipeline laid out rather than a free-form canvas whose edges would only have to be validated back
into the same fixed order.

### 1.9. Scene emitters are entities

The Scene view used to draw the previewed effect, which belonged to no entity: a fire nobody
could select, move, or delete, sitting in a view whose whole job is to show the world. It is gone
from there and has its own **Effect Preview** viewport, which draws that effect and nothing else
— no instances, no world lights, no world emitters. What the Scene view shows is the world's
emitters, and those are entities.

**Both backends, chosen by the effect.** `Simulation::RenderScene` gained `particle_emitters`
alongside `particle_billboards`. Which channel an emitter entity feeds is not a new component
field: it follows the domain each `CompiledEmitter` already declares. A Deterministic emitter is
stepped on the fixed tick into the record's host pool and extracted as finished billboards; a
Cosmetic one is not stepped at all — the sim only *places* it (transform, this frame's spawn
count, the compiled record and its LUT atlases) and the renderer emits and integrates it on the
GPU. That is what lets a scene emitter reach ribbons, mesh particles, depth collision, and counts
a 1024-particle host pool could never hold. The per-emitter runtime state (play head,
fractional-spawn carry) lives on the sim's record rather than on `ParticleEmitterParameters`,
because those are the *authored* parameters the scene file round-trips and a play head is neither
authored nor persisted.

**How an authored effect reaches the world.** The sim owns no asset loader — `.sushieffect` files
are the editor's business — so authored effects arrive through
`IWorldEditor::register_particle_effect`, which registers by **name**, replacing an existing
entry of the same name. Replacing rather than appending is the point: an edit in the particle
panel then lands on every emitter already playing that effect, which is what a shared asset is
supposed to do. The editor registers everything under `assets/effects` at startup, and the
panel's "Apply to World" and "Assign to Selection" buttons push the authoring surface into the
scene. The built-in Fire/Sparks/Smoke are simply the first three entries of the same library,
seeded by the sim.

One correctness fix came with it: the deterministic step used to take `emitters[0]`
unconditionally, which fed a cosmetic emitter to the CPU pool in any effect that mixed domains.
It now picks the effect's first *deterministic* emitter, and skips the entity when there is none.

### 1.10. The Particle System is a component, not a panel

There is no Particle Editor window. Adding a **Particle System** to an entity is what makes that
entity emit, so the whole authoring surface — emission, shape, forces, force fields, collision,
over-life curves, render alignment, the module graph, the timeline — is drawn inside that
component's Inspector section. A particle system is not a mode the editor is in; it is something
an entity has.

**The effect is the component's own data.** It lives on the entity, in the world, reachable
through `IWorldEditor::particle_effect_source` / `set_particle_effect_source`, and not as an
index into a shared library. Two consequences follow, and both were the point: editing one
emitter can never change another, and the scene file round-trips the effect with the entity
(`capture_effect` under the entity's `particle_emitter.source`) instead of saving an index whose
meaning depends on load order. A freshly added component is seeded with a visible default,
because a component that shows nothing reads as broken rather than as an invitation to author.
Files written before the effect moved onto the component simply keep that default.

The Inspector holds a scratch copy of the selected entity's effect for the widgets to bind to,
re-read whenever the selection moves, and writes it back while a widget is active — by which
point the value has already changed, so a drag is covered frame by frame — plus a dirty flag for
the one change no widget reports, a library load. `EffectDatabase::replace` makes the write in
place, so dragging a slider neither grows the database nor leaves dead compiled forms behind.

Library entries under `assets/effects` are **templates**, not bindings: clicking one copies it
into the selected emitter, so editing that emitter never touches another that started from the
same file.

**One preview surface.** The **Preview** viewport is the single screen anything being authored is
shown on, in isolation — no world instances, no world lights, no world emitters — and it carries
the transport for whatever it shows, effect and character alike, because "the thing being
authored, playing or paused" is a property of the surface rather than of the subject. The
Inspector mirrors the edited effect into it and points it at the entity's position, so the
isolated view and the Scene view never disagree. Nothing else calls itself a preview window: what
was "Animator Preview" is now "Animator", since layers, masks, and IK are authoring, not a
preview. The Scene view shows the world, where emitter entities are simply live; a **"Preview in
Scene"** toggle additionally draws the previewed effect there, off by default because an effect
belonging to no entity cannot be selected or deleted and should not squat in the view that shows
the world.

### 1.11. The particle material

Four render-module fields were authored, compiled, serialized, and shown in the Inspector from
VFX1 onward, and ignored by the renderer: the sprite texture, its flipbook grid, the
soft-particle fade, and `lit`. Closing that gap is what the particle material is.

It is deliberately *not* a PBR material. A puff has no roughness, no normal, no parallax; what it
has is a base-colour sheet, a cell in that sheet, a fade where it meets geometry, and a choice
about whether light touches it. A particle that genuinely needs a surface is a `Mesh`-aligned
one, drawn by `ParticleMeshPass` against real geometry. Keeping the split means the sprite path
never has to bind the scene set.

**Texture.** `ParticlePass`'s pipeline layout gained set 1 — the same slot, the same
`DescriptorHeap::layout()`, and the same `sampler2D bindless_textures[]` array `pbr.frag` samples
— so a sprite texture is registered once and addressed by the very index a material map is. The
authored value is a texture-library id; only the library knows which heap slot that id currently
occupies, so `ParticleSystem::prepare` resolves it while it flattens the frame (which is why it
takes the `TextureLibrary` alongside the `MeshRegistry`). A `RENDER_TEXTURED` flag, not a
sentinel index, says whether the slot is real: the untextured and textured cases differ in more
than the sample — untextured is the built-in radial dot, textured hands the falloff to the
texture's own alpha, and applying both would vignette every authored sheet twice — and it is the
bit the renderer clears when a texture cannot be resolved, which is what keeps the fragment stage
from indexing a slot that was never allocated.

**Flipbook.** `particle_simulate.comp` has chosen a cell from the particle's normalised age since
VFX1; nothing read it. `particle_sprite_uv` maps a quad corner into that cell — picked, never
interpolated, so the sub-image swaps cleanly instead of smearing between frames. A ribbon gets a
strip mapping instead, running head-to-tail down the trail, because that is how a trail sheet is
drawn.

**Soft particles.** Reverse-Z with an infinite far plane, so a stored depth linearises to
`near / depth` and the sky reads as infinitely far, which is the right answer when nothing was
hit. The fragment's own view depth is the interpolated `1 / gl_FragCoord.w` rather than the
sprite centre's, so a large billboard fades correctly across its own extent. The hard occlusion
discard stays as the cheap early-out; the fade handles the contact band it leaves behind.

**Lit is per emitter, not per draw.** Whether a particle receives the sun, its cascade shadow,
the clustered punctual lights, and the SH ambient used to be inferred from the *bucket* — so the
true-alpha bucket was always lit and the additive and ribbon buckets never could be. A bucket
mixes emitters, so it was always the wrong granularity. The flag now rides the emitter table down
to the fragment as a flat varying, and the push constant's spare lane carries only the ambient
scale. This also retires the documented limitation in
[ribbons and trails](#14-ribbons-and-trails-vfx3b) that a trail could not be lit.

**Persistence is by path.** A texture id means nothing to the next session, so `.sushieffect`
files and the `.sushiscene` particle emitters round-trip `RenderModule::texture_path`, and
`resolve_effect_textures` derives the handle after a load. The capture carries the live handle
too, because the same code serves the in-memory snapshots undo/redo and play-mode take — where
the handles are still the right ones and re-reading every texture off disk to restore them would
be absurd; a load from disk overwrites it from the path, so a stale one never crosses a session
boundary.
