# tests — one binary, se_functional_tests, built by the target sushiengine_functional_tests and
# covering unit/, integration/ and regression/. Every
# test in the tree is functional, so those three directories are the whole partition. The engine
# is header-only and ships no device code of its own, so the test binary is a single SYCL
# translation-unit set: every test that drives the ECS schedule or the PGS solver instantiates
# its kernels here and runs them against the real runtime — there are no mocks (see CLAUDE.md).
#
# Tests are grouped into CTest labels (unit / integration / regression) from the GTest
# suite-name prefix convention (Unit_* / Integration_* / Regression_*) rather than from the
# directory a case sits in, so `ctest -L unit` (or the umbrella regex the CLI uses) selects a
# sub-suite and new tests are picked up automatically as long as they follow the convention.

cmake_minimum_required(VERSION 3.22) # gtest_discover_tests(TEST_FILTER ...)

include(GoogleTest)

# Header-only JSON: the input-bindings serialization test round-trips through
# SushiEngine::Input::bindings_to_json/from_json, which is the one input header that
# includes nlohmann/json (the core action layer stays dependency-free). Same package the
# editor already resolves from vcpkg.
find_package(nlohmann_json CONFIG REQUIRED)

set(SE_TEST_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")

# Applies the cross-cutting settings to a test binary: the shared-helpers include
# dir, the engine + GTest links, and the shared SYCL policy (device flags + the
# Windows runtime-DLL copy) from cmake/SyclTarget.cmake — the same helper the
# engine's example apps use, so the two lanes cannot drift apart.
function(sushiengine_configure_test_target target)
    target_include_directories(${target} PRIVATE ${SE_TEST_COMMON_DIR})
    target_link_libraries(${target} PRIVATE SushiEngine GTest::gtest)
    set_target_properties(${target} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
    sushiengine_apply_sycl(${target})
endfunction()

add_executable(sushiengine_functional_tests
    common/test_main.cpp

    # The hazard semantic every execution backend answers to: the safety and
    # determinism floors of SushiEngine::Execution, driven through the vocabulary so
    # the same cases can be re-pointed at each backend's compiler as they land.
    unit/test_hazard_semantics.cpp
    unit/test_world.cpp
    unit/test_command_buffer.cpp
    unit/test_graph_coloring.cpp
    unit/test_floating_origin.cpp
    unit/test_floating_origin_stress.cpp
    unit/test_network_id.cpp
    unit/test_ui_layout.cpp
    unit/test_collision.cpp
    unit/test_broadphase.cpp
    unit/test_contact_body.cpp
    # Contact manifolds: the several points two shapes touch at, which is what a
    # box needs to rest rather than rock (P1, §7.3).
    unit/test_manifold.cpp
    # Resolving one: non-penetration, static friction positionally, and the
    # velocity pass that finally makes restitution expressible (P1, §7.4).
    unit/test_contact_projection.cpp
    # A contact as a constraint kind the solver holds: coloured against the
    # persistent kinds in one union, banded per colour, and rebuilt every tick
    # (P1, §6.3).
    unit/test_contact_constraint.cpp
    # The XPBD distance constraint, measured against a conservation law rather than
    # against a transcription of itself — which is what let a frame error survive.
    unit/test_distance_projection.cpp
    # The general convex narrowphase: one GJK/EPA routine that collides every
    # convex shape with every other, so a new shape is a support function (P2, §7.2).
    unit/test_gjk.cpp
    # And turning its single deepest point into the patch two shapes rest on, so a
    # cooked hull settles the way a primitive box already does (P2, §7.3).
    unit/test_convex_manifold.cpp
    # And the table that says which routine collides which pair, so a new shape is a
    # registration rather than a branch (P2, §4.2).
    unit/test_narrowphase_dispatch.cpp
    # §7.5 tier 2: an exact time of impact via conservative advancement, for the
    # motion tier 1's per-tick linear widening cannot bound — chiefly rotation,
    # which a linear speculative margin never sees at all (P5, §7.5 item 2).
    unit/test_conservative_advancement.cpp
    # §7.5's signed-distance-field narrowphase path: the gradient-as-normal
    # arithmetic against a field with a known closed form, both dispatch
    # orders, and the empty-field refusal (P5).
    unit/test_sdf_manifold.cpp
    # Static triangle-mesh geometry: the cooked hierarchy and the internal-edge
    # correction that stops a crate tripping on the seams (P2, §7.2).
    unit/test_mesh_collision.cpp
    # The last two shape kinds: terrain found by arithmetic rather than by search,
    # and one body carrying several colliders (P2, §5.2).
    unit/test_height_field_compound.cpp
    # The broadphase seam and its two implementations, held to the same sorted pair
    # set — the §4.4 substitutability suite — plus the pair cache warm starting
    # needs and the filter rules that reject a pair before geometry (P2, §7.1).
    unit/test_broadphase_conformance.cpp
    # Asking the collision world a question that is not "what is touching": rays,
    # sweeps, overlaps and closest points, built on the same tree (P2, §7.7).
    unit/test_scene_query.cpp
    # The joint library in isolation: the frame algebra, the seven registered kinds,
    # their limits and drives, and the force recovery every one of them folds. The
    # static assertion in here is what makes an unregistered joint kind a build error.
    unit/test_joint_projection.cpp
    # The PhysicsAssembly asset and the ragdoll built on it, as translation: the blob
    # round trip, world placement, mass by volume, and the bind offset that recovers a
    # joint's pose from the capsule standing in for its bone.
    unit/test_physics_assembly.cpp
    # Connected components and the sleeping built on them — the largest performance
    # win in any real scene, and the cheapest to get subtly wrong (P2, §13.2).
    unit/test_islands.cpp
    # Mass and inertia from a shape and a density: written in P0, wired in P2, and
    # untested in between, which is exactly when a closed form goes wrong quietly.
    unit/test_mass_properties.cpp
    # P2's acceptance scenes: 1 000 mixed-shape bodies in contact and 10 000
    # settled ones, measured rather than asserted in prose (P2, §13.1).
    unit/test_collision_scale.cpp
    # The authored-components-to-physics-descriptors translation, which was
    # untestable while it lived inside RuntimeSimulation.
    unit/test_physics_extract.cpp
    # The neutral geometry module, reachable without a renderer — which is the point
    # of having moved it out from behind Vulkan.
    unit/test_geometry_sdf.cpp
    # The cooking pipeline's first stage: what a dirty mesh measures as, and what a
    # repair is allowed to change about it (P4, §8.3 stage 1).
    unit/test_geometry_mesh_utilities.cpp
    # The distance hierarchy every cooking stage queries, against the brute-force sweep
    # it replaced — a pruned descent is fast and subtly wrong in ways that stay plausible.
    unit/test_geometry_mesh_distance.cpp
    # The cooking pipeline's contracts as distinct from its geometry: the fidelity dial
    # against §8.2's table, the cache key, and the two asset stores (P4, §8.1, §8.2, §8.5).
    unit/test_cooking_core.cpp
    # Polyhedral mass properties against the closed forms for the primitives, which is
    # the strongest oracle available for them (P4, §8.4 item 1).
    unit/test_mesh_mass_properties.cpp
    # §8.4 end to end — the hull build, the decomposition, the blob's refusals, and the
    # corpus of deliberately dirty meshes P4's acceptance criterion names.
    unit/test_collision_cooker.cpp
    # §8.3 end to end — the voxel fill, the lattice's closed boundary, the embedding's rest
    # round trip (§8.6 invariants 1 and 2), the level chain, and the .sushisoft blob.
    unit/test_soft_body_cooker.cpp
    # §8.1's import chain: who decides which cookers run, what the chain does when one of
    # them fails, and that cooking happens off the calling thread and is cached end to end.
    unit/test_import_chain.cpp
    # The bake surface's decisions (§14) — whether Re-cook gets past the cache, whether the
    # overlay follows the selection.
    unit/test_cook_bake_state.cpp
    # T1, the global dynamical core, reachable without a device for the same reason:
    # it is a neutral module, so its transform, its elliptic inversion and its
    # conservation properties are testable against references that are not itself.
    unit/test_quasigeostrophic_core.cpp
    # T0, the mean state T1 is a departure from: the asset format's refusals (tested against
    # synthesized bytes, so they fail on a machine with no asset checked out) and the shipped
    # bake's claims about the actual Earth.
    unit/test_climatology.cpp
    # The path from the epoch to T0's month axis, and the determinism the quantization buys.
    unit/test_season.cpp
    unit/test_deformable_mesh.cpp
    unit/test_math_primitives.cpp
    # P6-A: §9.1's deformation gradient and its two neo-Hookean constraints,
    # against closed-form cases (rest, uniform scale, pure rotation) and a
    # finite-difference check of the hand-derived gradients themselves.
    unit/test_fem_projection.cpp
    # P6-B: §9.3's Green strain / Cauchy stress / von Mises readout, against
    # the uniaxial-strain linear-elasticity closed form and the same
    # rest/rotation-invariance/degenerate-guard cases §9.1's own tests use.
    unit/test_fem_stress.cpp
    # P6-C: §9.4's permanent dent — the runtime 3x3 inverse plasticity needs
    # and nothing else in this phase does, the below-yield no-op, full creep
    # exactly zeroing the elastic strain it just absorbed, and the
    # accumulated-strain ceiling converging to rather than overshooting.
    unit/test_fem_plasticity.cpp
    # P6-D: §9.5's removal mechanics and its three named guard rails
    # (per-tick budget, minimum fragment size, scene-level cap), against
    # hand-constructed element sets whose connectivity is known by
    # inspection -- fracture's own bookkeeping never reads a position.
    unit/test_fem_fracture.cpp
    # P6-G: §8.6's invariants 1 and 2 — every render vertex bound or counted,
    # and the embedding round-tripping the source mesh at rest — plus the
    # deformed-normal pass, whose area weighting is only checkable against a
    # case where the right answer is stated rather than recorded.
    unit/test_mesh_embedding.cpp
    # P6-H: §9.1's bending constraint, the thing today's cloth lacks. The
    # weights are asserted by the property that defines them (they annihilate
    # the rest stencil) rather than by a formula, and P6's own acceptance
    # clause -- zero stiffness reproduces the old behaviour -- is checked
    # structurally: at zero, no bending constraints exist to run.
    unit/test_bending_constraint.cpp
    # P6-I: §6.5's cosmetic float column -- the rule that picks it, the clause
    # that overrides the rule (a body a rollback replays stays in double), and
    # the holder that keeps the decision from spreading past instantiation.
    unit/test_soft_body_precision.cpp
    # P8: §6.5's second half -- a cosmetic body's stored pose narrowed to
    # `sycl::half` between ticks, widened back to `float` before the next
    # tick's projection touches it. Checked for round-trip error against
    # half precision's own bound, for actually being half the storage width,
    # and for a free-fall trajectory staying close to the same scene with no
    # narrow/widen seam at all.
    unit/test_soft_body_half_storage.cpp
    # P7-A/P7-B: §11.1's beam and §11.2's claim that its numbers come from a
    # material. The load recovery is checked against Hooke's law rather than
    # against itself -- a compliant beam must report `E*A*d/L` -- which is the
    # whole argument for deriving a compliance from a Young's modulus at all.
    unit/test_beam_constraint.cpp
    # P7-C: the `.sushinodebeam` blob. The assertions worth having are not the round
    # trip -- every format passes that on the day it is written -- but the three
    # things a wide binary format actually gets wrong: a count raised without the
    # bytes behind it, a cross-reference nobody re-checked on the way in, and record
    # padding that makes two identical cooks differ in bytes nobody wrote.
    unit/test_node_beam_asset.cpp
    # P7-D: §11.3's cooker, and §11.2's claim that a vehicle's numbers come from a
    # material rather than from a person. The assertions that earn their keep are
    # the ones that would catch a *plausible* structure: beams whose compliance is
    # not the axial bar's `L/(E*A)`, cross-sections that do not add up to the body
    # they were divided out of, and a render mesh that does not come back where the
    # artist put it.
    unit/test_node_beam_cooker.cpp
    # P7-E: the hybrid instanced. The assertions that matter are the ones about what
    # only exists once the asset is in a world: the core's principal frame surviving
    # a spawn pose, the load that dents a beam building over ticks rather than
    # arriving with the first one, a part reported detached exactly when its last tie
    # goes, and two runs of the same crash breaking the same things in the same order.
    unit/test_node_beam_structure.cpp
    # P7-F: §11.2's suspension row. The assertions are quantitative where the physics
    # has a closed form, because a strut with its spring rate read as a compliance or
    # its travel signed the wrong way still holds a car up and still moves when pushed
    # -- what it does not do is settle at `m*g/k`.
    unit/test_vehicle_suspension.cpp
    # P7-G: §11.4's one-dimensional chain. Three closed forms carry it -- a clutch below
    # its capacity locks exactly, a differential's outputs sum to what came down the
    # shaft whatever its lock setting, and the impulse the wheels gain is the impulse
    # the chassis loses, which is what stops a car driving itself around in mid-air.
    unit/test_vehicle_powertrain.cpp
    # P7-H: §11.5's contact patch. The brush model is checked against its own closed
    # forms -- linear below the limit, exactly mu*N above it, one circle rather than two
    # axes -- and the world half against the two things a patch must not get wrong: the
    # load is the one the solver recovered, and whatever it pushes against takes the
    # other end of it.
    unit/test_vehicle_tyre.cpp
    # P7-I: §11.6's cross-system tie-in, from the physics side. The wind term is a
    # difference against the still-air drag `predict` applies anyway, so the property
    # that matters most is that still air costs *exactly* nothing -- anything else would
    # perturb every determinism test in the suite by an amount small enough to blame on
    # anything at all.
    unit/test_vehicle_aerodynamics.cpp
    # The animation stack, whose every phase shipped with a demo and no test. Starting at
    # the bottom: the asset layer everything else reads through (the skeleton cook's
    # topological reorder, the clip formats and their refusals, the sampling contract, the
    # compressed format's error bound).
    unit/test_animation_clip.cpp
    # The deterministic half — the state machine's semantics, and §0.2's contract that the
    # whole animator state is memcpy-snapshottable and a rollback replay is byte-exact.
    unit/test_animator_step.cpp
    # The five blend-node kinds, resolved out of a compiled controller blob. Each is checked
    # at its authored sample points and swept for the invariant that outranks them: weights
    # partition unity, or the pose scales up and down with the parameter.
    unit/test_animation_blend_tree.cpp
    # Layer folding, avatar masks and additive blending — where a mistake is quiet, because a
    # mask admitting one joint too many reads as bad content rather than as a bug.
    unit/test_animation_layers.cpp
    # The pose-modifier stack: each solver's convergence and limits, §10's substitutability
    # claim (a zero weight is an exact no-op), and the two failure modes these solvers have
    # actually shipped with — a joint rotating itself (a no-op) and a stranded child.
    unit/test_animation_ik.cpp
    # The glTF `weights` channel lane, against a real asset: the importer, the cooked
    # clip's named morph tracks, and sample_morph_state's name-keyed target mapping.
    unit/test_animation_morph_import.cpp
    # §4.4's retargeting: the avatar's name mapping, and the three transfers built on it. Two
    # rigs with deliberately *different* bind poses, because that is the only setup in which
    # transferring the bind-pose delta and copying the rotation are distinguishable answers.
    unit/test_animation_retarget.cpp
    # The Animator's persistence seam (§4.3). The round trip is asserted by compiling the desc
    # to a `.sushictrl` before and after and comparing the bytes — a hand-written field-by-field
    # comparison would grow the same blind spot as the serializer it guards.
    unit/test_animation_controller_json.cpp
    # The authoring side (§4.2): the sparse curves a dope sheet edits, the pose recorder, the
    # bake that resamples them into a dense clip, and the generic float tracks a baked clip
    # dispatches. Every bake case checks the frame against the curve at that frame's own time.
    unit/test_animation_keyframe.cpp
    # §12.4's tail, each item at exactly the scope it claims: the motion-matching database and
    # its reference crossfade driver, dual-quaternion skinning's blend algebra against closed
    # forms and against the linear baseline it exists to beat, the ARKit-52 facial mapping, and
    # the sequencer timeline's evaluation core.
    unit/test_animation_authoring_tail.cpp
    unit/test_rng.cpp
    unit/test_fixed_timestep.cpp
    unit/test_input_action_map.cpp
    unit/test_input_tick_sample.cpp
    unit/test_input_gamepad.cpp
    unit/test_input_rebind.cpp
    unit/test_input_bindings_json.cpp
    unit/test_input_virtual_controls.cpp
    unit/test_input_players.cpp
    unit/test_input_advanced.cpp

    # Audio: the phase-S1 DSP core in isolation — the lock-free SPSC ring, the
    # flush-to-zero denormal guard, the SIMD sample kernels, the filter set (one-pole,
    # RBJ biquad, Cytomic TPT-SVF), and the block graph's one-block feedback semantics.
    # Pure header-only maths, no device backend.
    unit/test_audio_dsp.cpp

    # Audio S2: the action layer — parameter smoothing / RTPC, the mixer bus DAG
    # (insert / post-fader gain / aux send / topological order), and the voice manager's
    # virtual/real ranking, distance culling, one-shot lifetime, and panning.
    unit/test_audio_mixer.cpp

    # Audio S3: propagation — the cubic-Lagrange fractional delay line, ISO 9613-1 air
    # absorption, the distance-attenuation laws, and the per-source Doppler (rising on
    # approach, falling on recede, none when stationary, snapped on a teleport).
    unit/test_audio_propagation.cpp

    # Audio S4: the spatializer — real (ACN/SN3D) spherical harmonics, the ambisonic
    # encode/decode kernel, and the analytic binaural decode (left/right cues, ITD,
    # front symmetry, and head-tracking through the engine).
    unit/test_audio_spatial.cpp

    # Audio S5: reverb — the lossless feedback matrices (Householder/Hadamard energy
    # preservation), the Jot FDN (coprime lengths, bounded/stable, a measured RT60 that
    # tracks the request, a darker tail as the HF decays faster, predelay), and the
    # room-geometry RT60 (Sabine/Eyring + crossover + the shoebox factory), all reached
    # through the IReverb seam and the mixer aux-bus adapter.
    unit/test_audio_reverb.cpp

    # Audio S6: the ECS bridge's control-plane structure (AudioScene) in isolation — the
    # snapshot â†’ live-voice reconciliation (start on appear, move/re-gain on persist, stop
    # on silence or removal) and reverb-zone steering, driven by a hand-built snapshot.
    unit/test_audio_scene.cpp

    # Audio S7: occlusion/obstruction — the acoustic BVH (BLAS ray hit/miss, TLAS placement
    # + refit), three-band material transmission (through-wall is bassy), soft multi-ray
    # occlusion, the occlusion DSP (obstruction muffles dry, occlusion also cuts the reverb
    # send), the room/portal graph (cross-room â†’ doorway source), and image-source early
    # reflections.
    unit/test_audio_occlusion.cpp

    # Audio S8: the asset pipeline — the codecs (PCM round-trip, IMA-ADPCM fidelity + 4:1
    # density + chunked==one-shot streaming safety), the bank binary (build/load round-trip,
    # media lookup/decode, malformed-data rejection), the event/container model
    # (Sound/Random/Sequence/Switch/Blend), the bank-backed IEmitterSourceFactory, and the
    # streaming decoder/source (memory source pumped into the ring, rendered, drained).
    unit/test_audio_bank.cpp

    # Audio S9: the live-profiler telemetry channel — the seqlock audioâ†’GUI snapshot
    # (publish/latest, empty before first publish), the mixer's per-bus meters, and the
    # AudioEngine gather (real/virtual/active voice counts, master peak/RMS tracking the
    # signal, per-bus meters, downsampled scope; profiling toggle).
    unit/test_audio_profiler.cpp

    # Audio S10: procedural sources — modal impact synthesis (ring/decay/energy scaling)
    # and wind (speed-driven level + seed determinism).
    unit/test_audio_procedural.cpp

    # Audio S10: fast convolution — the radix-2 FFT round-trip, the partitioned overlap-save
    # convolver vs a direct time-domain convolution, and the convolution reverb (bounded
    # decaying tail whose length tracks the requested decay).
    unit/test_audio_convolution.cpp

    # Audio S11 (AAA gap-closing): the bus dynamics rack — compressor (+ sidechain ducking),
    # lookahead brick-wall limiter, and noise gate.
    unit/test_audio_dynamics.cpp

    # Audio S11 (AAA gap-closing): voice-layer additions — sample-rate conversion, per-voice
    # pitch, voice stealing, HDR culling, the lock-free command ring, and multi-core rendering.
    unit/test_audio_voice.cpp

    # Audio S11 (AAA gap-closing): state-based mixing — snapshot snap + timed cross-fade.
    unit/test_audio_mix_state.cpp

    # Audio S12 (AAA gap-closing, dependency-free): the authoring project â†’ runtime flatten
    # (contiguous-child layout, container semantics survive the bake), the MagLS binaural
    # decoder + anthropometric HRTF warp (laterality, head-size ITD scaling) over a synthetic
    # HRTF, and ray-traced room acoustics (RT60 vs Sabine, Maekawa edge diffraction).
    unit/test_audio_advanced.cpp

    # VFX: the particle authoring model and the deterministic CPU backend, in isolation.
    # Curve/gradient evaluation + bake, the emitter compiler's POD boundary, and spawn
    # accounting — pure header-only maths, the shared foundation of both simulation domains.
    unit/test_vfx_authoring.cpp

    # Weather W5 (world coupling): WeatherWorldCoupling's WeatherColumn -> WeatherCoupling
    # bridge, weather_wind()'s analytic-plus-perturbation sampling, and the cloud-base
    # darkening WeatherCloudscapeCompiler applies under precipitation -- pure host maths,
    # isolated from the simulation's own tick.
    unit/test_weather_world_coupling.cpp

    # Weather W6 (flight-sim polish & data ingestion): icing_risk()/turbulence_intensity()'s
    # narrow query API, the real METAR text parser + its WeatherColumn translation, and
    # IngestedWeather's three-stage GRIB-background/METAR-station blend plus the LSP
    # substitutability proof the phase's own task brief asks for.
    unit/test_weather_flight_hazards.cpp
    unit/test_metar_parser.cpp
    unit/test_ingested_weather.cpp

    # Atmosphere phase A (spatial coupling): the weather field the renderer reads coverage
    # from -- that it is non-uniform when the simulation is, where its cells land in the
    # world, that every provider can publish one, and that the reference column the shader
    # divides by is the column the deck stack was compiled from.
    unit/test_weather_field.cpp
    unit/test_atmosphere_nest.cpp

    # WM-SEED: the planetary placement behind Manual mode. The claims are about variety --
    # that a planet contains both a storm and completely clear air, that a seed decides which
    # is where, and that the same seed always decides the same way.
    unit/test_synoptic_field.cpp

    # Astro: the solar-system model behind the WGS84/atmosphere/orbital-mechanics regime.
    # Pure double-precision maths, verified against astronomical reference landmarks and
    # structural invariants (round-trips, orthonormality, energy conservation).
    unit/test_julian_date.cpp
    unit/test_orbital_elements.cpp
    unit/test_celestial_bodies.cpp
    unit/test_gravity.cpp
    unit/test_topocentric.cpp
    unit/test_body_orientation.cpp
    unit/test_surface_frame.cpp
    unit/test_star_catalog.cpp
    unit/test_scene_frame.cpp
    unit/test_reference_frame.cpp
    # Where a body's prime meridian actually is, and the body-fixed rotation built on it.
    # The decisive case is external to the conversion rather than a round trip through it:
    # the Moon is tidally locked, so from selenographic (0, 0) the Earth has to stand
    # within the libration cone of the zenith. A frame that is self-consistent but
    # measured from the wrong node puts it forty degrees away and looks fine.
    unit/test_body_frame.cpp

    # Terrain P0 (`docs/slop/solar_system_overhaul.md`): the planetary tile addressing
    # and its projection, and the editable half of the height. Host-only header maths --
    # no renderer, no device, no baked asset, which is what lets the authoritative ground
    # be checked the same way the server will evaluate it. The precision case is the one
    # that matters: §9.2's difference form against a double reference at depth 20, run
    # side by side with the naive single-precision form it replaces.
    unit/test_cube_sphere.cpp
    unit/test_terrain_layers.cpp
    # Terrain P1: the baked asset's byte layout and every refusal its reader makes,
    # driven by synthesized bytes so the refusals are tested on a machine with no asset
    # checked out -- plus a few cases against the shipped lunar pack, which skip when it
    # is absent.
    unit/test_planet_pack.cpp
    # Terrain P2a: CDLOD node selection. The property that matters is not the node count
    # but that the emitted set is a proper cut of the tree -- every point of the body
    # covered exactly once -- which is checkable exactly, because a node covers 4^-depth
    # of its face and those are exact binary fractions.
    unit/test_terrain_quadtree.cpp
    # Terrain P2b: the tile cache's bookkeeping, kept away from the device on purpose.
    # The inheritance rectangle is checked by computing the geographic point it lands on
    # both ways -- a half-texel error there shifts terrain everywhere it inherits and is
    # undiagnosable from a screenshot.
    unit/test_tile_residency.cpp
    # Terrain P2b: the camera's crossing into the body's own frame. Header-only on the
    # renderer's side precisely so it can be checked here -- no device, no pack, no frame
    # in flight -- because the three conventions that meet in it (column order, which
    # half-space a plane keeps, when the body offset is taken) are each wrong in a way that
    # still draws a planet.
    unit/test_terrain_frame.cpp
    # §18 R8 (closed 2026-08-02): the run-report-to-statistics conversion that
    # groups a synthetic RunReport's node timings by name, against known-correct
    # sums -- the same grouping `soft_body_budget.cpp` does by hand, checked here
    # without needing a live runtime or device to produce the report.
    unit/test_physics_statistics.cpp

    integration/test_schedule.cpp
    integration/test_pgs_solver.cpp
    integration/test_xpbd_solver.cpp
    integration/test_physics_world.cpp
    integration/test_physics_bridge.cpp
    integration/test_physics_simulation.cpp
    # The mutable world and the one-composition tick. The compile-count assertions
    # here are what stop the late-binding design from silently regressing into a
    # graph that recompiles every time a body is spawned.
    integration/test_runtime_graph_builder.cpp
    # P8's per-island substepping leans on Execution::DynamicGraph/Region, which sat
    # unreached through the engine's own execution seam until now (docs/slop/
    # physics_system.md §6.6, §18 R3). Seam-level, not physics-level: two regions
    # keyed independently, a drop that takes effect immediately, and a run that does
    # not recompose when nothing changed.
    integration/test_execution_dynamic_graph.cpp
    # The shared IConstraintSolver conformance suite: the same scenes against every
    # implementation, which is what lets a device solver replace a host one.
    integration/test_solver_conformance.cpp
    # P3's acceptance criterion: the chassis-plus-hinged-door scene end to end,
    # driven through IJointService on the live simulation rather than through the
    # solver, because the claim is about the whole path and not the arithmetic.
    integration/test_joint_assembly.cpp
    # §16.44's park_sleeping_joints: a settled island's joint drops from the solve
    # graph rather than dispatching for its projection's own early return to catch.
    integration/test_joint_parking.cpp
    # P7's own acceptance criterion, in the roadmap's own words: a drivable vehicle that
    # deforms permanently on impact and loses parts, deterministic under replay. Every
    # earlier P7 file tests one piece against its own closed form; this one tests that the
    # pieces are a car. The §13.1 timing row is measured and printed rather than asserted,
    # because the target is quoted for a desktop GPU through SushiRuntime and this suite
    # runs the host reference solver on the CPU.
    integration/test_vehicle_acceptance.cpp
    integration/test_rollback.cpp
    integration/test_deterministic_replay.cpp
    integration/test_net_reconciliation.cpp
    integration/test_net_client_server.cpp
    # P6's own acceptance criterion: the cantilever-deflection test against
    # Euler-Bernoulli theory, run on FiniteElementModel end to end (the
    # *mukavemet* test, §15 item 3, §16's P6 row).
    integration/test_finite_element_model.cpp
    # P6-C's own acceptance phrase, "a body past yield keeps a permanent
    # dent," run through step()'s real substep loop rather than by calling
    # apply_fem_plasticity directly.
    integration/test_fem_plasticity_integration.cpp
    # §9.6.1: a soft surface against a rigid body's cooked distance field —
    # depth and normal at any penetration, a body resting at its thickness,
    # and the reaction the rigid half of the pair takes (P6-E).
    integration/test_soft_rigid_collision.cpp
    # §9.6.2: two deformable surfaces, the closest-feature weights a correction
    # is split by, and the swept test that catches a crossing a discrete one
    # cannot see at all (P6-E).
    integration/test_soft_soft_collision.cpp
    # §9.6.3: a surface against itself, behind the flag that keeps it off for
    # every body that does not need it (P6-E).
    integration/test_soft_self_collision.cpp
    # §4.4's conformance suite for ISoftBodyModel: one set of cases run against
    # all three implementations, which is what licenses §9.7 to swap between
    # them at runtime (P6-F).
    integration/test_soft_body_model_conformance.cpp
    # §9.7: the tier selection's hysteresis band, the barycentric state transfer
    # in both directions, and the claim the transfer exists for — that a body
    # does not pop when its tier changes (P6-F).
    integration/test_soft_body_lod.cpp
    # P6-G3: a cooked asset through create_soft_body, the world, the physics
    # service and the extract, to the renderer's DeformableMeshView channel —
    # including §8.6's third invariant, that the render mesh cannot lag the
    # simulation because it is derived from it rather than copied out of it.
    integration/test_soft_body_service.cpp
    # P6-G5: what the editor's soft-body debug views mean, with the drawing left
    # out — that full scale is a material threshold rather than the body's own
    # worst element, which is the one mistake here that produces a plausible
    # picture instead of an obviously broken one.
    unit/test_soft_body_heat.cpp
    integration/test_cloth.cpp
    integration/test_app.cpp
    integration/test_input_determinism.cpp
    integration/test_particle_determinism.cpp

    # Audio S6: the wall-clock audio snapshot extract against the real ECS world — the
    # listener-local conversion (facing from the Orientation quaternion, eye-subtracted
    # emitter positions), reverb-zone containment, the read-only (byte-identical) invariant,
    # and driving an AudioScene's voices as emitters move and are destroyed.
    integration/test_audio_ecs.cpp
    integration/test_ui.cpp
    integration/test_soft_body.cpp
    integration/test_astro_dynamics.cpp
    integration/test_ephemeris.cpp

    # The editor's scene capture/apply round-trip against the real simulation:
    # Save/Load, Undo/Redo, and the Play→Stop restore all ride this one seam, so a
    # component it forgets is destroyed by all four.
    integration/test_scene_serializer_roundtrip.cpp

    # §5.5's PhysicsJoint, from the Inspector's component down to the solver: a joint
    # holds a body up that a control body identically built does not stay up without,
    # it reports the load it carries (§10.4), it tears out above a threshold and stays
    # out, and every authored field survives the scene file — including the partner,
    # which is written as an array index and resolved in the same second pass the
    # parent link uses. P3's one outstanding item was that none of this was reachable
    # from a scene at all.
    integration/test_physics_joint_component.cpp

    # §5.3's surface materials and §7.7's collision filter, both of which the physics
    # honoured and nothing could author until now: a bouncy body returns more height than
    # a dead one, a 17-degree ramp holds rubber and loses ice, and a pair on mutually
    # excluded layers passes through itself while an identical control pair stacks. Plus
    # the sample scene, which is the exposure work's own claim that everything P0-P7 built
    # is reachable without writing C++ — asserted here so the claim is tested, against the
    # same builder the generator writes the shipped `.sushiscene` with.
    integration/test_physics_authoring.cpp

    # §5.5 VehicleInstance: a cooked .sushinodebeam named by a component, instanced
    # into the scene solver, driven through the boundary, and read back onto the entity.
    # The asset is cooked into a real file rather than handed over as bytes, because a
    # *path* is what a scene file stores and what has to survive being reopened.
    integration/test_vehicle_component.cpp

    unit/test_atmosphere_quality.cpp
    unit/test_autosave.cpp
    unit/test_preferences_roundtrip.cpp

    regression/test_regression.cpp
    # §15.4's golden-metric penetration-contract scenes: resting penetration
    # within tolerance, nothing tunnels at escalating speeds, and the cooker's
    # Hausdorff error is a real per-asset measurement rather than a constant (P5).
    regression/test_penetration_contract.cpp
)
sushiengine_configure_test_target(sushiengine_functional_tests)
# The target carries the repository's prefix so it cannot collide with a SushiRuntime one in the
# single flat CMake namespace the two trees share; the file on disk keeps the short name, which
# is what `se run se_functional_tests` looks for.
set_target_properties(sushiengine_functional_tests PROPERTIES
    OUTPUT_NAME se_functional_tests)
# The input-bindings serialization test includes nlohmann/json directly.
target_link_libraries(sushiengine_functional_tests PRIVATE nlohmann_json::nlohmann_json)
# The signed-distance baker is the one compiled unit in the neutral geometry module;
# everything else the tests reach there is header-only.
target_link_libraries(sushiengine_functional_tests PRIVATE sushiengine_geometry)
# The cooking pipeline: the content hash and the two asset stores are compiled units;
# the fidelity dial, the report and the seams are header-only.
target_link_libraries(sushiengine_functional_tests PRIVATE sushiengine_physics_cooking)
# The global dynamical core and its Fourier transform are the two compiled units in
# the neutral atmosphere module.
target_link_libraries(sushiengine_functional_tests PRIVATE sushiengine_atmosphere)
# The scene-serializer round-trip drives the real simulation behind IWorldEditor; the undo
# stack, the preferences store, the autosave clock and the bake model are covered through the
# same library the editor links rather than through translation units compiled in by hand.
target_link_libraries(sushiengine_functional_tests
    PRIVATE sushiengine_simulation sushiengine_serialization sushiengine_authoring)
# The physics sample scene, as the library samples/ declares rather than as a translation unit
# compiled in from another target's directory.
target_link_libraries(sushiengine_functional_tests PRIVATE sushiengine_physics_sample_scene)
# The terrain frame maths, by include path only -- it is header-only and Vulkan-free on
# purpose, so this buys the coverage without linking the renderer into the test binary.
target_include_directories(sushiengine_functional_tests
    PRIVATE "${CMAKE_SOURCE_DIR}/engine/presentation/render/source")
# The glTF importers `sushiengine_simulation` needs, as a module that links nothing rather than as
# three of the renderer's translation units compiled in by hand. It carries cgltf's
# include directory and its one implementation unit with it, so neither is named here.
target_link_libraries(sushiengine_functional_tests PRIVATE sushiengine_gltf)
# A test that imports a real glTF needs its path, and CTest's working directory is not the
# source tree. An absolute source-dir path keeps the test independent of where it runs from.
target_compile_definitions(sushiengine_functional_tests
    PRIVATE SE_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets/models")

# One label per group; the "functional" umbrella is expressed by the consumer as
# a label regex (`ctest -L 'unit|integration|regression'`) rather than a second
# baked-in label, which does not survive gtest_discover_tests on current CMake.
gtest_discover_tests(sushiengine_functional_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Unit_*"
    PROPERTIES LABELS "unit")
gtest_discover_tests(sushiengine_functional_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Integration_*"
    PROPERTIES LABELS "integration")
gtest_discover_tests(sushiengine_functional_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Regression_*"
    PROPERTIES LABELS "regression")

