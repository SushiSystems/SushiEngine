# ============================================================================
# SushiEngine test pipeline
#
# One binary, se_functional_tests, covering functional/{unit,integration,
# regression}. The engine is header-only and ships no device code of its own, so
# the test binary is a single SYCL translation-unit set: every test that drives
# the ECS schedule or the PGS solver instantiates its kernels here and runs them
# against the real runtime — there are no mocks (see CLAUDE.md).
#
# Tests are grouped into CTest labels (unit / integration / regression) from the
# GTest suite-name prefix convention (Unit_* / Integration_* / Regression_*), so
# `ctest -L unit` (or the umbrella regex the CLI uses) selects a sub-suite and new
# tests are picked up automatically as long as they follow the convention.
# ============================================================================

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
function(sushi_configure_test_target target)
    target_include_directories(${target} PRIVATE ${SE_TEST_COMMON_DIR})
    target_link_libraries(${target} PRIVATE SushiEngine GTest::gtest)
    set_target_properties(${target} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
    sushi_apply_sycl(${target})
endfunction()

add_executable(se_functional_tests
    common/test_main.cpp

    functional/unit/test_world.cpp
    functional/unit/test_command_buffer.cpp
    functional/unit/test_graph_coloring.cpp
    functional/unit/test_floating_origin.cpp
    functional/unit/test_floating_origin_stress.cpp
    functional/unit/test_network_id.cpp
    functional/unit/test_ui_layout.cpp
    functional/unit/test_collision.cpp
    functional/unit/test_broadphase.cpp
    functional/unit/test_contact_body.cpp
    # Contact manifolds: the several points two shapes touch at, which is what a
    # box needs to rest rather than rock (P1, §7.3).
    functional/unit/test_manifold.cpp
    # Resolving one: non-penetration, static friction positionally, and the
    # velocity pass that finally makes restitution expressible (P1, §7.4).
    functional/unit/test_contact_projection.cpp
    # A contact as a constraint kind the solver holds: coloured against the
    # persistent kinds in one union, banded per colour, and rebuilt every tick
    # (P1, §6.3).
    functional/unit/test_contact_constraint.cpp
    # The XPBD distance constraint, measured against a conservation law rather than
    # against a transcription of itself — which is what let a frame error survive.
    functional/unit/test_distance_projection.cpp
    # The general convex narrowphase: one GJK/EPA routine that collides every
    # convex shape with every other, so a new shape is a support function (P2, §7.2).
    functional/unit/test_gjk.cpp
    # And turning its single deepest point into the patch two shapes rest on, so a
    # cooked hull settles the way a primitive box already does (P2, §7.3).
    functional/unit/test_convex_manifold.cpp
    # And the table that says which routine collides which pair, so a new shape is a
    # registration rather than a branch (P2, §4.2).
    functional/unit/test_narrowphase_dispatch.cpp
    # Static triangle-mesh geometry: the cooked hierarchy and the internal-edge
    # correction that stops a crate tripping on the seams (P2, §7.2).
    functional/unit/test_mesh_collision.cpp
    # The last two shape kinds: terrain found by arithmetic rather than by search,
    # and one body carrying several colliders (P2, §5.2).
    functional/unit/test_height_field_compound.cpp
    # The broadphase seam and its two implementations, held to the same sorted pair
    # set — the §4.4 substitutability suite — plus the pair cache warm starting
    # needs and the filter rules that reject a pair before geometry (P2, §7.1).
    functional/unit/test_broadphase_conformance.cpp
    # Asking the collision world a question that is not "what is touching": rays,
    # sweeps, overlaps and closest points, built on the same tree (P2, §7.7).
    functional/unit/test_scene_query.cpp
    # The joint library in isolation: the frame algebra, the seven registered kinds,
    # their limits and drives, and the force recovery every one of them folds. The
    # static assertion in here is what makes an unregistered joint kind a build error.
    functional/unit/test_joint_projection.cpp
    # The PhysicsAssembly asset and the ragdoll built on it, as translation: the blob
    # round trip, world placement, mass by volume, and the bind offset that recovers a
    # joint's pose from the capsule standing in for its bone.
    functional/unit/test_physics_assembly.cpp
    # Connected components and the sleeping built on them — the largest performance
    # win in any real scene, and the cheapest to get subtly wrong (P2, §13.2).
    functional/unit/test_islands.cpp
    # Mass and inertia from a shape and a density: written in P0, wired in P2, and
    # untested in between, which is exactly when a closed form goes wrong quietly.
    functional/unit/test_mass_properties.cpp
    # P2's acceptance scenes: 1 000 mixed-shape bodies in contact and 10 000
    # settled ones, measured rather than asserted in prose (P2, §13.1).
    functional/unit/test_collision_scale.cpp
    # The authored-components-to-physics-descriptors translation, which was
    # untestable while it lived inside RuntimeSimulation.
    functional/unit/test_physics_extract.cpp
    # The neutral geometry module, reachable without a renderer — which is the point
    # of having moved it out from behind Vulkan.
    functional/unit/test_geometry_sdf.cpp
    # The cooking pipeline's first stage: what a dirty mesh measures as, and what a
    # repair is allowed to change about it (P4, §8.3 stage 1).
    functional/unit/test_geometry_mesh_utilities.cpp
    # The distance hierarchy every cooking stage queries, against the brute-force sweep
    # it replaced — a pruned descent is fast and subtly wrong in ways that stay plausible.
    functional/unit/test_geometry_mesh_distance.cpp
    # The cooking pipeline's contracts as distinct from its geometry: the fidelity dial
    # against §8.2's table, the cache key, and the two asset stores (P4, §8.1, §8.2, §8.5).
    functional/unit/test_cooking_core.cpp
    # Polyhedral mass properties against the closed forms for the primitives, which is
    # the strongest oracle available for them (P4, §8.4 item 1).
    functional/unit/test_mesh_mass_properties.cpp
    # §8.4 end to end — the hull build, the decomposition, the blob's refusals, and the
    # corpus of deliberately dirty meshes P4's acceptance criterion names.
    functional/unit/test_collision_cooker.cpp
    # §8.3 end to end — the voxel fill, the lattice's closed boundary, the embedding's rest
    # round trip (§8.6 invariants 1 and 2), the level chain, and the .sushisoft blob.
    functional/unit/test_soft_body_cooker.cpp
    # §8.1's import chain: who decides which cookers run, what the chain does when one of
    # them fails, and that cooking happens off the calling thread and is cached end to end.
    functional/unit/test_import_chain.cpp
    # The bake surface's decisions (§14) — whether Re-cook gets past the cache, whether the
    # overlay follows the selection — compiled from the editor tree, since `CookBakeState` is
    # a UI-free translation unit for exactly this reason.
    ../editor/physics/cook_bake_state.cpp
    functional/unit/test_cook_bake_state.cpp
    # T1, the global dynamical core, reachable without a device for the same reason:
    # it is a neutral module, so its transform, its elliptic inversion and its
    # conservation properties are testable against references that are not itself.
    functional/unit/test_quasigeostrophic_core.cpp
    # T0, the mean state T1 is a departure from: the asset format's refusals (tested against
    # synthesized bytes, so they fail on a machine with no asset checked out) and the shipped
    # bake's claims about the actual Earth.
    functional/unit/test_climatology.cpp
    # The path from the epoch to T0's month axis, and the determinism the quantization buys.
    functional/unit/test_season.cpp
    functional/unit/test_cloth_mesh.cpp
    functional/unit/test_math_primitives.cpp
    # The animation stack, whose every phase shipped with a demo and no test. Starting at
    # the bottom: the asset layer everything else reads through (the skeleton cook's
    # topological reorder, the clip formats and their refusals, the sampling contract, the
    # compressed format's error bound).
    functional/unit/test_animation_clip.cpp
    # The deterministic half — the state machine's semantics, and §0.2's contract that the
    # whole animator state is memcpy-snapshottable and a rollback replay is byte-exact.
    functional/unit/test_animator_step.cpp
    # The five blend-node kinds, resolved out of a compiled controller blob. Each is checked
    # at its authored sample points and swept for the invariant that outranks them: weights
    # partition unity, or the pose scales up and down with the parameter.
    functional/unit/test_animation_blend_tree.cpp
    # Layer folding, avatar masks and additive blending — where a mistake is quiet, because a
    # mask admitting one joint too many reads as bad content rather than as a bug.
    functional/unit/test_animation_layers.cpp
    # The pose-modifier stack: each solver's convergence and limits, §10's substitutability
    # claim (a zero weight is an exact no-op), and the two failure modes these solvers have
    # actually shipped with — a joint rotating itself (a no-op) and a stranded child.
    functional/unit/test_animation_ik.cpp
    # The glTF `weights` channel lane, against a real asset: the importer, the cooked
    # clip's named morph tracks, and sample_morph_state's name-keyed target mapping.
    functional/unit/test_animation_morph_import.cpp
    # §4.4's retargeting: the avatar's name mapping, and the three transfers built on it. Two
    # rigs with deliberately *different* bind poses, because that is the only setup in which
    # transferring the bind-pose delta and copying the rotation are distinguishable answers.
    functional/unit/test_animation_retarget.cpp
    # The Animator's persistence seam (§4.3). The round trip is asserted by compiling the desc
    # to a `.sushictrl` before and after and comparing the bytes — a hand-written field-by-field
    # comparison would grow the same blind spot as the serializer it guards.
    functional/unit/test_animation_controller_json.cpp
    # The authoring side (§4.2): the sparse curves a dope sheet edits, the pose recorder, the
    # bake that resamples them into a dense clip, and the generic float tracks a baked clip
    # dispatches. Every bake case checks the frame against the curve at that frame's own time.
    functional/unit/test_animation_keyframe.cpp
    # §12.4's tail, each item at exactly the scope it claims: the motion-matching database and
    # its reference crossfade driver, dual-quaternion skinning's blend algebra against closed
    # forms and against the linear baseline it exists to beat, the ARKit-52 facial mapping, and
    # the sequencer timeline's evaluation core.
    functional/unit/test_animation_authoring_tail.cpp
    functional/unit/test_rng.cpp
    functional/unit/test_fixed_timestep.cpp
    functional/unit/test_input_action_map.cpp
    functional/unit/test_input_tick_sample.cpp
    functional/unit/test_input_gamepad.cpp
    functional/unit/test_input_rebind.cpp
    functional/unit/test_input_bindings_json.cpp
    functional/unit/test_input_virtual_controls.cpp
    functional/unit/test_input_players.cpp
    functional/unit/test_input_advanced.cpp

    # Audio: the phase-S1 DSP core in isolation — the lock-free SPSC ring, the
    # flush-to-zero denormal guard, the SIMD sample kernels, the filter set (one-pole,
    # RBJ biquad, Cytomic TPT-SVF), and the block graph's one-block feedback semantics.
    # Pure header-only maths, no device backend.
    functional/unit/test_audio_dsp.cpp

    # Audio S2: the action layer — parameter smoothing / RTPC, the mixer bus DAG
    # (insert / post-fader gain / aux send / topological order), and the voice manager's
    # virtual/real ranking, distance culling, one-shot lifetime, and panning.
    functional/unit/test_audio_mixer.cpp

    # Audio S3: propagation — the cubic-Lagrange fractional delay line, ISO 9613-1 air
    # absorption, the distance-attenuation laws, and the per-source Doppler (rising on
    # approach, falling on recede, none when stationary, snapped on a teleport).
    functional/unit/test_audio_propagation.cpp

    # Audio S4: the spatializer — real (ACN/SN3D) spherical harmonics, the ambisonic
    # encode/decode kernel, and the analytic binaural decode (left/right cues, ITD,
    # front symmetry, and head-tracking through the engine).
    functional/unit/test_audio_spatial.cpp

    # Audio S5: reverb — the lossless feedback matrices (Householder/Hadamard energy
    # preservation), the Jot FDN (coprime lengths, bounded/stable, a measured RT60 that
    # tracks the request, a darker tail as the HF decays faster, predelay), and the
    # room-geometry RT60 (Sabine/Eyring + crossover + the shoebox factory), all reached
    # through the IReverb seam and the mixer aux-bus adapter.
    functional/unit/test_audio_reverb.cpp

    # Audio S6: the ECS bridge's control-plane structure (AudioScene) in isolation — the
    # snapshot â†’ live-voice reconciliation (start on appear, move/re-gain on persist, stop
    # on silence or removal) and reverb-zone steering, driven by a hand-built snapshot.
    functional/unit/test_audio_scene.cpp

    # Audio S7: occlusion/obstruction — the acoustic BVH (BLAS ray hit/miss, TLAS placement
    # + refit), three-band material transmission (through-wall is bassy), soft multi-ray
    # occlusion, the occlusion DSP (obstruction muffles dry, occlusion also cuts the reverb
    # send), the room/portal graph (cross-room â†’ doorway source), and image-source early
    # reflections.
    functional/unit/test_audio_occlusion.cpp

    # Audio S8: the asset pipeline — the codecs (PCM round-trip, IMA-ADPCM fidelity + 4:1
    # density + chunked==one-shot streaming safety), the bank binary (build/load round-trip,
    # media lookup/decode, malformed-data rejection), the event/container model
    # (Sound/Random/Sequence/Switch/Blend), the bank-backed IEmitterSourceFactory, and the
    # streaming decoder/source (memory source pumped into the ring, rendered, drained).
    functional/unit/test_audio_bank.cpp

    # Audio S9: the live-profiler telemetry channel — the seqlock audioâ†’GUI snapshot
    # (publish/latest, empty before first publish), the mixer's per-bus meters, and the
    # AudioEngine gather (real/virtual/active voice counts, master peak/RMS tracking the
    # signal, per-bus meters, downsampled scope; profiling toggle).
    functional/unit/test_audio_profiler.cpp

    # Audio S10: procedural sources — modal impact synthesis (ring/decay/energy scaling)
    # and wind (speed-driven level + seed determinism).
    functional/unit/test_audio_procedural.cpp

    # Audio S10: fast convolution — the radix-2 FFT round-trip, the partitioned overlap-save
    # convolver vs a direct time-domain convolution, and the convolution reverb (bounded
    # decaying tail whose length tracks the requested decay).
    functional/unit/test_audio_convolution.cpp

    # Audio S11 (AAA gap-closing): the bus dynamics rack — compressor (+ sidechain ducking),
    # lookahead brick-wall limiter, and noise gate.
    functional/unit/test_audio_dynamics.cpp

    # Audio S11 (AAA gap-closing): voice-layer additions — sample-rate conversion, per-voice
    # pitch, voice stealing, HDR culling, the lock-free command ring, and multi-core rendering.
    functional/unit/test_audio_voice.cpp

    # Audio S11 (AAA gap-closing): state-based mixing — snapshot snap + timed cross-fade.
    functional/unit/test_audio_mix_state.cpp

    # Audio S12 (AAA gap-closing, dependency-free): the authoring project â†’ runtime flatten
    # (contiguous-child layout, container semantics survive the bake), the MagLS binaural
    # decoder + anthropometric HRTF warp (laterality, head-size ITD scaling) over a synthetic
    # HRTF, and ray-traced room acoustics (RT60 vs Sabine, Maekawa edge diffraction).
    functional/unit/test_audio_advanced.cpp

    # VFX: the particle authoring model and the deterministic CPU backend, in isolation.
    # Curve/gradient evaluation + bake, the emitter compiler's POD boundary, and spawn
    # accounting — pure header-only maths, the shared foundation of both simulation domains.
    functional/unit/test_vfx_authoring.cpp

    # Weather W5 (world coupling): WeatherWorldCoupling's WeatherColumn -> WeatherCoupling
    # bridge, weather_wind()'s analytic-plus-perturbation sampling, and the cloud-base
    # darkening WeatherCloudscapeCompiler applies under precipitation -- pure host maths,
    # isolated from the simulation's own tick.
    functional/unit/test_weather_world_coupling.cpp

    # Weather W6 (flight-sim polish & data ingestion): icing_risk()/turbulence_intensity()'s
    # narrow query API, the real METAR text parser + its WeatherColumn translation, and
    # IngestedWeather's three-stage GRIB-background/METAR-station blend plus the LSP
    # substitutability proof the phase's own task brief asks for.
    functional/unit/test_weather_flight_hazards.cpp
    functional/unit/test_metar_parser.cpp
    functional/unit/test_ingested_weather.cpp

    # Atmosphere phase A (spatial coupling): the weather field the renderer reads coverage
    # from -- that it is non-uniform when the simulation is, where its cells land in the
    # world, that every provider can publish one, and that the reference column the shader
    # divides by is the column the deck stack was compiled from.
    functional/unit/test_weather_field.cpp
    functional/unit/test_atmosphere_nest.cpp

    # Astro: the solar-system model behind the WGS84/atmosphere/orbital-mechanics regime.
    # Pure double-precision maths, verified against astronomical reference landmarks and
    # structural invariants (round-trips, orthonormality, energy conservation).
    functional/unit/test_julian_date.cpp
    functional/unit/test_orbital_elements.cpp
    functional/unit/test_celestial_bodies.cpp
    functional/unit/test_gravity.cpp
    functional/unit/test_topocentric.cpp
    functional/unit/test_body_orientation.cpp
    functional/unit/test_surface_frame.cpp
    functional/unit/test_star_catalog.cpp
    functional/unit/test_scene_frame.cpp
    functional/unit/test_reference_frame.cpp

    functional/integration/test_schedule.cpp
    functional/integration/test_pgs_solver.cpp
    functional/integration/test_xpbd_solver.cpp
    functional/integration/test_physics_world.cpp
    functional/integration/test_physics_bridge.cpp
    functional/integration/test_physics_simulation.cpp
    # The mutable world and the one-composition tick. The compile-count assertions
    # here are what stop the late-binding design from silently regressing into a
    # graph that recompiles every time a body is spawned.
    functional/integration/test_runtime_graph_builder.cpp
    # The shared IConstraintSolver conformance suite: the same scenes against every
    # implementation, which is what lets a device solver replace a host one.
    functional/integration/test_solver_conformance.cpp
    # P3's acceptance criterion: the chassis-plus-hinged-door scene end to end,
    # driven through IJointService on the live simulation rather than through the
    # solver, because the claim is about the whole path and not the arithmetic.
    functional/integration/test_joint_assembly.cpp
    functional/integration/test_rollback.cpp
    functional/integration/test_deterministic_replay.cpp
    functional/integration/test_net_reconciliation.cpp
    functional/integration/test_net_client_server.cpp
    functional/integration/test_cloth.cpp
    functional/integration/test_app.cpp
    functional/integration/test_input_determinism.cpp
    functional/integration/test_particle_determinism.cpp

    # Audio S6: the wall-clock audio snapshot extract against the real ECS world — the
    # listener-local conversion (facing from the Orientation quaternion, eye-subtracted
    # emitter positions), reverb-zone containment, the read-only (byte-identical) invariant,
    # and driving an AudioScene's voices as emitters move and are destroyed.
    functional/integration/test_audio_ecs.cpp
    functional/integration/test_ui.cpp
    functional/integration/test_soft_body.cpp
    functional/integration/test_astro_dynamics.cpp
    functional/integration/test_ephemeris.cpp

    # The editor's scene capture/apply round-trip against the real simulation:
    # Save/Load, Undo/Redo, and the Play→Stop restore all ride this one seam, so a
    # component it forgets is destroyed by all four. Compiled from the editor tree
    # (serialization + command history are UI-free translation units).
    # (`sushi_sim` imports a glTF skeleton and its animations to build a live world;
    # the importers it needs are `sushi_import`, linked below, and no longer three of
    # render/'s translation units compiled in by hand.)

    ../editor/serialization/scene_serializer.cpp
    ../editor/serialization/effect_serializer.cpp
    ../editor/serialization/environment_serializer.cpp
    ../editor/core/command_history.cpp
    ../editor/core/preferences.cpp
    functional/integration/test_scene_serializer_roundtrip.cpp
    functional/unit/test_atmosphere_quality.cpp
    functional/unit/test_autosave.cpp
    functional/unit/test_preferences_roundtrip.cpp

    functional/regression/test_regression.cpp
)
sushi_configure_test_target(se_functional_tests)
# The input-bindings serialization test includes nlohmann/json directly.
target_link_libraries(se_functional_tests PRIVATE nlohmann_json::nlohmann_json)
# The signed-distance baker is the one compiled unit in the neutral geometry module;
# everything else the tests reach there is header-only.
target_link_libraries(se_functional_tests PRIVATE sushi_geometry)
# The cooking pipeline: the content hash and the two asset stores are compiled units;
# the fidelity dial, the report and the seams are header-only.
target_link_libraries(se_functional_tests PRIVATE sushi_cooking)
# The global dynamical core and its Fourier transform are the two compiled units in
# the neutral atmosphere module.
target_link_libraries(se_functional_tests PRIVATE sushi_atmosphere)
# The scene-serializer round-trip drives the real simulation behind IWorldEditor,
# and the editor translation units it compiles include their own headers by
# editor-relative paths.
target_link_libraries(se_functional_tests PRIVATE sushi_sim)
target_include_directories(se_functional_tests PRIVATE "${CMAKE_SOURCE_DIR}/editor")
# The glTF importers `sushi_sim` needs, as a module that links nothing rather than as
# three of the renderer's translation units compiled in by hand. It carries cgltf's
# include directory and its one implementation unit with it, so neither is named here.
target_link_libraries(se_functional_tests PRIVATE sushi_import)
# A test that imports a real glTF needs its path, and CTest's working directory is not the
# source tree. An absolute source-dir path keeps the test independent of where it runs from.
target_compile_definitions(se_functional_tests
    PRIVATE SE_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/examples/assets")

# One label per group; the "functional" umbrella is expressed by the consumer as
# a label regex (`ctest -L 'unit|integration|regression'`) rather than a second
# baked-in label, which does not survive gtest_discover_tests on current CMake.
gtest_discover_tests(se_functional_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Unit_*"
    PROPERTIES LABELS "unit")
gtest_discover_tests(se_functional_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Integration_*"
    PROPERTIES LABELS "integration")
gtest_discover_tests(se_functional_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Regression_*"
    PROPERTIES LABELS "regression")

