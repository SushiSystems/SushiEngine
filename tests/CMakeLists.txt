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
    functional/unit/test_cloth_mesh.cpp
    functional/unit/test_math_primitives.cpp
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
    # snapshot → live-voice reconciliation (start on appear, move/re-gain on persist, stop
    # on silence or removal) and reverb-zone steering, driven by a hand-built snapshot.
    functional/unit/test_audio_scene.cpp

    # Audio S7: occlusion/obstruction — the acoustic BVH (BLAS ray hit/miss, TLAS placement
    # + refit), three-band material transmission (through-wall is bassy), soft multi-ray
    # occlusion, the occlusion DSP (obstruction muffles dry, occlusion also cuts the reverb
    # send), the room/portal graph (cross-room → doorway source), and image-source early
    # reflections.
    functional/unit/test_audio_occlusion.cpp

    # Audio S8: the asset pipeline — the codecs (PCM round-trip, IMA-ADPCM fidelity + 4:1
    # density + chunked==one-shot streaming safety), the bank binary (build/load round-trip,
    # media lookup/decode, malformed-data rejection), the event/container model
    # (Sound/Random/Sequence/Switch/Blend), the bank-backed IEmitterSourceFactory, and the
    # streaming decoder/source (memory source pumped into the ring, rendered, drained).
    functional/unit/test_audio_bank.cpp

    # Audio S9: the live-profiler telemetry channel — the seqlock audio→GUI snapshot
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

    # Audio S12 (AAA gap-closing, dependency-free): the authoring project → runtime flatten
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

    functional/regression/test_regression.cpp
)
sushi_configure_test_target(se_functional_tests)
# The input-bindings serialization test includes nlohmann/json directly.
target_link_libraries(se_functional_tests PRIVATE nlohmann_json::nlohmann_json)

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
