# audio — one demo per phase of the audio system, each self-checked headlessly first and then
# played best-effort through a real device. The whole group needs sushiengine_audio_backend, the
# compiled device backend engine/domain/audio declares behind SUSHIENGINE_BUILD_AUDIO, so a configure
# without that option declares none of it rather than declaring targets that cannot link.
if(NOT TARGET sushiengine_audio_backend)
    return()
endif()

# audio_demo is the S0 silent-block vertical slice.
sushiengine_add_sample(audio_demo audio_demo.cpp)
target_link_libraries(audio_demo PRIVATE sushiengine_audio_backend)

# audio_dsp_demo, the S1 slice: generate → mix → filter a sine through the
# header-only DSP core's block graph.
sushiengine_add_sample(audio_dsp_demo audio_dsp_demo.cpp)
target_link_libraries(audio_dsp_demo PRIVATE sushiengine_audio_backend)

# audio_mixer_demo, the S2 slice: a prioritized multi-source mix through the voice
# manager (virtual/real cap, distance culling) and the mixer bus DAG (inserts, aux
# sends, RTPC-ramped gain).
sushiengine_add_sample(audio_mixer_demo audio_mixer_demo.cpp)
target_link_libraries(audio_mixer_demo PRIVATE sushiengine_audio_backend)

# audio_propagation_demo, the S3 slice: a flyby Doppler through the fractional-delay
# propagation model (delay/Doppler, air absorption, distance gain), self-checked
# headless (pitch up approaching, down receding).
sushiengine_add_sample(audio_propagation_demo audio_propagation_demo.cpp)
target_link_libraries(audio_propagation_demo PRIVATE sushiengine_audio_backend)

# audio_spatial_demo, the S4 slice: a source orbiting the head in binaural 3D through
# the ambisonic scene bus + analytic HRTF decode, self-checked headless (left/right
# cues + head-tracking) then played best-effort (headphones recommended).
sushiengine_add_sample(audio_spatial_demo audio_spatial_demo.cpp)
target_link_libraries(audio_spatial_demo PRIVATE sushiengine_audio_backend)

# audio_reverb_demo, the S5 slice: the Jot FDN reverb on a per-zone aux bus (I3DL2
# params, Sabine/Eyring room RT60), self-checked headless (bounded/decaying tail,
# predelay, longer-decay-rings-longer, shoebox factory).
sushiengine_add_sample(audio_reverb_demo audio_reverb_demo.cpp)
target_link_libraries(audio_reverb_demo PRIVATE sushiengine_audio_backend)

# audio_scene_demo, the S6 slice: the ECS→audio control-plane bridge (AudioScene)
# reconciling per-frame world snapshots into live voices — an emitter flyby (Doppler +
# L/R), a second emitter appearing, and one being dropped. The real ECS read is covered
# by Integration_AudioECS.
sushiengine_add_sample(audio_scene_demo audio_scene_demo.cpp)
target_link_libraries(audio_scene_demo PRIVATE sushiengine_audio_backend)

# audio_occlusion_demo, the S7 slice: occlusion/obstruction over the acoustic BVH
# (BLAS ray + TLAS refit), three-band material transmission, soft multi-ray occlusion,
# the room/portal doorway model, and image-source early reflections — self-checked
# headless (including the integrated voice-manager path) then played best-effort (a
# tone sliding behind a concrete wall).
sushiengine_add_sample(audio_occlusion_demo audio_occlusion_demo.cpp)
target_link_libraries(audio_occlusion_demo PRIVATE sushiengine_audio_backend)

# audio_bank_demo, the S8 slice: the asset pipeline — a bank of IMA-ADPCM SFX + a PCM
# music track baked to a binary and loaded back, a Random footstep event resolved
# through the bank-backed IEmitterSourceFactory, and a streamed music bed fed by a
# background worker into the lock-free ring.
sushiengine_add_sample(audio_bank_demo audio_bank_demo.cpp)
target_link_libraries(audio_bank_demo PRIVATE sushiengine_audio_backend)

# audio_profiler_demo, the S9 slice: the audio→GUI live-profiler telemetry channel —
# the seqlock snapshot (voice counts, per-bus + master meters, scope) published by the
# audio thread and read lock-free by the "GUI".
sushiengine_add_sample(audio_profiler_demo audio_profiler_demo.cpp)
target_link_libraries(audio_profiler_demo PRIVATE sushiengine_audio_backend)

# audio_procedural_demo, the S10 slice: procedural SFX — modal-impact synthesis (a
# struck-object resonator bank) and wind (speed-driven filtered noise + Aeolian tone) as
# plain VoiceSources.
sushiengine_add_sample(audio_procedural_demo audio_procedural_demo.cpp)
target_link_libraries(audio_procedural_demo PRIVATE sushiengine_audio_backend)

# audio_convolution_demo, the S10 slice: the convolution reverb (radix-2 FFT +
# partitioned overlap-save) behind the IReverb seam on a per-zone aux bus, driven by a
# synthesised room impulse response.
sushiengine_add_sample(audio_convolution_demo audio_convolution_demo.cpp)
target_link_libraries(audio_convolution_demo PRIVATE sushiengine_audio_backend)

# audio_authoring_demo: the authoring project (DAW backend) — author a container graph,
# flatten it to the runtime event database, and bake a bank. Pure engine headers.
sushiengine_add_sample(audio_authoring_demo audio_authoring_demo.cpp)
target_link_libraries(audio_authoring_demo PRIVATE sushiengine_audio_backend)

# audio_raytrace_demo: Monte-Carlo ray-traced room acoustics (measured RT60 + baked IR)
# and Maekawa edge diffraction. Pure engine headers, no third-party dep.
sushiengine_add_sample(audio_raytrace_demo audio_raytrace_demo.cpp)
target_link_libraries(audio_raytrace_demo PRIVATE sushiengine_audio_backend)

# audio_accelerator_demo, the S10 slice: the SushiRuntime SYCL DSP accelerator — batch
# FIR convolution offloaded to the device with k-block lookahead, checked against a CPU
# reference. A SYCL translation unit (it includes accelerator_sycl.hpp + the runtime);
# it needs no sushiengine_audio_backend, only the runtime the SYCL executable macro already links.
sushiengine_add_sample(audio_accelerator_demo audio_accelerator_demo.cpp)

# audio_miniaudio_demo: the same engine through the miniaudio (native low-latency)
# IAudioDevice backend instead of SDL.
sushiengine_add_sample(audio_miniaudio_demo audio_miniaudio_demo.cpp)
target_link_libraries(audio_miniaudio_demo PRIVATE sushiengine_audio_backend)

# audio_opus_demo: the Opus codec (compressed music/dialogue) behind the IAudioCodec
# seam — a full encode→decode round-trip self-check. Guarded by the presence of the
# vcpkg Opus package; it links core libopus (Opus::opus) and defines SE_AUDIO_OPUS.
find_package(Opus CONFIG QUIET)
find_package(OpusFile CONFIG QUIET)
if(Opus_FOUND AND OpusFile_FOUND)
    sushiengine_add_sample(audio_opus_demo audio_opus_demo.cpp)
    target_link_libraries(audio_opus_demo
        PRIVATE sushiengine_audio_backend Opus::opus OpusFile::opusfile)
    target_compile_definitions(audio_opus_demo PRIVATE SE_AUDIO_OPUS=1)
else()
    message(STATUS "Opus package not found — audio_opus_demo disabled (declare opusfile via ss)")
endif()

# audio_vorbis_demo: the Vorbis codec behind the IAudioCodec seam — encode→decode round-trip.
# Guarded by the vcpkg Vorbis package; links libvorbis (Vorbis::vorbis + Vorbis::vorbisenc).
find_package(Vorbis CONFIG QUIET)
if(Vorbis_FOUND)
    sushiengine_add_sample(audio_vorbis_demo audio_vorbis_demo.cpp)
    target_link_libraries(audio_vorbis_demo
        PRIVATE sushiengine_audio_backend Vorbis::vorbis Vorbis::vorbisenc)
    target_compile_definitions(audio_vorbis_demo PRIVATE SE_AUDIO_VORBIS=1)
else()
    message(STATUS "Vorbis package not found — audio_vorbis_demo disabled (declare libvorbis via ss)")
endif()

# audio_stream_compressed_demo: streaming Opus + Vorbis from a disk-like IDataSource through
# the StreamingDecoder pipeline. Needs both compressed codecs.
if(Opus_FOUND AND OpusFile_FOUND AND Vorbis_FOUND)
    sushiengine_add_sample(audio_stream_compressed_demo audio_stream_compressed_demo.cpp)
    target_link_libraries(audio_stream_compressed_demo
        PRIVATE sushiengine_audio_backend
                Opus::opus OpusFile::opusfile Vorbis::vorbis Vorbis::vorbisenc)
    target_compile_definitions(audio_stream_compressed_demo PRIVATE SE_AUDIO_OPUS=1 SE_AUDIO_VORBIS=1)
endif()

# audio_sofa_demo: the SOFA/HDF5 measured-HRTF path — loads a real HRIR set and drives the
# binaural spatializer through it. Guarded by the presence of the vcpkg HDF5 package; it
# links HDF5 (C API) and defines SE_AUDIO_SOFA.
find_package(hdf5 CONFIG QUIET)
if(hdf5_FOUND OR TARGET hdf5::hdf5-shared OR TARGET hdf5::hdf5-static)
    sushiengine_add_sample(audio_sofa_demo audio_sofa_demo.cpp)
    if(TARGET hdf5::hdf5-shared)
        target_link_libraries(audio_sofa_demo PRIVATE sushiengine_audio_backend hdf5::hdf5-shared)
    else()
        target_link_libraries(audio_sofa_demo PRIVATE sushiengine_audio_backend hdf5::hdf5-static)
    endif()
    target_compile_definitions(audio_sofa_demo PRIVATE SE_AUDIO_SOFA=1)
    # audio_magls_demo: MagLS ambisonic→binaural decode + anthropometric HRTF, driven from a
    # real SOFA set. Same HDF5 guard as the SOFA demo.
    sushiengine_add_sample(audio_magls_demo audio_magls_demo.cpp)
    if(TARGET hdf5::hdf5-shared)
        target_link_libraries(audio_magls_demo PRIVATE sushiengine_audio_backend hdf5::hdf5-shared)
    else()
        target_link_libraries(audio_magls_demo PRIVATE sushiengine_audio_backend hdf5::hdf5-static)
    endif()
    target_compile_definitions(audio_magls_demo PRIVATE SE_AUDIO_SOFA=1)
else()
    message(STATUS "HDF5 package not found — audio_sofa_demo disabled (declare hdf5 via ss)")
endif()
