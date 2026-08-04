# Audio {#module-audio}

`audio` owns the from-scratch digital signal processing graph, the voice and mixer model built on
it, and the spatialisation, propagation, reverb and asset seams around that. Everything above the
output device is host C++ a consumer compiles into its own translation unit, so a headless target
can mix a block and assert on the samples without opening a stream.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `core` (public) — listener and emitter positions are the engine's own vector types.

The module itself is header-only and links nothing else, which is what lets it exist for every
configure. The one part that touches an operating system is built separately as
`sushiengine_audio_backend`, gated on `SUSHIENGINE_BUILD_AUDIO`: two `IAudioDevice`
implementations, one over SDL2's output stream and one over the vendored miniaudio backends,
each driving the mix on the device's own callback thread. It links `sushiengine_audio` publicly
and `SDL2::SDL2` privately — the device header keeps the operating-system handle as an opaque
integer, so SDL never leaks into a consumer's translation unit — plus `Threads::Threads`, the
dynamic loader and the maths library on non-Windows platforms.

## Public surface

`audio.hpp` is the umbrella a consumer includes, and `dsp/dsp.hpp` the umbrella for the portable
processing core underneath it. Headers are relative to `include/SushiEngine/audio/`.

| Group | Headers | Declares |
|---|---|---|
| Signal processing core | `dsp/` (seventeen headers, including `graph.hpp`, `nodes.hpp`, `filters/`, `fft.hpp`, `fdn_reverb.hpp`, `convolution.hpp`, `spsc_ring.hpp`, `denormals.hpp`, `simd.hpp`) | The block processing graph and its nodes, the filter set, the transform, the feedback-delay-network reverb, and the real-time primitives underneath all of it. |
| Voices and mixing | `engine.hpp`, `voice.hpp`, `voice_manager.hpp`, `voice_render_pool.hpp`, `mixer.hpp`, `mix_state.hpp`, `parameter.hpp`, `dynamics.hpp` | The render-plane entry point, the voice model and its pooling, the bus graph, and parameter smoothing. |
| Spatialisation | `spatializer.hpp`, `hrtf.hpp`, `sofa_hrtf.hpp`, `magls.hpp`, `channel_layout.hpp` | Panning and binaural rendering, measured head-related transfer functions, and the channel layouts a mix targets. |
| Propagation | `propagation.hpp`, `occlusion.hpp`, `portals.hpp`, `early_reflections.hpp`, `reverb.hpp`, `reverb_params.hpp`, `convolution_reverb.hpp`, `acoustic_*.hpp` | The environmental model: obstruction, portal-connected spaces, early reflections, late reverb, and the acoustic scene they are computed against. |
| Assets and events | `bank.hpp`, `event.hpp`, `codec.hpp`, `opus_codec.hpp`, `vorbis_codec.hpp`, `streaming.hpp`, `authoring.hpp`, `procedural.hpp` | Sound banks and events, the streaming decoders, the authoring project model, and procedural and modal synthesis. |
| Seams | `device.hpp`, `accelerator.hpp`, `accelerator_sycl.hpp`, `audio_scene.hpp`, `profiler.hpp` | The output device and accelerator interfaces, the scene a mix is described against, and the in-editor profiler. |

## Tests

Covered by the functional suite in `tests/`: fifteen `tests/unit/test_audio_*.cpp` files across
the processing core, mixing, voices, spatialisation, propagation, occlusion, reverb, convolution,
dynamics, banks, procedural synthesis, the profiler and the scene, plus
`tests/integration/test_audio_ecs.cpp` for the extract that carries the acoustic scene out of the
simulation.

Neither output device backend is tested: opening a stream needs an audio device, and the suite
runs headless. The mixing path they drive is covered without them.

## Further reading

- [`audio_system.md`](../../../docs/design/audio_system.md) — the umbrella design: the
  architecture and the seams, rather than every kernel.
- [`domain-audio.md`](../../../docs/architecture/domain-audio.md) — the signal graph, spatialization
  and propagation.
