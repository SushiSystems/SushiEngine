# Audio

This file covers the from-scratch game-audio engine: the two-plane control/render split, the
device seams, the real-time DSP core, the mixer and voice model, propagation and spatialization,
reverb, the ECS bridge, the asset pipeline, and the editor's authoring and telemetry surfaces.

## 1. Audio (from-scratch AAA game-audio, Phase S0)

Design: `docs/design/audio_system.md`. A from-scratch, middleware-free game-audio engine, placed
like skeletal animation and cosmetic VFX — a **wall-clock snapshot consumer that lives outside
the deterministic sim island**, so a run is byte-identical with audio on or off. Its shaping
decision is the split every shipping AAA runtime is built on: a **control plane** (the game/ECS
thread, which allocates freely and only publishes intent) and an **audio-render plane** (a
high-priority callback thread that mixes under hard real-time discipline — no heap allocation, no
locks, no syscalls). The real-time mix runs on the device callback and deliberately **sidesteps
SushiRuntime**, which is a block-until-quiescent throughput engine with no real-time thread
class; the runtime enters only through an optional, deferred GPU batch-DSP seam.

Phase S0 stands up the render-plane boundary end to end with a trivial renderer, before any DSP
exists — the same "seams first, then fill" discipline the render and input stacks used.

**The header-only seams (`engine/domain/audio/include/SushiEngine/audio/`)** carry zero SDL, zero
SYCL, and no runtime link:

- `device.hpp` — the device I/O seam. `AudioStreamFormat` is the negotiated stream shape (sample
  rate, channel count, power-of-two block size); a backend may return a format other than the one
  requested, so a consumer always reads the obtained format back. `IAudioRenderer` is the
  real-time render sink:
  `render(float* const* channels, int channel_count, int frame_count) noexcept`, called once per
  block on the audio thread with **planar** (deinterleaved) buffers — the discipline the DSP core
  keeps throughout, interleaving only at the device boundary. `IAudioDevice`
  (open/close/is_running/format) isolates the one unstable, platform-specific dependency so the
  whole mix is testable against a trivial renderer and the backend is swappable without touching
  a line of DSP.

- `accelerator.hpp` — `IDSPAccelerator`, the optional GPU batch-DSP seam, declared now and
  implemented later (S10). It carries a single `available()` query so a subsystem that could
  offload long convolution / HRTF / ambisonic decode asks first and falls back to its CPU path —
  which is every build today. Kept intentionally thin because the runtime's fluent API is
  unstable; the batch-submit surface lands with the real implementation.

**The compiled backend (`engine/domain/audio/source/`, `sushiengine_audio_backend` STATIC).** The
mirror of `sushiengine_input_backend`: a plain STATIC library linking SDL2 and nothing else — no
SYCL, no runtime — so it builds on a stock toolchain and never touches the one-way
`SushiEngine → SushiRuntime` arrow. It carries the sole SDL-aware audio component,
`Audio::SDLAudioDevice` (`engine/domain/audio/source/backend/sdl/sdl_audio_device.*`), which
opens an `SDL_AudioDevice` and drives an `IAudioRenderer` once per block on SDL's callback
thread: the planar scratch and channel-pointer table are allocated once in `open` (never in the
callback), the renderer fills them, and the device interleaves the result into SDL's output
buffer. The OS handle crosses its header as an opaque `std::uint32_t`, so SDL never leaks into a
consumer translation unit.

A second backend, `Audio::MaAudioDevice`
(`engine/domain/audio/source/backend/miniaudio/ma_audio_device.*`), implements the same
`IAudioDevice` seam over **miniaudio**'s native low-latency APIs
(WASAPI/CoreAudio/ALSA/PulseAudio/…) as a drop-in alternative — one vendored public-domain
header, tighter buffer control, and the same allocate-in-`open`/interleave-in-callback
discipline; miniaudio's types stay behind a pimpl so they never reach a consumer. The mix path is
identical either way — only the device I/O differs. `samples/audio/audio_miniaudio_demo.cpp`
opens a real device end to end.

**The `App::runtime()` seam.** `Loop::App` now exposes its owned or borrowed runtime — the one
handle the later GPU accelerator path needs to allocate USM. It does not weaken the one-way
dependency: the App still owns the lifetime, and a borrowed runtime is returned, never destroyed.
Gameplay never needs it; the loop still hides the runtime behind `world()`, `commands()`, and
`system()`.

**Layering & build.** `SUSHIENGINE_BUILD_AUDIO` gates the compiled backend (OFF by default). The
`samples/audio/audio_demo.cpp` example is the S0 vertical slice: a headless, self-checking
software block loop (pump N blocks through a silence renderer, assert every sample is silent and
the renderer ran), then a best-effort real device open that is a clean no-op on a headless host.
`se audio` builds and runs it (configuring `SUSHIENGINE_BUILD_AUDIO=ON`, exactly as `se render`
does for the renderer probe).

**The DSP core (`engine/domain/audio/include/SushiEngine/audio/dsp/`, Phase S1).** The portable,
real-time-safe C++17 layer beneath the seams — no SDL, no SYCL, no runtime — that every later
mix, filter, and spatializer is built on. It has no build option: like the input action layer it
rides the SushiEngine INTERFACE target and is exercised headlessly by `Unit_Audio` tests.

- **Real-time primitives.** `ScopedNoDenormals` (`denormals.hpp`) is the RAII guard that sets
  flush-to-zero + denormals-are-zero on the FPU for the span of a callback — the fix for the slow
  subnormal arithmetic a decaying IIR or reverb tail would otherwise fall into; it restores the
  exact prior control word, and being outside the deterministic island the bit-level change is
  harmless. `SpscRing<T>` (`spsc_ring.hpp`) is the one queue of the two-plane model: a
  power-of-two, wait-free single-producer/single-consumer ring with its two indices on separate
  cache lines and acquire/release publication — the control thread pushes command records, the
  audio thread drains them, neither ever locks.

- **SIMD kernels (`simd.hpp`).** The per-block hot loops — `apply_gain`, `apply_gain_ramp` (the
  zipper-free level change), `mix_accumulate`, `copy_scaled`, `fill`, and constant-power
  `equal_power_pan` — as a 4-wide SSE path with a scalar remainder tail on x86 and a scalar
  fallback elsewhere. Applied to *summed* bus buffers, so they cost O(bus), not O(voice).

- **Filters (`dsp/filters/`).** `OnePole` (the cheapest smoother / DC block / parameter ramp),
  `Biquad` (the RBJ cookbook set — low/high-pass, band-pass, notch, peaking, shelves — in the
  numerically well-behaved Transposed Direct Form II), and `StateVariableFilter` (Andrew Simper's
  Cytomic TPT-SVF, the modulation-stable default that yields every mode from one pair of
  integrator states). Coefficients are designed in `double` off-thread and stored `float`, the
  audio-path rule even though the engine's `Scalar` is double.

- **Block graph (`graph.hpp`, `nodes.hpp`).** `BlockGraph` is a DAG of `Node` processors
  linearized by a Kahn topological sort computed off the audio thread; `process()` pulls one
  fixed block through that order and never allocates. **Feedback** is a connection flag, not a
  special node: a feedback edge is excluded from the ordering and, because each output buffer
  persists between blocks, the consumer reads the producer's previous-block output — a one-block
  z⁻¹, which is exactly what lets a comb or reverb loop be computed in a single forward pass.
  Built-in nodes are `SineNode`, `GainNode` (per-block ramp), `MixNode`, and `BiquadNode`. The
  `samples/audio/audio_dsp_demo.cpp` slice wires sine → mix → low-pass → gain and self-checks
  that the settled RMS proves the high tone was filtered out; `Unit_Audio`
  (`tests/unit/test_audio_dsp.cpp`) pins the ring, denormal flush, every filter, the SIMD
  kernels, the pan law, the topological order, and the one-block feedback exactly.

**The action layer (`engine/domain/audio/include/SushiEngine/audio/`, Phase S2).** The
header-only game glue over the DSP core — a prioritized multi-source mix — reached through the
`audio/audio.hpp` umbrella (now in `SushiEngine.hpp`). Like the DSP core it has no build option
and is exercised headlessly by `Unit_Audio`.

- **Parameters (`parameter.hpp`).** Every runtime mix change crosses the thread boundary the same
  way: `SmoothedValue` holds an atomic **target** the control thread publishes and the audio
  thread **slews toward** at a configured rate, bracketing each block as `[start, end]` for a
  click-free gain ramp — never a raw jump. `Rtpc` layers the "game variable → authored curve →
  target" mapping on top, evaluating the clamped piecewise-linear `RtpcCurve` on the control
  thread so the audio thread only ever ramps.

- **Voices (`voice.hpp`).** A `VoiceSource` renders mono and also offers a cheap `advance` (skip
  forward without producing output) — the path a *virtualized* voice takes to keep its play
  position current for ~free. `VoiceDescriptor` carries gain, priority, bus, pan, and a linear
  distance attenuation that reaches true silence, which is what makes it a clean culling signal.
  `ToneSource` and `BufferSource` are the S2 sources; decode/streaming is S8.

- **Mixer (`mixer.hpp`).** Voices sum into stereo **buses**, not into each other, so an effect
  runs once on a summed bus buffer (O(bus)) instead of per voice. A `Bus` has a series **insert
  chain** (`IBusEffect`), a post-fader `SmoothedValue` gain, an **output** route into a parent,
  and **aux sends** that copy its signal at a level into a parallel bus (the reverb-send
  pattern). `MixerGraph` orders buses by a topological sort over the routing and send edges so
  every contributor precedes its consumer, with the master rendered last. Buses are stereo now;
  the ambisonic scene bus of the design doc's §4 replaces that path at S4 without changing the
  routing/insert/send structure.

- **Voice manager + engine (`voice_manager.hpp`, `engine.hpp`).** `VoiceManager` holds a fixed
  pool and, each block, computes an effective **audibility** (base gain × attenuation) per active
  voice, ranks the set by `(priority, audibility)`, promotes the top **real** voices up to a cap
  to full rendering, and leaves the rest **virtual** (position bookkeeping only) — so hundreds of
  possible sounds collapse to a bounded render set that pans into a handful of bus buffers. Real
  voices ramp their gain and free themselves when a one-shot ends.

  `AudioEngine` is the `IAudioRenderer` the device drives: it sets the denormal guard, clears the
  mixer, folds the voice manager's real voices into the buses, runs the bus graph, and fans the
  stereo master out to the device channels. `samples/audio/audio_mixer_demo.cpp` starts 26 voices
  against a cap of 6 and self-checks the cap and the out-of-earshot virtualization; the
  `Unit_Audio` suite (`tests/unit/test_audio_mixer.cpp`) pins the smoother, RTPC, aux-send
  arithmetic, post-fader gain, bus order, the real/virtual cap, distance culling, pan centring,
  and one-shot lifetime. Occlusion, the HDR window, a separate decode cap, and voice stealing
  layer onto this ranking in later phases.

**Propagation (`propagation.hpp` + `dsp/fractional_delay.hpp`, `dsp/air_absorption.hpp`, Phase
S3).** A source's whole travel through the air is modelled as **one variable fractional delay
line of length distance/c**, and the Doppler falls out for free: the read pointer sits that far
behind the write pointer, so when the distance changes between blocks the read rate stops being
one — and a read rate other than one *is* a pitch shift. This is why there is no velocity term
anywhere; motion is implicit in the frame-to-frame distance change. `FractionalDelayLine` does
the read with a 4-point cubic-Lagrange (Farrow) kernel (non-recursive, no ringing, clean to
modulate).

`SourcePropagation` wraps it: it slew-limits the delay so a source cannot appear to break the
sound barrier (`|v_radial| < 0.9·c`), **snaps** the delay on a teleport instead of sweeping it (a
sweep would fire a synthetic Doppler screech), then dulls the block with a distance-driven
air-absorption low-pass and scales it by the distance gain. The air model is the full ISO 9613-1
absorption (`air_absorption_db_per_meter` from temperature/humidity/pressure) reduced to a
one-pole corner that falls with distance (`air_absorption_cutoff`); the speed of sound feeds both
the delay and the absorption from the same temperature, so delay, Doppler, and dullness stay
consistent.

`DistanceModel` (Linear/Inverse/Exponent) and the shared `distance_attenuation` give the rolloff,
used by *both* the audibility ranking and the rendered gain so a voice is culled by the level it
is played at. The voice manager runs a `SourcePropagation` per spatial real voice;
`VoiceManager::set_voice_position` is how a moving emitter (or, at S6, the ECS snapshot) drives
the whole effect. `samples/audio/audio_propagation_demo.cpp` flies a tone past the listener
(pitch up approaching, down receding) and `tests/unit/test_audio_propagation.cpp` pins the
delay-line accuracy, the ISO absorption, the distance laws, the delay ≈ distance/c, the Doppler
direction, and the teleport snap.

`AudioEngine::render` clamps its internal work to the prepared maximum block and zero-fills any
surplus device samples, so an OS mixer that hands back a larger callback block than `prepare` was
told degrades to brief silence rather than overrunning a buffer.

**The spatializer (`spatializer.hpp` + `dsp/spherical_harmonics.hpp`, Phase S4).** The 3D
rendering core, and the reason the audio path scales: a source is placed by **encoding** it into
a shared **ambisonic scene bus** with its spherical-harmonic gains — `(order+1)²` gains, cheap,
per source — so any number of sources collapse into one fixed field. The harmonics are real AmbiX
(**ACN** ordering, **SN3D** normalisation, so W = 1 and the first-order channels are the
direction's y/z/x), evaluated from the associated-Legendre recurrence.

The field is **decoded once** to a fixed 26-point virtual-speaker layout and each speaker is
rendered to the two ears through an analytic head model — a Woodworth interaural time difference
(a fractional delay per ear) and a head-shadow low-pass on the far ear — so the number of ear
renders is constant no matter how many sources play. Head tracking is free: the voice manager
encodes each source in **head-relative** coordinates (`head_relative_direction` rotates a world
direction into the listener's `forward`/`up` frame), so a head turn re-aims every source with no
extra state. `AudioEngine` owns the spatializer, decodes it to binaural each block, and sums that
with the non-spatial stereo master; a spatial voice encodes into the scene bus instead of
stereo-panning when a spatializer is present, and the decode is skipped on a block with no
spatial source. `samples/audio/audio_spatial_demo.cpp` orbits a tone around the head (audibly
circling on headphones); `tests/unit/test_audio_spatial.cpp` pins the SH convention and
orthogonality, the encode/decode kernel, the left/right level cues, the ITD, front symmetry, and
head-tracking.

The analytic HRTF is self-contained (no measured data) and gives solid horizontal localisation
and externalisation. The **measured-HRTF** fidelity upgrade now slots in behind the same encode →
decode seam: `hrtf.hpp` defines a dependency-free `IHRTFDatabase` (an HRIR pair per head-relative
direction) and `HrirConvolver` (a per-ear direct-form FIR), and
`BinauralSpatializer::set_hrtf_database` switches each virtual speaker from the analytic ITD +
shadow model to a convolution through that direction's measured impulse response — capturing the
pinna/torso cues the analytic model approximates.

`sofa_hrtf.hpp`'s `SofaHRTFDatabase` (behind HDF5, off the umbrella like the SYCL accelerator)
loads a real SOFA `SimpleFreeFieldHRIR` file (`Data.IR` / `SourcePosition` /
`Data.SamplingRate`), maps each source position to a head-relative unit vector, resamples the
taps to the stream rate, and serves the nearest pair; `write_sofa` bakes the same datasets.
Passing `nullptr` restores the analytic path, so the upgrade is strictly additive.
`samples/audio/audio_sofa_demo.cpp` verifies it against the real MIT KEMAR set.

Above the per-speaker HRIR sits the **MagLS** decode (`magls.hpp`). `MaglsBinauralDecoder`
solves, once at configure time from a measured set, a pair of decode FIRs per ambisonic channel
by least squares — complex (exact ITD) below a cutoff, **magnitude** LS with phase continuation
above it to suppress the high-frequency coloration a finite order otherwise produces — and
applies them straight to the bus (`channels × 2` convolutions, independent of the measurement
count). `BinauralSpatializer::set_magls_decoder` selects it above both the per-speaker HRIR and
the analytic model. `AnthropometricHRTFDatabase` personalizes any `IHRTFDatabase` by warping the
impulse-response time axis to the listener's head size. `samples/audio/audio_magls_demo.cpp`.

Beyond the parametric FDN/convolution reverbs, **ray-traced room acoustics**
(`acoustic_raytracer.hpp`) *measures* reverberation from the geometry: `RayTracedAcoustics`
Monte-Carlo path-traces bounces (specular/diffuse by scattering coefficient, per-band
absorption), bins arrivals at a receiver sphere, and Schroeder-integrates the energy-time
histogram into **RT60 per band** plus a decaying-noise impulse response the `ConvolutionReverb`
can render (verified within 4 % of Sabine for a shoebox). `maekawa_diffraction_db` adds geometric
edge diffraction (least-detour path over the occluder silhouette → Maekawa insertion loss per
band). `samples/audio/audio_raytrace_demo.cpp`.

**The reverb (`dsp/feedback_matrix.hpp` + `dsp/fdn_reverb.hpp` + `reverb.hpp`, Phase S5).** The
diffuse late field, generated by an order-16 **Jot feedback delay network**: `N = 16` delay lines
are scattered into each other every round trip by a **lossless** mixing matrix (Householder
`I − (2/N)·11ᵀ`, or Walsh–Hadamard), and the *only* loss in the loop is a per-line one-pole
**damping filter**. That split is the whole point — the orthogonal matrix sets echo density while
the damping alone sets decay, so RT60 becomes a clean knob: each line's damping filter is solved
so its DC gain is the round-trip loss for the broadband RT60 and its Nyquist gain the loss for
the (usually shorter) HF RT60, giving frequency-dependent decay from one first-order filter.

Coprime **prime** line lengths keep the modes from piling into flutter, and a slow per-line
**delay modulation** (read through the cubic-Lagrange `FractionalDelayLine`) smears the survivors
— the standard cure for a metallic tail. Input **Schroeder-allpass diffusion** and a **predelay**
precede the network. Because the matrix is orthogonal and every damping filter is strictly
contractive, the FDN is unconditionally stable — it can neither ring forever nor blow up.

Above the DSP core, `reverb.hpp` is the game-facing layer: the **`IReverb`** seam (an FDN today,
a convolution reverb later, same seam), the **I3DL2 / EAX** parameter set with presets and the
I3DL2→FDN mapping, a **`ReverbBusEffect`** that adapts any `IReverb` onto a per-zone reverb **aux
bus** as an `IBusEffect` (the S2 aux-send machinery routes voices into it), and **Sabine/Eyring**
room-geometry RT60 (Sabine below ᾱ = 0.3, Eyring above) with a `shoebox_reverb` factory that
turns a room's dimensions and materials into an I3DL2 preset. Early reflections (I3DL2
Reflections/ReflectionsDelay) are the image-source model delivered at S7 (see below).
`samples/audio/audio_reverb_demo.cpp` plays a percussive pattern into the aux bus;
`tests/unit/test_audio_reverb.cpp` pins the matrix losslessness, FDN boundedness, coprime
lengths, a measured T30 that tracks the requested decay, the darker-tail-as-HF-decays-faster
behaviour, the predelay, and the Sabine/Eyring maths.

**The ECS bridge
(`engine/world/simulation/include/SushiEngine/simulation/components.hpp` +
`engine/domain/audio/include/SushiEngine/audio/audio_scene.hpp` +
`engine/world/simulation/include/SushiEngine/simulation/audio_extract.hpp`, Phase S6).** How the
world starts making sound. Three trivially-copyable components join the built-in set:
`AudioListener` (its Transform + Orientation are the ears), `AudioEmitter` (a `sound` id plus the
routing and attenuation a designer authors, pose from Transform like the renderer reads), and
`ReverbZone` (a world box with an inline I3DL2 parameter set). None is read or written by a
fixed-step Schedule system; instead a **read-only wall-clock extract** — the audio sibling of the
render `extract()` — walks those columns each frame, so, exactly like skinning and VFX, it sits
*outside* the deterministic island and a run is byte-identical with audio on or off.

The extract does the two things the audio module cannot (it lives above the runtime, so it may):
it converts absolute double `WorldVector3` positions to **listener-local float** with the
renderer's eye-subtracted-in-double-then-cast idiom (planet-scale precision preserved), and reads
the listener's Orientation quaternion into a facing frame — producing a plain-float
`SceneSnapshot`. That snapshot is reconciled against the live voice pool by `AudioScene`: start a
voice when an emitter appears, move and re-gain it while it persists (the frame-to-frame position
delta is the Doppler), stop it when the emitter goes silent or is destroyed, and steer the reverb
aux bus from the zone the listener stands in.

The deliberate seam is Dependency-Inversion — `AudioScene` knows nothing about the ECS (it is
plain float, listener-local, unit-testable), and resolves a `sound` id to a source through an
injected `IEmitterSourceFactory`, the seam the S8 bank/event system fills. `I3DL2Reverb` was
split into a dependency-free `reverb_params.hpp` so the `ReverbZone` component carries the data
without the reverb engine. `samples/audio/audio_scene_demo.cpp` reconciles a hand-built emitter
flyby; `tests/unit/test_audio_scene.cpp` pins the reconciliation and
`tests/integration/test_audio_ecs.cpp` the extract against a real ECS world (listener-local
conversion, zone containment, the read-only invariant).

**Occlusion, rooms + portals, and early reflections (`audio/acoustic_geometry.hpp` +
`acoustic_material.hpp` + `occlusion.hpp` + `portals.hpp` + `early_reflections.hpp`, Phase S7).**
The geometry-coupled layer: what is blocked, and what that sounds like. Sound is occluded by a
**dedicated acoustic BVH** — coarse triangles tagged with three-band `AcousticMaterial`s, not the
render mesh — built two-level like a ray-tracer: an `AcousticBlas` per mesh (built once) under an
`AcousticScene` top-level BVH over placed instances, so moving a rigid body is a transform plus a
cheap `refit`, never a BLAS rebuild.

It answers two questions: a single `line_of_sight` ray (accumulating each pierced surface's
transmission — walls pass lows more than highs, so through-wall sound is bassy for free) and a
`soft_occlusion` sphere sample (a deterministic ray fan → a smooth 0..1 fraction, so cover fades
instead of popping). `OcclusionFilter` renders the result from the design's **two scalars**:
*obstruction* (a pillar) muffles and attenuates the dry only, while *occlusion* (a wall) also
pulls the reverb send down — each slewed as an edge-diffraction coefficient.

Indoor propagation is the **rooms + portals** model (`PortalGraph`): a cross-room source is heard
through the doorways, each portal on a shortest path becoming a secondary virtual source at the
opening — cheap coupling and doorway diffraction without a wave solve. `ImageSourceModel` adds
first-order shoebox **early reflections** (the six wall images as delay/gain/direction taps),
rendered by `EarlyReflections` from a multi-tap delay line. The `VoiceManager` runs a per-voice
`OcclusionFilter` after propagation and sends the post-occlusion dry to a reverb aux bus scaled
by the occlusion wet-scale; new `Room`/`Portal` ECS components and an `AUDIO_EMITTER_OCCLUDED`
flag let the wall-clock extract soft-test each emitter against an `AcousticScene` and inject
doorway virtual emitters through a `PortalGraph` — still read-only, so a run stays
byte-identical. `samples/audio/audio_occlusion_demo.cpp` slides a tone behind a concrete wall;
`tests/unit/test_audio_occlusion.cpp` pins the BVH, transmission, soft fraction, the occlusion
DSP, the portal doorway, and the early reflections.

**The asset pipeline: bank, codecs, events, and streaming (`audio/codec.hpp` + `event.hpp` +
`bank.hpp` + `streaming.hpp`, Phase S8).** How authored audio gets into the engine, and how the
game plays it without ever naming a file. A **bank** is one compact, versioned binary — a media
table, the baked event/container definitions, and a blob of encoded sound — that `Bank` loads and
`BankBuilder` writes.

Each sound is stored in the codec that suits it behind the `IAudioCodec` seam: from-scratch
`PcmCodec` (16-bit/float) and `ImaAdpcmCodec` (a continuous 4-bit stream, ≈4:1, encoder and
decoder both here); **`OpusCodec`** (`opus_codec.hpp`, behind libopus) now adds compressed
music/dialogue (≈31:1) over the bank's own length-framed packet container. Because the
header-only bank cannot `new` a dependency-gated codec, `set_external_codec_factory` lets an
Opus-linked TU register it (`register_opus_codec` in one call) and `make_codec` then serves Opus
media transparently. **Vorbis** (`vorbis_codec.hpp`, behind libvorbis) is the same pattern; the
external-codec hook is a **registry** (`add_external_codec_factory`) so both coexist. Both
compressed codecs also **stream** from an `IDataSource` through the `StreamingDecoder`, and
`decode_ogg_opus` ingests external `.opus` files. The codec is a stateful forward decoder, so the
same code serves a resident one-shot (`decode_all`) and a chunked stream (byte-boundary carry).

The game posts an **event ID**; an `EventDatabase` of flattened
`Sound`/`Random`/`Sequence`/`Blend`/`Switch` containers resolves it to one media id (a
self-contained splitmix64 for the random variation) — the sound-designer indirection.
`BankSourceFactory` is the concrete `IEmitterSourceFactory` the S6 `AudioScene` was built to
accept: resolve → decode-once-cached → `BufferSource`. Long assets **stream**: a
`StreamingDecoder` (the lone producer) reads an `IDataSource` in chunks and pushes samples into
the S1 `SpscRing`, while a `StreamingSource` (the lone consumer, on the audio thread) only ever
pops — no I/O, decode, or allocation on the hot path; a `StreamingWorker` runs the pump on a
`std::thread` (tests pump synchronously, keeping the core deterministic).
`samples/audio/audio_bank_demo.cpp` fires footstep Random events over a streamed music bed;
`tests/unit/test_audio_bank.cpp` pins the codecs, the bank round-trip, container selection, the
factory, and the streaming path.

**The live-profiler telemetry channel (`audio/profiler.hpp` + mixer/engine metering, Phase S9,
part 1).** How the editor watches the audio thread without ever locking it. The audio thread
`publish`es a fixed-size POD `AudioProfileSnapshot` — real/virtual/active voice counts, master
peak/RMS, per-bus meters, a downsampled output scope, a monotonic block index — into an
`AudioProfiler` once per `render`; the GUI reads the latest whenever it repaints. The channel is
a single-latest-value **seqlock**: the writer (audio thread) is wait-free and the reader only
spins if it catches a publish mid-flight, so the hard-real-time rule holds (the design doc's §0
one-way telemetry).

The `MixerGraph` now records each bus's post-fader peak/RMS
(`bus_peak`/`bus_rms`), and `AudioEngine` gathers those plus the voice population and a scope of
the true output into the snapshot. This is the data half of the design doc's §11 editor
authoring; the ImGui panels that render it — the mixer/bus strip, the emitter inspector, the
reverb-zone editor, the profiler view — are the remaining half of S9.
`samples/audio/audio_profiler_demo.cpp` polls the channel live off a running device;
`tests/unit/test_audio_profiler.cpp` pins the seqlock, the engine gather, and the real/virtual
split.

**Editor audio authoring (simulation seam + `applications/editor/source/audio/`, Phase S9, part
2).** The editor half of the design doc's §11: a designer places an Audio Emitter, moves the
Scene camera, and hears it, with no game running. The sim seam gains editor-facing
`AudioEmitterParameters`/`ReverbZoneParameters`/`AudioListenerParameters` and the matching
`IWorldEditor` accessors, stored as host bookkeeping on the entity record like light/cloth (no
ECS migration, and no Audio dependency in the sim layer). The editor's `AudioEditorSystem` owns a
live `AudioEngine` + SDL device with a small mixer (Master/SFX/Music + a reverb aux bus running
the FDN) and, each frame, projects the world's emitters and the containing reverb zone into the
voice pool with the Scene camera as the listener — a default factory maps each `sound` id to a
distinct looping tone (the S8 bank plugs in behind the same seam later).

The UI is an Audio Mixer
window (faders + live `ImDrawList` meters), an Audio Profiler window (voice counts, meters,
output scope), and Inspector sections for the emitter (params + a live attenuation-curve plot + a
Play audition), the reverb zone (I3DL2 params + presets), and the listener — wired into the Add
Component and Window menus and the `.sushiscene` serializer. The editor now links
`sushiengine_audio_backend`.

**Procedural SFX, convolution reverb, and the GPU accelerator (`audio/dsp/` + `audio/`, Phase
S10).** The final phase, three independent pieces. **Procedural SFX**: modal synthesis
(`dsp/modal.hpp`) — a bank of two-pole resonators struck by an impulse, with material presets —
wrapped as a `ModalImpactSource` (a one-shot ring) and a `WindSource` (speed-driven filtered
noise + a Strouhal Aeolian tone), both plain `VoiceSource`s.

**Convolution reverb**: a from-scratch radix-2 FFT behind an `IFFT` seam (`dsp/fft.hpp`), a
uniformly-partitioned
overlap-save `PartitionedConvolver` (`dsp/convolution.hpp`), and a `ConvolutionReverb`
(`audio/convolution_reverb.hpp`) — an `IReverb` interchangeable with the FDN on the same aux bus,
synthesising its room impulse response from the I3DL2 parameters. **GPU accelerator**:
`SYCLDSPAccelerator` (`audio/accelerator_sycl.hpp`) is the concrete `IDSPAccelerator`, offloading
batch FIR convolution to the SushiRuntime SYCL device with k-block lookahead (async submit /
deferred collect over a USM slot ring) while the RT mix stays on the CPU — the only place the
runtime enters the audio subsystem, confined to that one SYCL-only header (kept off the umbrella)
and falling back to the CPU path when no device is present.

With S10 the audio roadmap is **complete**: S0–S4 (the critical path) plus S5 (reverb), S6 (ECS
integration), S7 (occlusion / rooms + portals / early reflections), S8 (asset pipeline), S9
(profiler telemetry + editor authoring), and S10 (procedural SFX + convolution reverb + GPU
accelerator) all landed.
