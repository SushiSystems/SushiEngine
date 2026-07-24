# Audio System — AAA game-audio engine, from-scratch native DSP (SushiEngine::Audio)

This document is the **umbrella** for SushiEngine's AAA game-audio system: the product vision (a
Wwise/FMOD-class runtime built from scratch — no middleware dependency), the real-time DSP core, the
3D spatializer, the propagation/occlusion/reverb model, voice management, the ECS integration, and the
editor authoring surface. The heavy per-sample math lives in the portable CPU DSP core
(`include/SushiEngine/audio/dsp/`); this doc specifies the **architecture and the seams**, not every
kernel's source.

Companion docs: `docs/design/vfx_particle_system.md` (the sibling wall-clock snapshot consumer that also
lives *outside* the deterministic island), `docs/design/animation_system.md` (the structural template —
trivially-copyable state column + snapshot extract), and `docs/guides/ARCHITECTURE.md`.

**Status:** Design; **phases S0–S4 landed — the S0–S4 critical path is complete** (device seam + silent
block loop; DSP core base; voice manager + mixer bus DAG + RTPC; propagation/Doppler/air-absorption;
ambisonic scene bus + binaural spatializer with head-tracking — see §14). In the tree: the header-only seams and the `sushi_audio`
SDL2 backend; the `App::runtime()` accessor; the portable DSP core (`audio/dsp/`: `ScopedNoDenormals`,
`SpscRing`, SIMD kernels, one-pole/biquad/TPT-SVF filters, the topo-sorted block graph with one-block
feedback, built-in nodes); and the header-only action layer (`audio/`: `SmoothedValue`/`Rtpc`, voices
+ sources, the stereo mixer bus DAG with inserts/aux-sends/topo order, the voice manager's virtual/real
prioritized mix, and the `AudioEngine` renderer), reached through `audio/audio.hpp` (in the
`SushiEngine.hpp` umbrella). Plus the propagation model (`audio/dsp/fractional_delay.hpp` + `air_absorption.hpp`,
`audio/propagation.hpp`: Farrow fractional delay, ISO 9613-1 air absorption, distance models, and
`SourcePropagation` — delay-line Doppler with slew/teleport-snap — integrated per spatial voice).
Plus the spatializer (`audio/dsp/spherical_harmonics.hpp`, `audio/spatializer.hpp`: real ACN/SN3D
spherical harmonics, the ambisonic scene bus, and the virtual-speaker + analytic-HRTF binaural decode
with head-relative encoding, integrated into the voice manager and `AudioEngine`). Slices
`audio_demo`/`audio_dsp_demo`/`audio_mixer_demo`/`audio_propagation_demo`/`audio_spatial_demo`; 38
`Unit_Audio` tests. Everything from S5 (FDN reverb + I3DL2 + per-zone aux) onward remains design. The
S5+ phases follow this doc directly.

**Scope note (locked 2026-07-24):** this is a *game-audio* engine only. The earlier ambition to also ship
a standalone JCM800 amp simulator, VST3/CLAP plugins, a separate reusable **SushiDSP** repository, and
aero/thermo synthesis *products* is **dropped**. Because there is no longer a "ship a VST without the SYCL
toolchain" requirement, the DSP core stays an **internal engine module** (`audio/dsp/`), not a separate
layer/repo. Lightweight procedural SFX (modal impact synthesis, wind) survives only as an optional
in-engine feature (phase S10), never as a product.

---

## Implementation status — where we left off (2026-07-24)

The **S0–S4 critical path is complete and verified**: a working, audible AAA audio core —
device I/O → real-time DSP → prioritized multi-source mixer → Doppler/air propagation →
head-tracked binaural 3D. 38 `Unit_Audio` tests pass; five demos build and run (and were heard
playing through a real device). Everything below S4 in the table is still design.

| Phase | Status | Delivered | Key files | Demo · verifying tests |
|---|---|---|---|---|
| **S0** | ✅ done | Device seam + silent block loop; `App::runtime()`; `SE_BUILD_AUDIO`; `sushi_audio` SDL2 lib | `audio/device.hpp`, `audio/accelerator.hpp`, `audio/sdl/sdl_audio_device.*` | `audio_demo` |
| **S1** | ✅ done | DSP core: denormal guard, SPSC ring, SIMD kernels, one-pole/biquad/TPT-SVF, topo-sorted block graph (one-block feedback) + nodes | `audio/dsp/*` (`denormals`, `spsc_ring`, `simd`, `filters/*`, `graph`, `nodes`) | `audio_dsp_demo` · `test_audio_dsp.cpp` |
| **S2** | ✅ done | RTPC/smoothing, voices+sources, stereo mixer bus DAG (inserts/aux/topo), voice manager (virtual/real cap, audibility sort), `AudioEngine` | `audio/parameter.hpp`, `voice.hpp`, `mixer.hpp`, `voice_manager.hpp`, `engine.hpp` | `audio_mixer_demo` · `test_audio_mixer.cpp` |
| **S3** | ✅ done | Propagation: Farrow fractional delay, ISO 9613-1 air absorption, distance models, `SourcePropagation` (delay-line Doppler, slew/teleport-snap) | `audio/dsp/fractional_delay.hpp`, `dsp/air_absorption.hpp`, `audio/propagation.hpp` | `audio_propagation_demo` · `test_audio_propagation.cpp` |
| **S4** | ✅ done | Ambisonic scene bus (ACN/SN3D SH encode) + virtual-speaker + analytic-HRTF binaural decode; head-tracking via head-relative encode | `audio/dsp/spherical_harmonics.hpp`, `audio/spatializer.hpp` | `audio_spatial_demo` · `test_audio_spatial.cpp` |
| **S5** | ▶ **next** | FDN reverb (Householder/Hadamard, coprime delays, per-line damping) + I3DL2 API + per-zone aux bus + Sabine/Eyring RT60 | — | — |
| S6–S10 | design | ECS components; occlusion/rooms/portals+BVH; asset/bank/streaming; editor; procedural SFX + GPU `IDspAccelerator` | — | — |

**The next concrete step (S5).** Replace the placeholder low-pass insert on the reverb aux
bus (currently in `audio_mixer_demo`) with a real **FDN**: N prime-power coprime delay lines,
a lossless feedback matrix, per-line damping for frequency-dependent RT60, the **I3DL2**
parameter set as the public API, and Sabine/Eyring RT60 from geometry. The S2 aux-send
machinery is already in place to route voices into it. See §3.7 and §7.

**Consciously deferred (intentional gaps, not omissions to "fix").**
- **Batched command ring** — the control thread cannot yet start/stop voices mid-stream; S2+
  parameters cross via atomics and voices are set up before the device opens. The `SpscRing`
  (S1) is built and waiting; wiring the play/stop/event protocol through it is a later phase.
- **Voice stealing** — `VoiceManager::play` returns `INVALID_VOICE` when the pool is full
  rather than evicting the least-audible voice.
- **HRTF fidelity** — the S4 spatializer uses a self-contained *analytic* head model (Woodworth
  ITD + head-shadow): solid left/right localisation and externalisation, but weak elevation and
  front/back (no pinna cues). A measured-HRTF / **SOFA + MagLS + UPOLS** decode is the upgrade,
  and slots in behind the same encode → decode seam without touching call sites. The design's
  Wigner-D *bus* rotation is likewise unnecessary for point sources (head-relative encode is
  exact and cheaper) and would only be needed for externally-authored ambisonic content.

**Build & run the audio subsystem.** `SE_BUILD_AUDIO` gates the compiled backend and the demos
(`se audio` builds+runs `audio_demo`; the other demos build under the same flag). Tests are
`Unit_Audio` in the functional suite (`se build` then `se test`, or filter
`--gtest_filter='Unit_Audio.*'`). Coordinate frame across the spatial code: **x = front,
y = left, z = up** (right-handed); audio sample buffers are `float`, coefficients computed in
`double`. **Always prepare an `AudioEngine`/graph with `max_block ≥` any device callback block**
(the OS mixer here hands 480-frame blocks even when 256 is requested; `AudioEngine::render`
clamps defensively, but an under-sized prepare degrades to brief silence).

---

## §0 The one decision that shapes everything — two planes, one queue

Every shipping AAA audio runtime (Wwise, FMOD, Unreal, CRIWARE) is built on a single invariant, and so
is this one:

> **A strict split between a control plane and an audio-render plane, joined by exactly one lock-free
> queue.** The control plane (game/ECS thread) runs at a variable rate, allocates freely, and only ever
> publishes *intent* — play, stop, set-parameter — into a power-of-two **SPSC ring buffer**. The
> audio-render plane (one high-priority thread woken by the device callback) drains that ring at the top
> of each fixed-size block and does the real-time mix under hard real-time discipline: **no heap
> alloc, no locks, no syscalls, no file I/O, no exceptions** on the audio thread, ever.

```
   ECS / game thread (control plane)                 audio callback thread (render plane)
   ─────────────────────────────────                 ───────────────────────────────────
   world snapshot extract (wall-clock)                drain SPSC command ring
   voice manager: audibility sort, cull      ──ring──▶ apply targets, ramp params
   post events, set RTPC targets                       walk bus DAG (post-order), block render
   allocate / stream / decode (worker thread)          fractional-delay + spatialize + reverb
                                             ◀─ring─── meters / waveform snapshots
                                                        FTZ+DAZ set for the whole callback
```

**Why the runtime is sidestepped.** SushiRuntime is a compile-once/replay, block-until-quiescent,
single-flight throughput engine (see `sushiruntime-realtime-gaps` memory / audit): `run()` blocks the
caller, `execute()` is non-reentrant so no external callback thread can drive it, there is no RT thread
class, and per-step submit takes a mutex and may allocate. The real-time audio mix therefore runs
**single-thread on the device callback and does not touch the runtime scheduler at all.** The runtime is
used only for *optional, non-blocking, k-block-lookahead batch DSP* on the GPU (§12.2), behind a thin
`IDspAccelerator` seam. This is the same architectural placement as skinning and cosmetic VFX: a
**wall-clock snapshot consumer outside the deterministic sim island** (the fourth such consumer alongside
render/net/vfx per the SushiLoop design).

**Invariant:** a deterministic run is byte-identical with audio on or off. Audio never writes sim state;
it only reads a wall-clock snapshot of emitter/listener transforms.

---

## §1 Why none exists today — audit

1. **Greenfield.** No `include/SushiEngine/audio/`, no `audio/` backend, no audio component, no device
   callback, no DSP. The `input/` module is the exact structural template to mirror (§2, §12).
2. **Runtime has zero DSP math and real-time gaps.** No FFT/convolution/filters/BLAS; no RT-safe generic
   ring buffer; blocking executor. Strong and reusable: the typed USM data plane (`Buffer<T>`,
   `Residency::Shared|Device`, `dynamic_graph()` streaming add/remove) — a direct fit for GPU batch DSP
   and for voice/effect add-remove *if and when* the accelerator path is implemented.
3. **`App` does not expose the runtime.** `include/SushiEngine/loop/app.hpp` keeps `runtime_` private with
   no accessor. Audio needs `Runtime&` to allocate USM buffers for the (later) accelerator path — the S0
   prerequisite adds `App::runtime()`.
4. **`Scalar` is double.** Correct for the control/graph layer, but audio **sample buffers are `float`**
   (SIMD width, cache); coefficient computation stays `double`, results stored as `float` (§3).

---

## §2 Layering

Internal engine module — **not** a separate repo. Three tiers plus optional accelerator:

```
   SushiRuntime  ── optional, GPU batch DSP only, behind IDspAccelerator (non-blocking, k-block lookahead)
        ▲
   include/SushiEngine/audio/dsp/     ── portable CPU C++17 DSP core, NO SYCL
        │   block graph · SPSC/MPSC ring · ScopedNoDenormals(FTZ/DAZ)
        │   biquad(RBJ) · TPT-SVF(Cytomic) · one-pole · oversampling/ADAA
        │   UPOLS + Gardner NUPC convolution (IFFT behind seam) · Farrow fractional delay
        │   FDN reverb · ambisonic encode/rotate(Wigner-D)/MagLS decode · HRTF/SOFA loader
        ▲
   include/SushiEngine/audio/         ── ECS / game glue (header-only action layer)
        │   components · snapshot extract · voice manager · bus/mixer DAG
        │   RTPC/State/Switch · event system · bank loading
        ▲
   audio/  → sushi_audio (STATIC lib)  ── compiled backend: SDL2 device I/O, codec decode
```

Mirrors `input/` exactly: a plain `STATIC` library that links **SDL2 and nothing else** (no SYCL, no
sushiruntime) so it builds on a stock toolchain and never touches the one-way SushiEngine→SushiRuntime
layering. The header-only action layer above it is picked up through the SushiEngine INTERFACE target.
The `dsp/` core is header-mostly portable C++17; the accelerator is the only optional SushiRuntime seam.

Build: `SE_BUILD_AUDIO` option (mirror `SE_BUILD_INPUT`), `audio/CMakeLists.txt` → `sushi_audio`, examples
via `add_sushi_sycl_executable`, tests `TEST(Unit_Audio, …)`. Build with the `se` CLI.

---

## §3 The DSP core (`audio/dsp/`)

Portable, real-time-safe, SIMD-batched. All state pre-allocated in `prepare()`; `process()` is
alloc/lock/syscall-free.

**§3.1 Block graph.** A DAG of processor nodes with typed ports; the audio thread pulls one fixed block
(256–512 samples, power-of-two, SIMD-aligned) through a linearized schedule computed **off-thread** by
topological sort (Kahn) and swapped in atomically. Feedback (reverb, comb) is expressed by an explicit
one-block/one-sample **z⁻¹ delay node** that breaks the cycle. A buffer-pool pass lets non-overlapping
edges share physical buffers. Sub-block splitting at parameter-change boundaries gives sample-accurate
automation. `Node::prepare/process/reset`; `process` is `noexcept`.

**§3.2 Real-time safety.** Power-of-two **SPSC ring** (two `alignas(64)` atomic indices, acquire/release
publish) is the workhorse: control→audio pushes *pointers to pre-built immutable data*, never large data;
audio→GUI pushes meters; audio→worker pushes retired pointers for off-thread free (a "garbage" ring — the
audio thread never `delete`s). MPSC only where topology forces it (CAS write index). **Denormals:**
`ScopedNoDenormals` sets MXCSR FTZ+DAZ (x86) / FPCR FZ (ARM64) once at callback entry — mandatory the
moment any IIR/reverb tail exists.

**§3.3 SIMD & layout.** Deinterleave once at the input boundary, process **planar** throughout,
interleave once at output. Gain/pan/mix kernels via SSE/AVX/NEON (FMA accumulate); 32-byte-aligned
buffers, scalar remainder tails; ISA chosen by CPUID dispatch behind a `Vec4f/Vec8f` wrapper.
Polyphonic synth voices use SIMD-across-voices (SoA) where independent.

**§3.4 Filters.** Biquad via the **RBJ cookbook** (analog prototype → bilinear, Transposed Direct Form
II); **Cytomic TPT state-variable filter** (trapezoidal integrators, stable to very low cutoff, all
outputs at once) as the modulatable/robust default; one-pole for smoothers/DC-block/envelope followers.
Coefficients recomputed off-thread on parameter change, ramped/crossfaded on the audio thread.
Nonlinearities get ×2–×8 oversampling (polyphase/halfband) and/or **ADAA** anti-aliasing.

**§3.5 Convolution.** **UPOLS** (uniformly-partitioned overlap-save) at the callback block size for
HRTF/direct paths (latency = one block, filter spectra pre-transformed into a frequency-domain delay
line). **Gardner NUPC** (non-uniform: tiny head partitions + doubling tail, larger classes scheduled
across callbacks/background) for zero-latency long IRs — convolution reverb and BRIR tails. Real FFT
behind an `IFft` seam (pffft/KissFFT default; FFTW optional).

**§3.6 Fractional delay line.** One circular buffer per source; **cubic-Lagrange Farrow** interpolation
is the moving-source default (non-recursive, no ringing, clean modulation); allpass (Thiran) reserved for
magnitude-preserving feedback loops; windowed-sinc polyphase for hero/offline resampling. Drives
Doppler + propagation (§5).

**§3.7 Reverb primitive — FDN.** Order-N feedback delay network: N prime-power (coprime) delay lines, a
**lossless** feedback matrix (Householder — max echo density, 2N−1 adds; or Hadamard — no multiplies for
N=2^k), per-line **low-pass damping filter** for frequency-dependent RT60, Jot tonal-correction output
filter. Mixing and decay decoupled (damping after the lossless prototype). Public parameters exposed as
the **I3DL2** set (§7).

**§3.8 Spatial primitives.** Ambisonic **encode** (evaluate SH `Y_ℓ^m` at source direction — (L+1)²
gains, recomputed only on movement), **rotate** (per-degree Wigner-D block, Ivanic–Ruedenberg recursion),
**MagLS decode** (precomputed SH→binaural matrix, magnitude-only fit above ~1.5 kHz), **AllRAD** speaker
decode (dual-band rV/max-rE). **HRTF/SOFA loader:** import `SimpleFreeFieldHRIR`, normalize
coordinate/EQ conventions, decompose min-phase + ITD, Delaunay-triangulated barycentric interpolation.

---

## §4 Spatialization pipeline

The rendering core is an **ambisonic scene bus** (AmbiX = ACN/SN3D, order 3 = 16 channels, configurable
1–3). This is the single most important cost decision: per-source work is only distance attenuation +
SH-encode gains; **N sources collapse into a fixed 16-channel bus**, so the expensive spatialization
(rotation, HRTF decode) runs **once** regardless of source count — the reason Resonance/FB360/Wwise scale
to hundreds of sources.

```
   per source:  s(t) ─▶ [propagation §5] ─▶ SH encode (Y_ℓ^m gains) ─┐
                                                                      ├─▶ Σ  ambisonic bus (16ch)
   other sources ───────────────────────────────────────────────────┘        │
                                                       inverse-head-pose rotation (Wigner-D)
                                                                                │
                                     ┌──────────────────────────────────────────┴────────┐
                              MagLS SH→binaural (one HRTF for whole scene)        AllRAD speaker decode
                                     ▼                                                     ▼
                              stereo headphones + head-tracking                    5.1 / 7.1.4 beds
```

Generic HRTF (SADIE II KU100/KEMAR default) **paired with head-tracking** is the highest-leverage quality
win before any personalization; optional anthropometric ITD scaling and user SOFA import are later tiers.
LOD-tier the per-source path for speed: near = ambisonic-encode + high interp; far = cheap pan + Hermite.

---

## §5 Propagation — Doppler, delay, distance, air

**The key insight:** model per-source propagation as **one variable fractional delay line of length
distance/c**, and let Doppler fall out for free. Do **not** compute a radial-velocity ratio and resample.
The read pointer sits `distance/c · fs` behind the write pointer; as distance changes the read rate ≠ 1,
and *that is the Doppler* — physically exact, free propagation delay, correct curved-flyby bend, cheapest
(one fractional read/sample, no FFT/transcendentals), and it keeps the dry signal in sync with occlusion
and reverb sends.

Per-source signal chain:

```
   source ─▶ [Farrow cubic fractional delay, len = distance/c] ─▶ [air-absorption LPF, fc(d) ISO 9613-1]
          ─▶ [distance gain: inverse 1/r or authored curve] ─▶ SH encode → ambisonic bus
```

- **Interpolation:** cubic-Lagrange Farrow default; slew-limit Δ per block; **snap (don't sweep) on
  teleports** to avoid a synthetic Doppler screech; clamp |v_radial| < c (max-pitch safety).
- **Distance gain:** OpenAL-style models (`INVERSE` 1/r default = physical; `LINEAR` reaches true silence,
  which matters for voice culling; `EXPONENT`) plus author-defined curves per source (what shipping games
  use most). Controls: `doppler_scale` (0/1/>1), propagation-delay toggle, max-pitch clamp.
- **Air absorption:** ISO 9613-1 α(f, T, RH, P) → a distance-driven one-pole LPF cutoff, slew-limited;
  don't run full multi-band α per sample — precompute α at a few reference frequencies for current
  weather.
- **Speed of sound:** one engine constant, `c = 331.3 + 0.606·T` (°C); reuse T for both `c` and air
  absorption so delay, Doppler, and absorption stay consistent. Planet-scale listener geometry uses
  `WorldVector3`/floating-origin; `runtime.rebalancer(false)` kills migration jitter.

---

## §6 Occlusion, obstruction, rooms & portals

- **Obstruction** (direct blocked, reverb open — a pillar): degrade **dry only**. **Occlusion** (both
  blocked — a wall): degrade **dry + wet**. Two scalars per emitter, routed to separate DSP, mapped
  through author-defined Volume/LPF/HPF curves.
- **Acoustic BVH:** a simplified, dedicated acoustic collision mesh (or voxel proxy), **not** render
  geometry. Two-level: a static BLAS built once (high quality) + a dynamic TLAS of moving rigid instances
  (refit on transform / stable-topology deform, rebuild on topology change). Batch edits, single
  commit/frame, no query during commit. Embree-class backend or built-in.
- **Soft occlusion:** multi-ray / volumetric sampling (sample the source as a sphere, return fraction
  unoccluded) → smooth 0..1, not a single binary ray that pops.
- **Transmission:** per-material 3-band `absorption`/`scattering`/`transmission` (centres ≈ 400 Hz /
  2.5 kHz / 15 kHz); on a blocked path accumulate transmission across the first K surfaces → 3-band EQ
  (through-wall sound is bassy for free).
- **Rooms + Portals:** partition the world into rooms joined by portal volumes; a portal in an adjacent
  room becomes a **secondary virtual source at the doorway** whose direction/level track the opening —
  correct, cheap doorway diffraction and room coupling without a wave solve. This is the first indoor
  propagation target. Drive obstruction LPF/gain from an **edge-diffraction coefficient**, not raw
  ray-block percentage, to avoid popping.

---

## §7 Reverb

Two-stage, geometry-coupled:

- **Early reflections:** low-order **image-source method** from acoustic geometry (sparse, directional,
  first ~20–80 ms). Feed the FDN so the tail inherits room texture and reaches density faster.
- **Late reverb:** an **FDN** (§3.7). Derive per-band **RT60** from geometry — **Sabine** `0.161·V/A` when
  mean absorption ᾱ < 0.3, **Eyring** when ᾱ > 0.3 (Resonance's dual-formula-with-crossover-correction
  recipe), computed per octave band and fed into each line's damping-filter design.
- **Convolution reverb** (Gardner NUPC) reserved for signature static spaces; FDN wins for the dynamic
  late field.
- **Public API = I3DL2 parameter set** (Room/RoomHF, DecayTime, DecayHFRatio, Reflections/ReflectionsDelay,
  Reverb/ReverbDelay, Diffusion, Density, HFReference, WetDryMix) — the de-facto vocabulary; maps directly
  onto the FDN.
- **Game integration:** per-zone **aux bus** (one reverb effect per environment; voices send at a level
  for their zone; distance-weighted blending across overlapping zones to avoid transition clicks). Rooms &
  Portals (§6) extend this to indoor propagation.

---

## §8 Voice management, mixer graph, parameters

- **Virtual/real voice split.** Real voice = full pipeline (decode+resample+filter+spatialize+mix),
  capped (~64–256 desktop/console). Virtual voice = position bookkeeping only, ~free. Each control tick:
  compute **effective audibility** per emitter (distance atten × occlusion × bus gain × HDR window), sort
  by (priority, audibility), take top-N real, virtualize the rest. Priority may be distance-offset.
  Virtual-return behavior configurable (resume-from-where-it-would-be for loops). Cap *simultaneously
  decoding* voices separately — decode is the real cost. An HDR mix window arbitrates hundreds of
  overlapping sounds down to the audible set.
- **Mixer graph.** Bus DAG evaluated post-order per block; insert chain (series on a bus/voice) + aux
  sends (copy at a level to a parallel reverb bus). Effects run on *summed* bus buffers → O(bus) not
  O(voice). Master bus renders to the endpoint; optional object-based pass-through (Atmos/Sonic/Tempest).
- **Parameters.** **RTPC** (continuous game var → authored curve → any property, global or per-emitter),
  **State** (global mix snapshot), **Switch** (per-object discrete selector). All smoothing on the audio
  thread by **ramping toward atomically-published targets** — never a raw jump.
- **Events.** The game posts an **event ID**, never "play file X"; the event resolves to a spawn-list of
  voices via authored containers (random/sequence/blend/switch). This indirection is the sound-designer
  seam.

---

## §9 ECS integration

Trivially-copyable components (per CLAUDE.md `component_id<T>()` rules), consumed via a **wall-clock
snapshot** extracted outside the fixed-step schedule (like animation/skinning):

- `AudioListener` — pose (`WorldVector3` + orientation), the decode reference.
- `AudioEmitter` — event/voice handle, gain, priority, attenuation curve id, RTPC values, aux-send levels,
  obstruction/occlusion scalars.
- `ReverbZone` — bounds + I3DL2 params (or geometry ref for computed RT60).
- `Room` / `Portal` — the portal-graph topology for §6 propagation.

The snapshot extract publishes emitter/listener transforms into the control-plane structures the voice
manager reads; nothing here writes sim state (the on/off byte-identical invariant, §0).

---

## §10 Asset pipeline & streaming

- **Bank format:** authored content baked into a compact binary (media + flattened structure/event/param
  definitions). Runtime loads a bank, posts events by ID.
- **Codecs by content class:** PCM (tiny critical SFX), ADPCM (dense repetitive SFX — artifacts hide),
  Vorbis/Opus streamed (music/dialogue) with a resident prefetch chunk.
- **Streaming thread:** long assets read from disk in chunks into a ring, decoded on the fly by a
  normal-priority worker feeding the voice's input ring; the audio thread never does I/O or decode.
- Distinct memory pools: bank metadata, in-memory samples, streaming/decode buffers, voice/DSP scratch.

---

## §11 Editor authoring

Mirrors the existing editor panel discipline: a mixer/bus panel (bus tree, insert/aux routing, meters),
an emitter inspector (event, attenuation curve editor, RTPC bindings), a reverb-zone editor, and a **live
profiler** (voice count real/virtual, CPU per bus, waveform/meter snapshots streamed from the audio
thread over the audio→GUI ring). Sound designers author → build bank → hot-reload — no programmer in the
loop.

---

## §12 Device I/O and the accelerator seam

**§12.1 Device backend — SDL2 (decided).** SDL2 is already in the stack (input + window), so the audio
device layer adds **zero new dependency** and gives one Windows/Linux/macOS path. `sushi_audio` opens an
`SDL_AudioStream` / audio device and drives the render callback; this is *only* the device I/O + buffer
plumbing — the entire DSP is our from-scratch core. The seam is `IAudioDevice` (open/close, callback
registration, sample-rate/block negotiation) so the backend is swappable (miniaudio / raw
WASAPI-PipeWire) without touching the mix.

**§12.2 GPU offload — `IDspAccelerator` (thin seam now, implementation deferred to S10).** The interface
is defined from the start so the architecture stays clean, but the CPU path is the only implementation
through S9. When implemented, it offloads *batch* DSP (long convolution, HRTF, ambisonic decode,
aeroacoustic field eval) to the SushiRuntime SYCL graph with **k-block lookahead** so the RT thread never
waits on the GPU. The RT mix stays on the CPU (latency-critical, per-sample). This defers all exposure to
the runtime's real-time gaps while preserving the offload path. Keep the seam thin — the runtime's fluent
API is unstable.

---

## §13 SOLID seams

`IAudioDevice` (device I/O), `IDspAccelerator` (optional GPU batch), `IFft` (FFT backend), `IAudioCodec`
(decode), `IHrtfDatabase` (SOFA/generic), `IReverb` (FDN/convolution interchangeable), `ISpatializer`
(ambisonic/panner LOD tiers). Each is thin, isolates an unstable or swappable dependency, and lets a
subsystem be tested against a trivial implementation without mocking the whole engine.

---

## §14 Roadmap

Prefix **S**. S0–S4 are the critical path. Each phase builds with the `se` CLI, ships an example via
`add_sushi_sycl_executable`, tests via `TEST(Unit_Audio, …)`, and updates `docs/CHANGELOG.md` +
`ARCHITECTURE.md` in the same PR.

| Phase | Deliverable |
|---|---|
| **S0** ✅ | Prerequisite `App::runtime()` accessor; `sushi_audio` STATIC lib + `SE_BUILD_AUDIO`; `IAudioDevice` (SDL2) + `IDspAccelerator` (empty) seams; silent block-producing device loop (`audio_demo`, `se audio`). |
| **S1** ✅ | DSP core base: SPSC ring, `ScopedNoDenormals`, block graph (topo-sort schedule + one-block feedback), SIMD gain/pan/mix, biquad + TPT-SVF + one-pole. Play/filter/mix a sine (`audio_dsp_demo`, `Unit_Audio`). |
| **S2** ✅ | Voice manager (virtual/real, audibility sort) + bus/mixer DAG + insert/aux-send + RTPC ramping. Multi-source prioritized mix (`audio_mixer_demo`, `Unit_Audio`). |
| **S3** ✅ | Propagation: Farrow fractional delay (Doppler + delay), distance-model gain, ISO 9613-1 air-absorption LPF, slew/teleport-snap. Flyby Doppler (`audio_propagation_demo`, `Unit_Audio`). |
| **S4** ✅ | Spatializer: ambisonic encode (ACN/SN3D) → head-relative rotation → virtual-speaker + analytic-HRTF binaural decode (Woodworth ITD + head-shadow). Binaural 3D + head-track (`audio_spatial_demo`, `Unit_Audio`). *(SOFA/MagLS/UPOLS = later fidelity upgrade behind the same seam.)* |
| **S5** | Reverb: FDN (Householder, coprime delays, damping) + I3DL2 API + per-zone aux bus; Sabine/Eyring RT60. |
| **S6** | ECS integration: `AudioEmitter`/`AudioListener`/`ReverbZone`/`Room`/`Portal` + wall-clock snapshot extract. |
| **S7** | Occlusion/obstruction: acoustic BVH (static BLAS + dynamic TLAS), volumetric soft occlusion, material transmission, rooms+portals propagation. |
| **S8** | Asset/streaming: bank format, codecs (PCM/ADPCM/Opus), disk streaming + decode thread; event system. |
| **S9** | Editor authoring: mixer/bus panel, emitter inspector, RTPC curves, live profiler. |
| **S10** (opt.) | Procedural SFX (modal impact synthesis, wind: filtered noise + Aeolian/Strouhal tone); `IDspAccelerator` GPU implementation on SushiRuntime; convolution reverb (Gardner NUPC) upgrade. |

---

## §15 References

Foundational: Julius O. Smith III, *Physical Audio Signal Processing* (CCRMA, free); Zotter & Frank,
*Ambisonics* (Springer 2019); Guy Somberg (ed.), *Game Audio Programming: Principles and Practices*
(vols 1–5). Open code to study: **Google Resonance Audio** (renderer + Sabine/Eyring reverb — best single
reference), **IEM Plugin Suite** (AmbiX + MagLS + AllRAD), **Steam Audio** (occlusion/BVH/pathing),
**Faust** reverb library, **Embree** (BVH), **pffft** (FFT). Key papers: Gardner 1995 (convolution
without I/O delay); Jot & Chaigne 1991 (FDN); Raghuvanshi & Snyder 2014 (parametric wave-field coding);
RBJ Audio EQ Cookbook; Cytomic/Zavalishin TPT filters; Laakso et al. 1996 (fractional delay); MagLS
(Schörkhuber et al.); AllRAD (Zotter & Frank); ISO 9613-1 (air absorption); the UNC survey *Sound
Synthesis, Propagation, and Rendering* (arXiv 2011.05538).
