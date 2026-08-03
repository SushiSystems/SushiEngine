# Repository restructure (RESTRUCTURE0)

Status: approved 2026-08-03, not started.

## 1. Why

Three incompatible header policies coexist today, so a single module is split across
up to three root-level trees:

| Policy | Modules | Consequence |
| --- | --- | --- |
| Header-only | `physics` (101 headers), `animation` (44), `ecs`, `loop`, `ui`, `vfx`, `astro`, `terrain`, `core` | live only under `include/SushiEngine/`, no root source directory |
| Split | `geometry`, `atmosphere`, `execution_native`, `import`, `sim`, `cooking` | headers under `include/SushiEngine/<x>/`, sources at root `<x>/` |
| Colocated | `render`, `editor`, `scene`, `platform`, `input`, `audio`, `se_player` | headers beside their sources |

`cooking/` is the worst case: sources at root `cooking/`, headers under
`include/SushiEngine/physics/cooking/`, its own target. Maintaining one module means
editing two or three unrelated root directories, which is the opposite of the
locality `docs/CLAUDE.md` demands.

A single central include tree also *cannot express* "physics may not see render".
`astro/ephemeris.hpp:53` and `atmosphere/quasigeostrophic_core.hpp:79` both include
render headers and both shipped, because nothing could stop them.

## 2. Target layout

Root keeps 12 files and 8 directories. All engine C++ moves under `engine/`, grouped
by named tier. Tier order lives in `cmake/EngineLayers.cmake` and is enforced at
configure time.

```
sushiengine/
├── CMakeLists.txt  CMakePresets.json  README.md  LICENSE  CLAUDE.md
├── .clang-format  .editorconfig  CODEOWNERS  .sushiengine-root
├── Dockerfile  Doxyfile  .github/  .gitignore  .dockerignore  .gitmodules
├── cli/                      unchanged location (external contracts, §6)
├── cmake/                    + EngineLayers.cmake, Module.cmake, Shaders.cmake
├── assets/  third_party/  build/{default,editor,player}/
│
├── engine/
│   ├── foundation/   core · ecs · execution · platform
│   ├── domain/       geometry · physics · environment · animation · astro ·
│   │                 atmosphere · terrain · vfx · ui · audio · input
│   ├── asset/        gltf
│   ├── presentation/ render
│   └── world/        loop · simulation · serialization · authoring
│
├── applications/  editor/ · player/
├── samples/       sandbox/ · animation/ · audio/ · physics/ · rendering/ · authoring/
├── tools/         shader_compiler/ · probes/
├── tests/         unit/ · integration/ · regression/ · native_execution/ · goldens/
└── docs/          getting-started/ · architecture/ · modules/ · guides/ ·
                   reference/ · contributing/ · design/
```

Every module is one directory:

```
engine/domain/physics/
├── CMakeLists.txt
├── README.md          what it owns, its tier, its dependencies
├── include/SushiEngine/physics/
├── source/
└── tests/
```

Because each module's public root is `<module>/include` and the subtree beneath it
keeps its spelling, roughly 900 files keep their `#include` lines byte-for-byte.
Only three subtrees change spelling, and all three are rule violations being paid
off anyway: `sim/` → `simulation/` (69 files), the two orphan glTF headers (8 files),
and `loop/rng.hpp` → `core/random_number_generator.hpp` (4 files).

## 3. Enforcement

`cmake/Module.cmake` defines `sushiengine_add_module()` and is **the only place a
public include directory is ever granted**:

```cmake
sushiengine_add_module(NAME physics LAYER domain PUBLIC_DEPENDS core geometry)
```

A dependency whose tier index exceeds its own raises `FATAL_ERROR` naming the
offending edge. `cmake/EngineLayers.cmake` holds the ordered tier list, a
`SUSHIENGINE_FORBIDDEN_EDGES` pair list stating invariants the tier order alone
cannot ("presentation/render must never see world/simulation"), and a dated
`SUSHIENGINE_LAYER_EXCEPTIONS` list that CI prints and that may only shrink.

### Where presentation sits, and why

The tier order is `foundation → domain → asset → presentation → world →
application`: the renderer is a **service the world consumes**, not a consumer of it.
That is what the tree actually says. `render/` includes something under
`SushiEngine/sim/` exactly once, at `render/probe/atmosphere_main.cpp:56`, and that
file moves to `tools/probes/` — so after the restructure presentation→world is zero
edges.

The traffic runs the other way and stays there. Eleven `#include
<SushiEngine/render/...>` lines across the world modules descend into
`domain/environment` with `environment.hpp` (7), `atmosphere_nest.hpp` (2),
`weather_field.hpp` (1) and `light.hpp` (1), which also retires the astro→render
inversion. Exactly three edges survive, and all three are legal downward edges under
this order:

| Consumer | Header |
| --- | --- |
| `world/simulation` (`simulation.hpp`) | `render/scene_view.hpp` |
| `world/authoring` (`preferences.hpp`) | `render/render_settings.hpp` |
| `world/serialization` (`effect_serializer.cpp`) | `render/material.hpp` |

So `SUSHIENGINE_LAYER_EXCEPTIONS` starts **empty**. `SUSHIENGINE_FORBIDDEN_EDGES`
still carries `render;simulation`, deliberately redundant with the tier comparison,
so that reordering the tier list can never silently legalise the renderer reaching
into the simulation.

### Why `loop` is a world module

`loop/` is the orchestration surface — `Loop::App`, the fixed-timestep clock,
rollback and reconciliation — not a domain model, so it sits in `world` beside the
simulation. Its own headers include only `ecs`, `core` and `execution`, all
foundation, so nothing about it resists the placement.

Placing it there is what gives the enforcement something real to catch:
`atmosphere/quasigeostrophic_core.hpp:79` includes `loop/rng.hpp`, which is a domain
module reaching up into the session loop for a seeded random number generator. The
fix is the descent already planned — `rng.hpp` includes nothing but `<cstdint>` and
belongs in `core`. Had `loop` been left in `domain`, that edge would have been
same-tier and legal, and the enforcement would have had nothing to say about it.

`material.hpp` must be split before that move: `atmosphere_nest.hpp:1687` defines
`IAtmosphereMirror`, which `IAssetLibrary` (`material.hpp:232`) inherits.
`IAssetLibrary` stays in render as `asset_library.hpp`. That is a code edit, not a
move, so it belongs to phase 2.

Name the descended aggregate `Environment::Description` — `SushiEngine::World`
collides with the ECS class and `SushiEngine::Scene` is taken by the serializer.

## 4. Phases

Each phase builds clean on its own and is reviewed on its own.

| Phase | Scope | Size |
| --- | --- | --- |
| **1 — move** | pure `git mv`, byte-identical content, plus the CMake rewiring needed to build | ~1100 files |
| **2 — naming** | `sushiengine_` prefix, acronym casing, abbreviation removal, license headers, comment hygiene | 278 CMake identifiers + several hundred C++ symbols |
| **3 — unlinked** | wire or remove all 36 audited items | 36 items |
| **4 — documentation** | restructure and rewrite `docs/` | 73 work items |

### How the tree keeps building while it moves

The `SushiEngine` INTERFACE target currently grants `include/` to every consumer,
which is the flat visibility this restructure exists to remove. Cutting it in one
step would break the tree for the whole of phase 1, so it is retired last instead.

Each module relocated in 1b–1f declares itself with `sushiengine_add_module()`,
which grants its own `include/`. The `SushiEngine` INTERFACE target then links the
relocated module targets, so consumers that have not moved yet still resolve every
header transitively and the build stays green after every task. Enforcement is
therefore live between relocated modules from 1b onward, and grows with each step,
while the not-yet-moved directories keep riding the umbrella. Task 1g removes the
umbrella and points each remaining consumer at the modules it actually uses — that
is the step where the flat visibility finally goes away.

### Phase 1 hard constraints

Git has no rename records: `git log --follow` and `git blame` depend on similarity
detection at read time, which fails when a file moves *and* its content changes in
the same commit. Therefore:

- Every move commit contains byte-identical file content and nothing else.
- Every path-string, target-name and namespace edit lands in a separate commit.
- Add `.git-blame-ignore-revs` listing the move commits and set
  `blame.ignoreRevsFile` in the repository config.

Two rows look like moves but are code edits, and are therefore deferred to phase 2:

- `render/geometry/meshlet.*` → `domain/geometry` requires extracting `MeshVertex`
  out of `mesh_registry.hpp` first; `meshlet.cpp:26` pulls that header in and it
  includes `<vulkan/vulkan.h>` at lines 42-43.
- `render/terrain/terrain_frame.hpp` → `domain/terrain` is **not viable at all**
  until `CameraView` is extracted from `scene_view.hpp`. Its lines 49-50 include
  `render/environment.hpp` and `render/scene_view.hpp`, and its only function
  (line 131) takes `const CameraView&` and `const Environment&`. Moving it as
  proposed would create a new domain→presentation edge.

## 5. Move table

| From | To |
| --- | --- |
| `include/SushiEngine/core/` | `engine/foundation/core/include/SushiEngine/core/` |
| `include/SushiEngine/loop/rng.hpp` | `engine/foundation/core/include/SushiEngine/core/random_number_generator.hpp` |
| `include/SushiEngine/ecs/` | `engine/foundation/ecs/include/SushiEngine/ecs/` |
| `include/SushiEngine/execution/` | `engine/foundation/execution/include/SushiEngine/execution/` |
| `execution_native/*.cpp` | `engine/foundation/execution/source/backend/native/` |
| `platform/` | `engine/foundation/platform/{include/SushiEngine/platform,source}/` |
| `geometry/` + `include/SushiEngine/geometry/` | `engine/domain/geometry/{source,include/SushiEngine/geometry}/` |
| `include/SushiEngine/physics/` | `engine/domain/physics/include/SushiEngine/physics/` |
| `cooking/*.cpp` | `engine/domain/physics/source/cooking/` (include path stays `SushiEngine/physics/cooking/`, 41 includers) |
| `render/{environment,light}.hpp`, `render/{atmosphere_nest,weather_field,synoptic_field}.hpp` | `engine/domain/environment/include/SushiEngine/environment/` |
| `include/SushiEngine/{animation,astro,terrain,vfx,ui}/` | `engine/domain/<x>/include/SushiEngine/<x>/` |
| `include/SushiEngine/loop/` (remainder) | `engine/domain/loop/include/SushiEngine/loop/` |
| `atmosphere/*.cpp` + `include/SushiEngine/atmosphere/` | `engine/domain/atmosphere/{source,include/SushiEngine/atmosphere}/` |
| `audio/{sdl,miniaudio}/` + `include/SushiEngine/audio/` | `engine/domain/audio/{source/backend,include/SushiEngine/audio}/` |
| `input/sdl/` + `include/SushiEngine/input/` | `engine/domain/input/{source/backend,include/SushiEngine/input}/` |
| `import/*.cpp` + the two orphan glTF headers | `engine/asset/gltf/{source,include/SushiEngine/gltf}/` |
| `render/` (minus `tools/`, `probe/`, `geometry/meshlet`) | `engine/presentation/render/{source,include/SushiEngine/render,shaders}/` |
| `sim/` + `include/SushiEngine/sim/` | `engine/world/simulation/{source,include/SushiEngine/simulation}/` |
| `scene/` | `engine/world/serialization/{source,include/SushiEngine/serialization}/` |
| `editor/core/{command_history,preferences,autosave}.*`, `editor/physics/{cook_bake_state,soft_body_heat}.*` | `engine/world/authoring/` (new library `sushiengine_authoring`) |
| `editor/` (remainder) | `applications/editor/source/` |
| `se_player/` | `applications/player/` |
| `sandbox/main.cpp`, `examples/*.cpp` (53) | `samples/sandbox/`, `samples/{animation,audio,physics,rendering,authoring}/` |
| `editor/scene/physics_sample_scene.*` | `samples/physics/sample_scene/` |
| `examples/assets/*.gltf` (3) | `assets/models/` |
| `render/tools/shader_compiler/` | `tools/shader_compiler/` |
| `render/probe/`, `atmosphere/probe/` | `tools/probes/{render,render_golden,atmosphere,atmosphere_global}/` |
| `render/probe/goldens/` | `tests/goldens/render/` |
| `tests/functional/{unit,integration,regression}/` | `tests/{unit,integration,regression}/` |
| `tests/execution_native/` | `tests/native_execution/` (keeps its deliberate reach-back to shared translation units) |
| `include/SushiEngine/SushiEngine.hpp` | `engine/include/SushiEngine/SushiEngine.hpp` |
| `build/`, `build-editor/`, `build-player/` | `build/{default,editor,player}/` |
| `docs/CLAUDE.md` | `CLAUDE.md` (root) |
| root `LICENCE` + `docs/LICENSE` | one root `LICENSE` |

Deleted: `cooked/`, `editor/serialization/`, `editor/window/` (all three verified
empty on disk), `docs/LICENSE`, `docs/api-site/`, and `include/` itself once drained.
`imgui.ini` is **not** tracked by git — nothing to remove; the fix is to set
`io.IniFilename` under `Platform::user_data_directory()` and ship
`applications/editor/layout/default_layout.ini` as the first-run default.

Tests move mechanically in phase 1; the `functional/` level is dropped because every
test in the tree is functional and the three subdirectories already match the three
`gtest_discover_tests` label partitions. Colocating the 125 unit tests into their
modules happens afterwards, module by module, aggregating into the same binary via
per-module `target_sources()` so ctest labels never change.

## 6. Things that must not move, and why

`cli/` stays at the repository root. `Dockerfile:83` pip-installs it,
`cli/sushiengine/config.py:52` and `:185` hard-code `root/"cli"`, `.gitignore` carries
three `cli/` rules, `cli/sushistack.deps.toml` is read by the external `ss` tool at a
path this repository does not own, and `cli/sushiengine/config.py:29-30` imports from
`sushicli`, a separate sibling repository. Moving it is a cross-repository contract
change dressed up as decluttering.

But its root marker **must** change. `cli/sushiengine/config.py:40` resolves the
project root with `walk_up(cwd, has_marker("CMakeLists.txt"))`, which finds the
*nearest* ancestor holding one. Running `se build` from inside `render/` already
mis-resolves today; after this restructure that landmine sits under every one of the
~20 module directories. Add a root-only `.sushiengine-root` sentinel and change that
one line.

## 7. Fixed in the same change

- `cmake/Runtime.cmake` `add_subdirectory(../sushiruntime)` into one flat CMake
  namespace is the entire reason `CONTRIBUTING.md` mandates the `sushiengine_`
  prefix, and no CI job ever configures both repositories together — so the
  collision the rule prevents has never been tested. Add that job.
- `SE_EXECUTION_BACKEND=native` has no CLI flag, no CI job and no documentation
  entry; `SE_BUILD_PLAYER` is configured by nothing in `ci.yml`.
- `sandbox` and `pgs_demo` are declared twice (`CMakeLists.txt:89/:544` and
  `:90/:545`), once per execution lane, and are the only two targets proving the
  native lane works. `samples/CMakeLists.txt` must keep both lanes.
- `render/CMakeLists.txt:459-460` compile absolute source-tree and build-tree paths
  (`SUSHI_SHADER_SOURCE_DIR`, `SUSHI_PIPELINE_CACHE_DIR`) into the shipped library.
  Route both through `Platform::user_data_directory` and a relative shader search
  path.
- Asset resolution is by cwd-relative string literal in shipped code
  (`render/terrain/planet_terrain.cpp:69`, `include/SushiEngine/sim/climatology_asset.hpp:52`
  in a public header, `editor/vfx/particle_panel.cpp:54` pointing at a directory that
  does not exist). Two tests already work around it with a five-deep walk-up hunt.
  This needs one seam, not a new `assets/effects/` directory invented to satisfy a
  dangling literal.
- `render/CMakeLists.txt` `SHADER_INCLUDES` omits `punctual_shadow_common.glsl`, so
  edits to it do not rebuild `sky.frag` or `clustered_lighting.glsl`.
- `Doxyfile` `INPUT` points at `docs/guides/ARCHITECTURE.md`, which does not exist,
  so the architecture manual is silently dropped from the generated site.
- Four UI-free editor translation units are recompiled into two or three targets each
  because no library target exists for them; the new `sushiengine_authoring` library
  deletes `CMakeLists.txt:496-501` and `tests/CMakeLists.txt:148,561,562,616`.
- No `.clang-format`, `.editorconfig`, `CODEOWNERS` or `CMakePresets.json` exists, so
  `CONTRIBUTING.md` §4 has no mechanical enforcement at all.

## 8. Phase 2 — naming

| Class | Count |
| --- | --- |
| Bare `sushi_` CMake identifiers | 278, and zero `sushiengine_` ones. All 13 libraries and all three functions, including `sushi_configure_test_target` — the exact collision example `CONTRIBUTING.md` gives |
| Title-cased acronyms | `Aabb` (111), `Xpbd` (~200), `Dsp` (37 files), `Vfx` (31), `Gi` (11), plus `Gpu`, `Sdf`, `Bvh`, `Ibl`, `Taa`, `Ssr`, `Gtao`, `Fxaa`, `Hiz`, `Fem`, `Gltf`, `Hrtf`, `Fdn`, `Lod`, `Rng`, `Dag`, `Json`, `Sdl`, `Glsl`, `Lut`, `Ik`, `Fft` |
| Abbreviated identifiers | `*Params` 33 types, `*Desc` 33 types, `Mat4` (76 files, defined beside a correctly-spelled `Vector3`), `cmd` 737 occurrences including virtual interface signatures |
| Lowercase namespaces | `detail` ×20 in public headers while `Detail` appears correctly ×12; `placeholder`; `fs` aliases ×7 |
| Truncated license headers | 151 files missing the four-line grant clause |
| Separator comments | 210 lines across 65 files |
| Historical narration comments | 247 lines across 109 files |
| File names | the `sim/` rename (69 including files), `dof_pass`, `particle_sim_pass`, `hiz_pass`, `*_params.hpp` ×4, `cluster_config.hpp`, `upscaler_info.hpp`, `ma_audio_device`, `rect.hpp`, `rng.hpp`, `net.hpp`, four `*_impl.cpp` |

`UiPass`/`UiBuffers`/`UiView`/`UiVertex` are self-contradicting: `include/SushiEngine/ui/`
already spells `UIButton`/`UIText`/`UIDrawList` correctly.

## 9. Phase 3 — unlinked inventory

Build wiring itself is clean: every `.cpp` on disk appears in an explicit
(non-glob) source list, no orphan translation units exist, every shader is compiled
and catalogued, and every `PanelVisibility` flag has both a menu item and a draw
function. The unlinked mass is elsewhere. Policy: wire it; remove the control only
when wiring needs shader or pass work that cannot be verified without a GPU.

**Data loss**

- `SoftBodyParams` is addable via Add Component ▸ Soft Body but absent from
  `scene_serializer.cpp`'s `capture_scene`, so Save, Load, Undo/Redo and Play→Stop
  each destroy it silently.

**Implemented features with no entry point**

- The Crowd component quintet on `IWorldEditor` is implemented in
  `sim/runtime_simulation.cpp:939-996,1280-1290` and rendered through the
  skinned-instance channel, with no menu item, no Inspector section, no scene
  serialization and no test.
- `Terrain::LayerStack` (Flatten/Raise/Crater) is implemented and unit-tested and
  `PlanetTerrain::layers()` exposes it, but nothing ever adds a layer.
- `Vfx::BeamModule` + `RenderAlignment::Beam` have a working GPU path
  (`particle_ribbon.vert`, `is_beam`) but no Alignment combo entry and no serializer
  field.
- `Vfx::SortMode` is authored, serialized, compiled and uploaded to the GPU emitter
  table, and read by no shader or pass.
- `PlanetTerrain::statistics()` is computed every frame and displayed nowhere.
- `create_interop_buffer` / `IInteropBuffer`, `IVehicleService::vehicle_surface` and
  `ISoftBodyService::soft_body_maximum_stress` have zero callers.

**Controls that do nothing** — Material Inspector's Blend Mode, Render Queue, Wrap
Mode and Anisotropic Filtering; Post Process's Bloom Threshold and Knee. Also
`Material::receive_shadows` / `gpu_instancing` (serialized, no UI, no consumer) and
`ColorGradeSettings::lut_enabled`.

**Documentation and lane links** — `docs/CLI_GUIDE.md` never mentions `se player` or
`se climatology`; `docs/README.md`'s tables omit player/planet/climatology,
`SE_BUILD_PLAYER` and `SE_EXECUTION_BACKEND`; `docs/VEHICLES.md` is absent from the
index; `first_game.cpp` and `audio_authoring_demo.cpp` are built and documented
nowhere; `include/SushiEngine/input/replay_json.hpp` is included by no file.

**Zero test coverage** — `sushi_render` is linked by no test target at all;
`sushi_platform`, the compiled input translator, both audio device backends and all
of `se_player` have no ctest coverage.

## 10. Phase 4 — documentation

`ARCHITECTURE.md` is 298 KB / 3790 lines with a single 12,806-character paragraph.
`CHANGELOG.md` is 205 KB across 617 lines — 304 bullets, median 180 characters,
90th percentile 681, longest 2788 — against a `CONTRIBUTING.md` §5 rule that says
"a short bullet, not an essay… one line, a past-tense verb, then the object."

The restructure:

- `docs/architecture/` — `ARCHITECTURE.md` split by tier, with an overview and a
  table of contents.
- `docs/modules/<module>.md` — mirrors `engine/<tier>/<module>/` one-to-one. A CI
  script walks both trees and fails on asymmetry. Each module also keeps a short
  `README.md` stating what it owns, its tier and its dependencies.
- `docs/getting-started/`, `docs/guides/`, `docs/reference/`, `docs/contributing/`.
- `docs/slop/` → `docs/design/`, indexed, each file carrying a status header,
  excluded from Doxygen `INPUT` and from every public link.
- `docs/documentation-style-guide.md` (new) — voice, naming, link and path-reference
  rules, **and explicit length ceilings**: one line per changelog bullet, a paragraph
  ceiling, a file ceiling that forces a split. Enforced by a CI length check, because
  the rule already exists in prose and is already ignored.
- A root `README.md`, which does not exist today; `docs/README.md` becomes an index
  only.
- `docs/glossary.md` for the phase codes scattered across the corpus (WP-3, M2-M5,
  S0-S10, A0-A9, W4-W6, VFX1-6, P0-P2b, UX0-UX6, PLATFORM0 S1-S6, RHI0, UHM, T1/T2).

Documents obey the repository's own no-abbreviation and acronym-casing rules in prose
and headings, and every markdown link must resolve.

## 11. Verification

Phase 1 is verified when `se build`, `se editor --no-run`, `se build --player` and
`se test --suite functional` all reproduce their pre-move results, with an added CI
job configuring both repositories together and a job configuring the native lane.
Phases 2 and 3 add no new build modes; phase 3's items each need a scene round-trip
check. Builds run on the user's machine, not here.
