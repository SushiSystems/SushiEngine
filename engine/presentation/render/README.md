# Render {#module-render}

`render` owns drawing a frame: the render graph that derives every barrier, layout and transient
alias from what a pass declares, the passes themselves, the caches and pools they draw from, and
the Vulkan device behind the render hardware interface seam. Consumers program against the
abstract `IRenderDevice`, `IWindowRenderer` and `ISceneView`; `source/rhi/vulkan/` is the only
implementation and the only place Vulkan-specific device code lives.

## Tier

`presentation` — the fourth tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation`, `domain`, `asset` and other `presentation` modules. `SUSHIENGINE_FORBIDDEN_EDGES`
additionally refuses `render` → `simulation` outright, whatever the tier order says, so the
renderer can never reach into the live world.

## Dependencies

Engine modules, all public because each is spelled in a header this target publishes:

- `core` — every position and transform crossing the seam.
- `ecs` — the scene systems step entities.
- `material` — `asset_library_interface.hpp` hands back the material a loaded primitive comes as.
- `environment` — `scene_view.hpp` carries the environment, and the asset library answers with
  the atmosphere mirror.
- `ui` — `scene_view.hpp` carries an interface draw list.
- `terrain` — `planet_terrain.hpp` carries a quadtree and layer stack.
- `vfx` — `particle_system.hpp` carries the compiled emitter table.
- `geometry` — meshes and their distance fields.
- `gltf` — the importer's types, and cgltf's implementation unit, which this target takes from
  there rather than compiling again.

`animation` is private: the glTF importer fills in skeleton and skin-vertex blobs, and no header
here exposes them.

External packages: `Vulkan::Headers`, `Vulkan::Loader`, `GPUOpen::VulkanMemoryAllocator` and
`vk-bootstrap::vk-bootstrap` publicly; `glslang` (three targets) and `Threads::Threads`
privately. `Stb` and cgltf are header-only and reached by include directory, with
`source/material/stb_impl.cpp` as the single translation unit carrying both implementations. The
module links neither SushiRuntime nor SYCL, so it builds on a stock toolchain.

The 84 shader stages under `shaders/` are compiled to C++ headers of embedded SPIR-V words by
`tools/shader_compiler`, so the binary ships its shaders inside itself and reads no source at
run time. `SUSHIENGINE_SHADER_SOURCE_DIR` points hot-reload at the source tree when it exists,
which is true of a development build and not of a deployed one.

## Public surface

Nine headers, relative to `include/SushiEngine/render/`. Everything else is implementation and
lives under `source/`.

| Header | Declares |
|---|---|
| `rhi/device.hpp` | `IRenderDevice` — the graphics-device abstraction the renderer is written against. |
| `window_renderer.hpp` | `IWindowRenderer` — the presentation surface a windowed host draws through. |
| `scene_view.hpp` | `ISceneView` — an offscreen three-dimensional view a host samples into its own interface. |
| `asset_library_interface.hpp` | The seam a host loads textures and meshes through without seeing a device type. |
| `render_settings.hpp` | How the renderer is asked to trade fidelity against frame time. |
| `quality_params.hpp` | The one place a quality tier turns into concrete per-pass parameters. |
| `upscaler_info.hpp` | Which reconstruction backends the build carries, and what a request resolves to. |
| `deformable_mesh.hpp` | The renderer's view of host-simulated, per-frame-changing geometry. |
| `interop.hpp` | Renderer memory another interface can address without a copy. |

The implementation is grouped by concern under `source/`: `graph/` derives barriers and resource
lifetimes, `resources/` owns the caches and pools, `passes/` holds one file per pass, `scene/`
and `geometry/` carry the shared scene data, `gi/` the probe and distance-field global
illumination, `atmosphere/` the regional weather nest, `terrain/` the planetary tile path, and
`rhi/vulkan/` the device itself.

## Tests

Effectively none, and this is the largest coverage gap in the repository. No test target links
`sushiengine_render`. Two cases reach into it by include path only, without the library:

- `tests/unit/test_deformable_mesh.cpp` includes `<SushiEngine/render/deformable_mesh.hpp>` and
  covers the grid triangulation and the vertex-triangle adjacency table.
- `tests/unit/test_terrain_frame.cpp` includes `source/terrain/terrain_frame.hpp` directly and
  covers the frame maths, which is header-only and Vulkan-free on purpose.

The render graph, every pass, the descriptor and pipeline caches and the Vulkan device have no
automated coverage. `se render` builds and runs the headless `render_probe`, which exercises the
device, shader and submit path — but it is a smoke test outside the CTest suite, not coverage of
the pass stack. Closing this needs a device the continuous-integration machines do not have.

## Further reading

- [`render_pipeline_refactor.md`](../../../docs/design/render_pipeline_refactor.md) — the living
  plan for the pipeline, phase by phase.
- [`atmosphere_system.md`](../../../docs/design/atmosphere_system.md) — the regional nest and the
  cloudscape this module implements on the device.
- [`solar_system_overhaul.md`](../../../docs/design/solar_system_overhaul.md) — the planetary
  terrain draw path.
- [`unified_hazard_model.md`](../../../docs/design/unified_hazard_model.md) — the shared
  execution vocabulary the graph's barrier derivation is stated in.
- [`presentation-render.md`](../../../docs/architecture/presentation-render.md) — the frame graph
  and every pass hanging off it.
