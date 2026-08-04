# Module index

Every engine module carries its own `README.md` beside its code, stating what it owns, which
tier it sits in, what it links and why, its public headers, and what tests cover it. Those files
are the source of truth: they are updated by whoever changes the module. This page is an index
onto them and holds no facts of its own.

The tier order is `foundation` → `domain` → `asset` → `presentation` → `world` → `application`.
A module may depend on its own tier and on anything below it; `cmake/EngineLayers.cmake` states
the order, the module-to-tier assignment and the edges that are refused outright, and
`cmake/Module.cmake` fails the configure on an illegal edge.
`tools/documentation/check_module_documentation.py` fails when a module has no `README.md`, when
a `README.md` carries the wrong Doxygen label, or when this table and the module tree disagree.

| Module | Tier | Owns |
|---|---|---|
| [core](../../engine/foundation/core/README.md) | `foundation` | The value vocabulary the engine is written in — scalar, vector, quaternion, floating-origin position — and the seeded generator a deterministic system draws from. |
| [ecs](../../engine/foundation/ecs/README.md) | `foundation` | Entities, archetype-chunk storage of structure-of-arrays columns, and the schedule that turns declared reads and writes into execution nodes. |
| [execution](../../engine/foundation/execution/README.md) | `foundation` | The `Context`/`Graph`/`Buffer` seam every subsystem allocates and schedules through, its normative hazard semantic, and the two backends that satisfy it. |
| [platform](../../engine/foundation/platform/README.md) | `foundation` | The window and operating-system lifecycle seam, and its SDL2 implementation. |
| [animation](../../engine/domain/animation/README.md) | `domain` | Skeletons and clips, the animator state machine and blend trees, the inverse-kinematics and pose-modifier stack, and the evaluators that produce a skinning palette. |
| [astro](../../engine/domain/astro/README.md) | `domain` | Where the sun, moon, planets and stars are: astronomical time, orbital elements, the ephemeris, gravity, and the observer's reference frames. |
| [atmosphere](../../engine/domain/atmosphere/README.md) | `domain` | The global dynamical core — two-layer moist quasi-geostrophic flow — its baked mean climatology, and the transform its elliptic inversion is built on. |
| [audio](../../engine/domain/audio/README.md) | `domain` | The from-scratch digital signal processing graph, the voice and mixer model on it, and the spatialisation, propagation, reverb and asset seams around it. |
| [environment](../../engine/domain/environment/README.md) | `domain` | The scene's outdoor state as plain data: sun, sky, punctual lights, and the atmospheric fields a frame is shaded against. |
| [geometry](../../engine/domain/geometry/README.md) | `domain` | Engine-neutral triangle geometry: the mesh types, topology repair, the closest-point hierarchy, the signed-distance bake, and the meshlet split. |
| [input](../../engine/domain/input/README.md) | `domain` | The device-neutral action layer: events, bindings, per-player action maps, gestures, haptics, text entry and the replay tick stream. |
| [material](../../engine/domain/material/README.md) | `domain` | The authored surface: its maps and scalars, the identifiers a texture or mesh is registered under, and the state that decides how it is drawn. |
| [physics](../../engine/domain/physics/README.md) | `domain` | What a simulated body is: colliders, constraints, the solvers that move them, soft bodies, vehicles, and the cooking seam that produces all three. |
| [terrain](../../engine/domain/terrain/README.md) | `domain` | A planet's surface as data: the cube-sphere map, the quadtree, tile addressing and residency, and the height sources and layer stack a tile is baked from. |
| [ui](../../engine/domain/ui/README.md) | `domain` | The retained interface as ECS components, the anchor solver and pointer model over them, and the draw list they flatten into. |
| [vfx](../../engine/domain/vfx/README.md) | `domain` | A particle effect as authored and as compiled, plus the deterministic host backend that steps the compiled emitter table. |
| [gltf](../../engine/asset/gltf/README.md) | `asset` | Turning a glTF file into engine data: triangles as a mesh, skins and animations as relocatable blobs. |
| [render](../../engine/presentation/render/README.md) | `presentation` | Drawing a frame: the render graph, the passes on it, the caches and pools they draw from, and the Vulkan device behind the render hardware interface seam. |
| [authoring](../../engine/world/authoring/README.md) | `world` | The services an editor is built out of, with no editor in them: undo, preferences, autosave, the cook and bake model, and the soft-body heat scale. |
| [loop](../../engine/world/loop/README.md) | `world` | The orchestration surface a session is driven through: `Loop::App`, the fixed-timestep clock, the rollback ring and network reconciliation. |
| [serialization](../../engine/world/serialization/README.md) | `world` | The one JSON shape for a scene file, a particle effect asset, and the environment both embed. |
| [simulation](../../engine/world/simulation/README.md) | `world` | The live world behind the `ISimulation` seam: a runtime, a `World` and a `Schedule`, wired into one stepping simulation and extracted back out. |

The `application` tier holds the executable shells (`applications/editor`, `applications/player`)
rather than modules, so nothing in it appears above.
