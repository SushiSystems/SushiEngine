# SushiEngine API Reference {#mainpage}

SushiEngine is the head of the stack: a C++17 game engine — an archetype
entity-component-system, an extended position-based physics solver, a Vulkan frame graph and
the stacks around them — that plugs the
[SushiRuntime](https://github.com/SushiSystems/SushiRuntime) orchestration runtime in as its
battery. You describe a world of entities and components, register systems, and the engine
lowers them onto the runtime's task graph to execute in parallel across the central and
graphics processors.

This reference documents the public surface of every module: the `include/` tree of each
directory under `engine/<tier>/<module>/`. A module's `source/` is its implementation and does
not reach this site.

## Where to start

- **Entity-component-system** (`SushiEngine/ecs`) — `World`, the archetype and chunk storage,
  the `Schedule` that orders systems, and the deferred `CommandBuffer`.
- **Physics** (`SushiEngine/physics`) — `Constraint`, graph colouring, and `PGSSolver`.
- **Core** (`SushiEngine/core`) — the shared value types, `Scalar` among them.
- **Render** (`SushiEngine/render`) — the frame graph, the passes and the scene view seam.

The single umbrella include is `SushiEngine/SushiEngine.hpp`, in `engine/include/`.

Each module also carries its own `README.md`, reachable from the related-pages list, stating
what the module owns, which tier it sits in, what it links and what covers it in test.
