# SushiEngine API Reference {#mainpage}

SushiEngine is the head of the stack: a header-only C++17 engine — an archetype
ECS and a projected-Gauss-Seidel physics solver — that plugs the
[SushiRuntime](https://github.com/SushiSystems/SushiRuntime) orchestration runtime
in as its battery. You describe a world of entities and components, register
systems, and the engine lowers them onto the runtime's task graph to execute in
parallel across CPU and GPU.

This reference documents the public engine surface under
`include/SushiEngine`. The layered design — how the ECS schedule maps onto the
runtime's graph, and how the PGS solver is coloured and batched — is described in
the hand-written [Architecture guide](../guides/ARCHITECTURE.md).

## Where to start

- **ECS** (`SushiEngine/ecs`) — `World`, `Archetype`/`Chunk` storage, the
  `Schedule` that orders systems, and the deferred `CommandBuffer`.
- **Physics** (`SushiEngine/physics`) — `Constraint`, graph colouring, and the
  `PGSSolver`.
- **Core** (`SushiEngine/core`) — shared value types.

The single umbrella include is `SushiEngine/SushiEngine.hpp`.
