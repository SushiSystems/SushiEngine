# Execution {#module-execution}

`execution` owns the seam every subsystem allocates and schedules through: the access algebra a
node declares its memory intent in, the normative hazard semantic that says which declarations
conflict, and the `Context`/`Graph`/`Buffer` names a backend resolves. The seam is a
compile-time policy rather than a virtual interface, because a device backend forwards each
kernel into its own launch as the original callable and a type-erased one cannot be captured
into device code. The module also owns the choice of backend, since it owns the seam.

## Tier

`foundation` — the lowest tier in `cmake/EngineLayers.cmake`, so a module here may depend only on
other `foundation` modules.

## Dependencies

- No engine module. `execution` sits below everything, so it links nothing from the tree.
- `sushiruntime` (interface, `runtime` backend only) — SushiRuntime's task graph is one
  implementation of the seam; the module links it and defines
  `SUSHIENGINE_EXECUTION_BACKEND_RUNTIME` when the configure selects that lane.
- `sushiengine_execution_native` and `Threads::Threads` (`native` backend only) — the SYCL-free
  lane. Two symbols cannot live in a header (a thread pool owning real OS threads, and the
  directed-acyclic-graph compiler's execution loop), so they are compiled into their own target
  and linked into the seam alongside `SUSHIENGINE_EXECUTION_BACKEND_NATIVE`.

`SUSHIENGINE_EXECUTION_BACKEND` must be `runtime` or `native`; any other value fails the
configure.

## Public surface

Headers are relative to `include/SushiEngine/execution/`.

| Header | Declares |
|---|---|
| `access.hpp` | The access algebra: `AccessIntent`, `DeterminismClass` and the intent a node declares over a buffer range. |
| `interval.hpp` | `BufferInterval` — the byte range a hazard is keyed on. |
| `memory.hpp` | The allocation vocabulary a backend satisfies. |
| `node_descriptor.hpp` | One unit of schedulable work and the accesses it declares. |
| `hazard.hpp` | The normative hazard semantic every backend must implement, pinned by a conformance suite. |
| `run_report.hpp` | What a completed graph run reports back. |
| `context.hpp` | Selects the backend and publishes `Context`, `Graph` and `Buffer` under one set of names. |
| `backend/runtime_backend.hpp` | The SushiRuntime implementation of the seam. |
| `backend/native_backend.hpp` | The SYCL-free implementation, for platforms SushiRuntime cannot reach. |
| `backend/native/thread_pool.hpp`, `backend/native/dag_compiler.hpp` | The native backend's two compiled parts. |

`detail/hazard_core.hpp` is the engine's own tracker behind the native backend and is excluded
from the generated documentation.

## Tests

Covered twice, on purpose. `tests/unit/test_hazard_semantics.cpp` and
`tests/integration/test_execution_dynamic_graph.cpp` are the conformance cases; the runtime lane
compiles them into `se_functional_tests`, and `tests/native_execution/CMakeLists.txt` compiles
the same two files out of those directories into `se_native_execution_tests`. One set of cases
built twice is the only arrangement in which the two backends cannot silently drift onto
different coverage.

## Further reading

- [`unified_hazard_model.md`](../../../docs/design/unified_hazard_model.md) — the shared
  execution vocabulary this seam is the engine-side half of.
- [`cross_platform_engineering_plan.md`](../../../docs/design/cross_platform_engineering_plan.md)
  — why a second, SYCL-free backend exists at all.
- [`overview.md`](../../../docs/architecture/overview.md) — where this seam sits between the engine
  and the runtime.
