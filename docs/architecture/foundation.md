# Foundation

This file covers the two foundation-tier concerns every other tier is written against: the
entity-component-system and the execution graph it compiles to, and the single seam the engine
takes its scalar and vector types from.

## 1. The ECS and the system graph

This is where the engine's "the graph behaves like a game engine" thesis is built — on the
runtime, not in it.

**The ECS is typed against the execution seam, not against a runtime.** `World`, `Archetype`,
`Chunk`, and `Schedule` name `Execution::Context`, `Execution::Graph`, and `Execution::Buffer` —
the engine's own vocabulary — and which implementation those denote is a compile-time build
policy (`SUSHIENGINE_EXECUTION_BACKEND=runtime|native`). SushiRuntime's task graph
(`RuntimeBackend`) is the default and the only one exercised in CI; a second, SYCL-free
implementation (`NativeBackend`,
`engine/foundation/execution/include/SushiEngine/execution/backend/native_backend.hpp` plus the
compiled `sushiengine_execution_native` library) exists for platforms SushiRuntime cannot reach
(RUNTIME-PORT1) — a thread pool dispatching whole ready nodes concurrently (node-granular, not
element-granular: the ECS's one-node-per-chunk shape is thousands of small independent nodes,
which is where this backend's parallelism comes from) over `Execution::Detail::HazardCore`'s own
ordering.

The seam is compile-time rather than virtual for a concrete reason: a device backend forwards
each kernel into its own launch as the original callable, and a type-erased one cannot be
captured into device code — one binary needs one backend either way, since a SYCL translation
unit already requires that compiler for the whole unit.

**Storage is archetype chunks.** Entities sharing one component set form an *archetype*; an
archetype stores its entities in fixed-capacity *chunks*. Within a chunk each component is a
separate contiguous column backed by its own execution-backend allocation (structure-of-arrays).
A column is therefore a distinct byte interval the hazard tracker keys on, which is what makes
chunks the unit of parallelism: two systems touching different chunks run in parallel, two
touching the same column are ordered — with no scheduler written in the engine.

**A system is a graph node.** A system declares the components it reads and writes (`Read<T>` /
`Write<T>`); the Schedule emits one node per matching chunk, declaring that chunk's column
intervals as compute reads and writes. The backend's hazard tracker *is* the system scheduler: it
derives the ordering from those declarations. A read-after-write on a component orders the two
systems; disjoint access leaves them parallel. What counts as a conflict is fixed by the engine,
not by the backend — the semantic is stated normatively in
`engine/foundation/execution/include/SushiEngine/execution/hazard.hpp` and pinned by a
conformance suite, so two backends may build different edge sets but never disagree about which
pairs must be ordered.

**Counts are late-bound; structure is compiled.** Each node iterates its chunk's live entity
count, re-read every step, so spawning and destroying entities within existing chunks varies the
work with no recompile. The graph is rebuilt only when the *chunk set* changes (a new archetype
or chunk) — reported by the world's `structure_version`. Pre-reserving chunks keeps a steady
spawn/destroy workload at a single compile.

**Structural changes are deferred.** Systems run as device kernels and must never see entities
appear or vanish mid-frame. Gameplay code records spawns and destroys into a `CommandBuffer`
during the frame, and the loop applies them once, at an explicit barrier between steps. Destroy
is an O(1) swap-remove that keeps a chunk's live rows packed; entity handles carry a generation
so a stale handle to a destroyed entity is detected, not silently reused.

The worked example in `samples/sandbox/main.cpp` exercises all of this — Position, Velocity,
Mass, and Lifetime components; `apply_forces`, `integrate`, and a parallel `decay_lifetime`
system; per-frame spawn and deferred destroy — and checks every surviving entity against an
independent scalar reference, with the graph compiled exactly once across the whole run.

## 2. The value-type seam

The engine takes its scalar, vector, matrix, and quaternion types — and the operations on them —
from `engine/foundation/core/include/SushiEngine/core/types.hpp` and nowhere else. Those types
belong to **SushiBLAS** (tensors, and the floats derived from them). Until that library exists,
`types.hpp` aliases a minimal placeholder in
`engine/foundation/core/include/SushiEngine/core/blas_placeholder.hpp`, which now carries
`Vector3`, `Matrix4`, `Quaternion` and the handful of operations the renderer and camera need
(`perspective`, `look_at`, `compose_transform`, `mul`, …). When SushiBLAS lands, re-point
`types.hpp` at it and delete the placeholder — a single-file change, because nothing else in the
engine names the underlying type.

This is the same discipline as [the head and the battery](overview.md#1-the-head-and-the-battery):
one seam, not parallel paths.

The same seam also carries the planet-scale floating-origin types: `WorldVector3` is a double
3-vector for absolute ECEF positions (a distinct type from `Vector3` to mark
absolute-vs-local intent), `SectorCoord` is an integer index of a fixed-size cube ("sector") in
that world space, and `FloatingOriginVector3` pairs a `SectorCoord` with a `Scalar`-precision
local offset from that sector's corner. `to_floating_origin`/`from_floating_origin` convert
between `WorldVector3` and `FloatingOriginVector3` given a sector size. Keeping the local offset
small (at most one sector wide) keeps a fragment's distance from the camera representable when it
narrows to 32-bit at the GPU boundary, regardless of how far the sector is from the world origin.
These types are the SushiLoop M0 foundation (`docs/design/SUSHILOOP.md`) and are not yet consumed
by any simulation code.

The boundary `Scalar` is **always double** — there is no build switch. The engine's purpose is to
simulate planet- and solar-scale worlds, where single precision quantises camera and transform
math to roughly a metre at 10 000 km out (float32 carries ~7 significant digits), making it
unusable at the seam; double is the engine's one and only `Scalar`. The placeholder's `Float` is
a plain `using Float = double`
(`engine/foundation/core/include/SushiEngine/core/blas_placeholder.hpp`), the sole reader of the
choice.

The vector and quaternion types are, however, element-parametric
(`Vector3T<T>`/`QuaternionT<T>`), and the physics layer templates on that element
(`RigidBodyT<T>`, `XPBDDistanceConstraintT<T>`), so the **simulation's physics-solve precision is
a separate runtime choice** (`Simulation::Precision`): both a float and a double solver are
compiled into `sushiengine_simulation` and `create_simulation(Precision)` picks one behind
`Simulation::IPhysicsScene`, letting the editor switch physics precision live (rebuilding the
world from a scene snapshot) without a rebuild of the binary. `sushiengine_render` shares the
value types (across `MeshInstance`/`CameraView`) without linking the engine target — it links the
runtime otherwise. The Vulkan upload path narrows to 32-bit explicitly at the push-constant
boundary, camera-relative, so absolute double positions never reach the GPU as a raw cast.
