# Physics {#module-physics}

`physics` owns what a simulated body is: the colliders and shapes it is made of, the constraints
that hold it together, the solvers that move it, the soft-body and vehicle models built on those
solvers, and the cooking seam that turns an imported mesh into any of them. One compliant
constraint framework carries rigid bodies, articulated assemblies, cloth and volumetric soft
bodies, so a new shape does not mean a new solver.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `core` (public) — every position, orientation and inertia tensor is the engine's own value
  type.
- `execution` (public) — the solvers schedule their colour sweeps through the execution seam.
- `geometry` (public) — the collision and cooking paths read a `TriangleMesh` and its distance
  hierarchy.

The module is header-only, so its kernels instantiate inside the consuming translation unit —
the safe way to ship device code.

The offline half is built separately as `sushiengine_physics_cooking`, declared only on the
`runtime` execution lane because that is the only lane its consumers are declared on. It is
host-only and never linked into a shipping runtime path, which is what lets it be slow and
allocate freely. It links `sushiengine_physics`, `sushiengine_geometry` and `Threads::Threads` —
the cooking service owns a worker thread so a dropped asset does not freeze the editor. The
glTF importer is deliberately not linked: the service takes a `MeshLoader` seam, so cooking
still builds on a machine with no parser and no device.

## Public surface

There is no umbrella header; a consumer includes what it needs, and the ten subdirectories are
the map. Headers are relative to `include/SushiEngine/physics/`.

| Group | Contents | Declares |
|---|---|---|
| `core/` | `rigid_body.hpp`, `handle.hpp`, `body_flags.hpp`, `material.hpp`, `configuration.hpp`, `statistics*.hpp` | Body state and its handle, the physical material, the world configuration, and the per-step statistics. |
| `geometry/` | `shapes.hpp`, `gjk.hpp`, `closest_point.hpp`, `mass_properties.hpp`, `mesh_mass_properties.hpp`, `mesh_bvh.hpp`, `continuous_proximity.hpp` | The shape set, the convex distance routines, mass and inertia derivation, and the mesh hierarchy. |
| `collision/` | `broadphase.hpp`, `bvh_broadphase.hpp`, `dynamic_bvh.hpp`, `narrowphase*.hpp`, `manifold.hpp` and its five specialisations, `contact.hpp`, `contact_solver.hpp`, `scene_query.hpp`, `conservative_advancement.hpp` | Pair finding, manifold generation per shape pair, the contact record, and scene queries. |
| `constraints/` | `constraint.hpp`, `xpbd_constraint.hpp`, `joint.hpp`, `joint_primitives.hpp`, `beam_constraint.hpp`, `bending_constraint.hpp`, and the `*_projection.hpp` set | The constraint descriptors and the projections that resolve them. |
| `solver/` | `solver_interface.hpp`, `pgs_solver.hpp`, `xpbd_solver.hpp`, `host_solver.hpp`, `graph_coloring.hpp`, `incremental_coloring.hpp`, `constraint_store.hpp`, `contact_store.hpp`, `runtime_graph_builder.hpp` | The solver seam, the projected Gauss-Seidel and XPBD implementations, the colouring that parallelises them, and the graph they are compiled into. |
| `soft/` | Twenty-six headers: `soft_body*.hpp`, `finite_element_model.hpp`, `mass_spring_model.hpp`, `shape_matching_model.hpp`, `fem_*.hpp`, `cloth.hpp`, `mesh_embedding.hpp`, `soft_contact.hpp`, and the `soft_*_collision.hpp` set | The soft-body models behind one seam, the finite-element stress, plasticity and fracture path, cloth, the embedding of a render mesh in a simulated one, and soft-soft and soft-rigid contact. |
| `scene/` | `physics_world.hpp`, `islands.hpp` | The world lifecycle — register, finalize, step — and the island partition it steps in. |
| `vehicle/` | `vehicle_asset.hpp`, `vehicle_instance.hpp`, `node_beam_structure.hpp`, `powertrain.hpp`, `suspension.hpp`, `tyre*.hpp` | The vehicle asset and its instance, the node-beam structure, and the drivetrain, suspension and tyre models. |
| `cooking/` | `cooker_interface.hpp`, `cooking_parameters.hpp`, `cooking_report.hpp`, `cooking_service.hpp`, `cooked_asset_store.hpp`, `collision_*.hpp`, `soft_body_*.hpp`, `node_beam_*.hpp`, `convex_decomposition.hpp`, `tetrahedral_mesh.hpp`, `mesh_post_processor.hpp`, `import_profile.hpp` | The offline pipeline: the fidelity dial, the report and its refusal thresholds, the cooker seams, the content-hash store, and the three cookers that produce collision, soft-body and node-beam assets. |
| `aero/` | `wind.hpp` | The wind field a body is pushed by. |

`SushiEngine.hpp` pulls in the common entry points — the body, shape, broadphase, contact,
constraint, solver, cloth, soft-body and world headers — for a consumer that wants the usual set.

## Tests

The most heavily covered module in the tree: around sixty files across `tests/unit/`,
`tests/integration/` and `tests/regression/` include a physics header.

- Shapes and queries: the `test_gjk`, `test_mass_properties`, `test_mesh_mass_properties`,
  `test_scene_query` and `test_conservative_advancement` unit files.
- Collision: `test_broadphase` and its conformance sibling, `test_collision`,
  `test_collision_scale`, `test_narrowphase_dispatch`, the four manifold files
  (`test_manifold`, `test_convex_manifold`, `test_mesh_collision`, `test_sdf_manifold`),
  `test_height_field_compound`, and the three contact files.
- Constraints and solvers: `test_beam_constraint`, `test_bending_constraint`,
  `test_distance_projection`, `test_joint_projection`, `test_graph_coloring`, `test_islands` and
  `test_physics_statistics` as units, plus `test_pgs_solver`, `test_xpbd_solver`,
  `test_solver_conformance`, `test_runtime_graph_builder` and `test_physics_world` as
  integration.
- Soft bodies: the four `test_fem_*` files, `test_mesh_embedding`,
  `test_soft_body_half_storage` and `test_soft_body_precision` as units, plus `test_soft_body`,
  `test_soft_body_lod`, `test_soft_body_model_conformance`, `test_cloth`,
  `test_finite_element_model` and the three `test_soft_*_collision` files as integration.
- Vehicles: `test_vehicle_powertrain`, `test_vehicle_suspension`, `test_vehicle_tyre`,
  `test_vehicle_aerodynamics` and `test_node_beam_structure`, plus
  `tests/integration/test_vehicle_acceptance.cpp`.
- Cooking: `test_cooking_core`, `test_collision_cooker`, `test_soft_body_cooker`,
  `test_node_beam_cooker`, `test_node_beam_asset` and `test_import_chain`. The test binary links
  `sushiengine_physics_cooking` directly.
- `tests/regression/test_penetration_contract.cpp` holds the penetration contract between the
  visible mesh and the simulated one.

## Further reading

- [`physics_system.md`](../../../docs/design/physics_system.md) — the umbrella design: the
  unified XPBD solver, the cooking pipeline, the penetration contract, and the road to
  deformable vehicles.
- [`domain-physics.md`](../../../docs/architecture/domain-physics.md) — the solver as it stands
  today.
