# Model Import — a glTF file as an entity hierarchy (`SushiEngine::Asset`)

**Status:** designed, 2026-08-06 (§13).

A glTF file reaches the engine today through three doors, and every one of them throws the file's
structure away. The cooking pipeline merges the whole file into a single `Geometry::TriangleMesh`.
The renderer's importer bakes each node's world transform into its vertices and returns a flat list
of primitives. The Renderer component binds exactly one of those primitives and ignores the rest.
A model authored in Blender as a root with five objects, each carrying two children, therefore
arrives in the scene as one piece of one object.

This document specifies the import path that keeps the structure: a glTF node graph becomes an
entity hierarchy, with the pivots, names, per-node materials, lights and cameras the file actually
declares. It also specifies the per-asset import settings that drive it, stored beside the asset
rather than in a project-wide table keyed by path.

Companion docs: `static_mesh_authoring.md` §10, which deferred exactly this work and whose two
deferrals this document takes up; `physics_system.md` §3.4, which explains why the importer sits
below the renderer; and `editor_ux_overhaul.md`, whose asset-centric direction the follow-on
inspector work continues.

---

## §1 Audit — what exists today, and where it stops

Traced file by file rather than assumed from a name.

- **The engine already has entity parenting, and the editor already exposes it.**
  `Simulation::IWorldEditor::parent`/`set_parent`
  (`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp:1250-1263`) maintain the
  tree and reject a cycle. The Hierarchy panel nests rows by it and reparents by drag
  (`applications/editor/source/scene/hierarchy_panel.cpp:144-179`). Nothing is missing on the
  consuming side; what is missing is the step that produces a tree from a file.
- **The cooking importer flattens the file into one mesh.** `Geometry::import_gltf_mesh`
  (`engine/asset/gltf/source/mesh_importer.cpp:129-144`) walks nodes, applies
  `cgltf_node_transform_world` and appends every primitive of every node into a single output. Its
  header says so plainly: "All primitives of all meshes are merged"
  (`engine/asset/gltf/include/SushiEngine/gltf/mesh_import.hpp:80-84`). That is correct for a
  collider, which is a property of the whole model, and useless for a hierarchy.
- **The render importer flattens differently — into the vertices.**
  `engine/presentation/render/source/material/gltf_importer.hpp:26-36`: "Each primitive becomes one
  mesh, baked into its node's world transform so a multi-part asset assembles correctly without a
  scene graph on the render side." The node's placement survives only as displaced vertex data, so
  two nodes that reference one glTF mesh at two transforms produce two separate uploads and share
  nothing.
- **Only the first primitive is ever bound.** `bind_shape_mesh`
  (`applications/editor/source/scene/inspector_panel.cpp:178-193`) calls
  `load_gltf(..., capacity = 1)` and assigns `meshes[0]`. `resolve_scene_assets` does the same on
  load (`engine/world/serialization/source/scene_serializer.cpp:1530`). A file with twenty-two
  primitives contributes one.
- **`ShapeParameters` holds one mesh, and material is a separate component.**
  `simulation.hpp:436-442` carries a single `Render::MeshId`; the entity's `Material` is set through
  `IWorldEditor::set_material` (`simulation.hpp:1133`). One entity is therefore one mesh with one
  material, which is the constraint §5's node-splitting rule exists to satisfy.
- **Per-asset settings exist, cover cooking only, and are keyed by path.**
  `Physics::Cooking::ImportProfileOverride`
  (`engine/domain/physics/include/SushiEngine/physics/cooking/import_profile.hpp:110-134`) holds
  five optional fields. `CookBakeState` persists them into one project-level JSON document whose
  `overrides` object is keyed by the asset's path string
  (`engine/world/authoring/source/cook_bake_state.cpp:274,281-298`). Renaming or moving an asset
  orphans its settings silently: nothing reports it, and the asset reverts to the project default.
- **The asset drag source already exists; the drop target does not.**
  `set_asset_drag_source` (`applications/editor/source/ui/panel_widgets.cpp:46-53`) publishes an
  `ASSET_PATH_PAYLOAD` (`applications/editor/source/ui/panel_widgets.hpp:59`) for every file tile
  in the Project panel (`applications/editor/source/project/project_panel.cpp:459`), and
  `accept_asset_drop` (`panel_widgets.cpp:56-67`) is the matching consumer.
  `static_mesh_authoring.md` §10 states that no drag source exists; that sentence is stale, and §12
  of this document corrects it. No drop target for an asset path exists on the viewport or the
  Hierarchy panel.
- **Every glTF file is treated as a rigged character.** `has_character_extension`
  (`project_panel.cpp:62-68`) returns true for any `.gltf` or `.glb`, and double-clicking one calls
  `open_character_in_preview` (`:85-104`), which loads it into the animated preview and logs
  "Loaded character '…' into the preview." — or, on failure, "Could not load '…' as a rigged
  character." A static prop is neither loaded nor named correctly.
- **The engine has entity-level lights and cameras, but no entity-level directional light.**
  `create_light`/`LightParameters` (`simulation.hpp:1625`, `:453-462`) cover point and spot only —
  the field is a `bool is_spot`, with no third case. The scene's directional light is an
  `Environment` property, `DirectionalLight sun`
  (`engine/domain/environment/include/SushiEngine/environment/environment.hpp:841`), not a
  component. `create_camera`/`CameraParameters` (`simulation.hpp:1285`, `:186`) do exist.
- **The tier order forbids the obvious shortcut.** `SUSHIENGINE_LAYER_ORDER`
  (`cmake/EngineLayers.cmake:8-11`) is `foundation`, `domain`, `asset`, `presentation`, `world`, and
  `gltf` is an `asset` module (`:35`). An importer in `asset` cannot name `Render::MeshId`, so the
  node graph and the uploaded meshes must be joined by something both sides can derive
  independently. §6 makes that the glTF node index.
- **The by-path import cache is already in place.** `AssetLibrary::load_gltf`
  (`engine/presentation/render/source/material/asset_library.cpp:95`) caches its result per path
  (`asset_library.hpp:227-257`), so several entities importing one file share one `MeshId` and the
  opaque pass's per-geometry grouping (`opaque_pass.cpp:454`) applies. §6's new entry point inherits
  that cache rather than opening a second one.

## §2 Non-goals

- **No prefab system.** The imported hierarchy is not a prefab instance: there are no per-instance
  overrides, no nested prefabs, and no automatic propagation when the asset changes. The link is
  the two fields of §4.3 and the explicit Reimport action of §7. Anything more is a separate
  subsystem with its own override-resolution and merge semantics.
- **No rigged-model path.** A node carrying a skin is reported and skipped. Skinned import already
  has its own machinery (`import_gltf_skinned_mesh`, `import_gltf_animated`) reached through the
  Crowd component, and joining it to a node hierarchy is a distinct problem.
- **No asset inspector.** The Bake window and the Cooking Override modal stay where they are. The
  Unity-style "select an asset, edit its importer in the Inspector" surface is the next sub-project
  and consumes §4.2's settings type; building the surface before the type it edits would mean
  building it twice.
- **No new material system work.** Materials are read exactly as the renderer's importer already
  reads them. Whether a glTF material maps onto every engine material lobe is an audit this
  document does not perform.
- **No change to `PrimitiveKind` or to the cooking pipeline's own stages.** §7's collider generation
  calls the existing pipeline; it does not modify it.

## §3 Module layout

Three parts, and no part can see the next one.

```
engine/asset/gltf/            existing module; owns cgltf
  include/SushiEngine/gltf/scene_import.hpp     new
  source/scene_importer.cpp                     new
      A glTF file's node graph, as the file states it. No settings, no engine
      policy, no renderer type. Fidelity only.

engine/asset/model/           new module; no cgltf, no device
  include/SushiEngine/model/import_settings.hpp
  include/SushiEngine/model/instantiation_plan.hpp
  source/import_settings_io.cpp                 the .meta sidecar
  source/instantiation_plan.cpp                 the pure decision function
      What the settings say to do with what the file says. Hierarchy shape,
      node splitting, scale, axis, naming, and the report.

applications/editor/source/project/model_instantiate.{hpp,cpp}   new
      Executes a plan against IWorldEditor and IAssetLibrary. Decides nothing.
```

The split is not decoration. Everything difficult — which node becomes which entity, when a node
splits, how a dropped pivot folds its transform into its children, how a name collision resolves —
lives in `instantiation_plan.cpp`, which links no graphics stack and no editor and is therefore
unit-testable on a machine with no GPU. What is left in the editor is a loop of `create` and
`set_parent` calls with no branches worth testing, kept small for the reason
`applications/editor/source/physics/cook_bake_panel.hpp` already states about itself: it is the part
that cannot be tested.

`model` depends on `gltf` publicly, because `instantiation_plan.hpp` names `GLTFSceneDescription` in
its signature, and on `geometry` and `physics` for the transform type and the embedded
`ImportProfileOverride` of §4.2. An `asset` module may depend on another `asset` module
(`engine/asset/gltf/README.md`, "Tier"), and the dependency costs nothing at the parser level: cgltf
is a private include directory of `gltf` and no header there names a cgltf type, so `model` takes
the description type without taking the parser.

## §4 Data model

### §4.1 `GLTFSceneDescription` — what the file says

```cpp
namespace SushiEngine::Geometry
{
    struct GLTFNodeDescription
    {
        std::string name;               // the node's own name, or "" when unnamed
        std::int32_t parent = -1;       // index into GLTFSceneDescription::nodes; -1 is a root
        std::uint32_t source_index = 0; // the glTF file's own node index — §6's join key
        Vector3f translation{0, 0, 0};
        Quaternionf rotation{0, 0, 0, 1};
        Vector3f scale{1, 1, 1};
        std::int32_t mesh = -1;         // glTF mesh index, or -1
        std::uint32_t primitive_count = 0;
        std::int32_t camera = -1;
        std::int32_t light = -1;        // KHR_lights_punctual, or -1
        std::int32_t skin = -1;
    };

    struct GLTFSceneDescription
    {
        std::vector<GLTFNodeDescription> nodes;   // parents always precede their children
        std::vector<GLTFLightDescription> lights; // colour, intensity, range, cone, kind
        std::vector<GLTFCameraDescription> cameras;
        std::uint32_t material_count = 0;
        std::uint32_t skin_count = 0;
    };

    bool import_gltf_scene(const char* path, GLTFSceneDescription& out);
}
```

Nodes are emitted parent-before-child so a consumer builds the tree in one pass and never holds a
forward reference. `parent` indexes this vector; `source_index` is the file's own numbering and is
the only identity §6 shares with the renderer.

The transform is kept as translation, rotation and scale rather than a matrix. glTF permits either
form and cgltf can decompose, and the decomposed form is what `IWorldEditor::set_transform`
(`simulation.hpp:1057`) takes — converting to a matrix here only to decompose it again in the
caller would lose precision for nothing.

### §4.2 `ModelImportSettings` and the `.meta` sidecar

```cpp
namespace SushiEngine::Model
{
    enum class AxisConvention { AsAuthored, BlenderZUp };

    struct ModelImportSettings
    {
        float scale_factor = 1.0f;
        AxisConvention axis_convention = AxisConvention::AsAuthored;
        bool import_materials = true;
        bool import_lights = true;
        bool import_cameras = true;
        bool preserve_pivots = true;
        bool generate_colliders = false;
        Physics::Cooking::ImportProfileOverride cooking;

        std::uint64_t hash() const noexcept;   // §4.3's staleness key
    };
}
```

Settings are stored in `<asset>.meta` beside the asset — `models/Car.gltf` is described by
`models/Car.gltf.meta`. This replaces the path-keyed `overrides` object of §1: a setting written
beside its asset is moved, renamed, copied and version-controlled with the asset instead of being
orphaned by the first of those operations. The Project panel filters `.meta` out of its listing, the
same way it already special-cases extensions in `tile_color` and `has_text_extension`
(`project_panel.cpp:136-152,210-220`).

Migration runs once, on `CookBakeState::load_profiles`: every entry in the existing `overrides`
object whose asset still exists is written out as a `.meta` file carrying that override in its
`cooking` field, and the `overrides` object is dropped from the project document, which keeps only
`project_default`. An entry whose asset no longer exists is discarded and reported — it was already
dead, and saying so is the honest outcome rather than carrying it forward invisibly.

`hash()` covers every field including the embedded override. It is a value hash, not a file hash: it
answers "were these the settings this subtree was built from", which is what §7's Reimport prompt
needs, and it does not change when an unrelated asset's `.meta` does.

### §4.3 `ModelSourceParameters` — the link back to the asset

```cpp
namespace SushiEngine::Simulation
{
    struct ModelSourceParameters
    {
        std::string path;                 // the .gltf or .glb this subtree was built from
        std::uint64_t settings_hash = 0;  // ModelImportSettings::hash() at instantiation
    };
}
```

Carried by the root entity of an imported subtree and by nothing else, with the accessor triple
every other component already has on `IWorldEditor` — `has_model_source`, `model_source`,
`set_model_source` — and a round trip through `scene_serializer.cpp` beside the existing component
blocks. Two fields are the whole linkage mechanism: the editor compares the stored hash against the
current `.meta`'s and can then say which instances in the open scene are out of date.

## §5 The instantiation plan

`plan_model_instantiation(const GLTFSceneDescription&, const ModelImportSettings&)` returns a
`ModelInstantiationPlan` and a `ModelImportReport`. It reads no file, touches no device, and is the
only place a decision is made.

```cpp
struct PlannedEntity
{
    std::string name;
    std::int32_t parent = -1;          // index into PlannedEntity list; -1 is the subtree root
    EntityTransform local;
    PlannedComponent component = PlannedComponent::None;  // Shape, Light, Camera, or None
    std::uint32_t source_node = 0;     // glTF node index this came from
    std::uint32_t primitive = 0;       // which primitive of that node's mesh, for Shape
    std::int32_t light = -1;           // index into GLTFSceneDescription::lights
    std::int32_t camera = -1;
};
```

The rules, in the order they apply:

1. **Every node becomes an entity**, keeping its name and its local transform. An unnamed node is
   named after its glTF index (`"node 7"`) rather than left blank, because a blank row in the
   Hierarchy panel is unselectable in practice.
2. **A node whose mesh has exactly one primitive carries the `Shape` itself.** No extra entity.
3. **A node whose mesh has more than one primitive stays a plain transform and gains one child per
   primitive**, named `"<node name> (0)"`, `"<node name> (1)"`, and so on. This is forced by §1's
   constraint that an entity holds one mesh and one material: a node with two materials cannot be
   one entity. The children carry identity transforms, so the visible pivot is still the node's.
4. **A node carrying a light** gains a `Light` when `import_lights` is set. A **directional** light
   is skipped and counted: the engine's directional light is an `Environment` property (§1), not a
   component, and silently dropping the node or silently rewriting the scene's sun would both be
   worse than saying so.
5. **A node carrying a camera** gains a `Camera` when `import_cameras` is set. Imported cameras are
   created inactive; a file that declares three cameras must not fight the editor's own view for
   which one is rendering.
6. **A node carrying a skin** is reported and its mesh is skipped (§2). The node itself is still
   created, so the hierarchy keeps its shape and the omission is visible rather than a hole.
7. **`preserve_pivots == false` drops a node that carries no mesh, light, camera or skin and is not
   the subtree root**, folding the dropped node's local transform into each of its children so the
   composed world transform is unchanged. The default is `true`, because a dropped pivot is what
   makes a wheel impossible to rotate and a door impossible to open.
8. **A single glTF root becomes the subtree root. Several roots gain a synthetic root** named after
   the file stem, so the result is always one selectable, one movable, one deletable thing.
9. **`scale_factor` and `axis_convention` apply to the subtree root only.** Applying either per node
   would compound it once per level of the hierarchy. `BlenderZUp` contributes a -90 degree rotation
   about X to the root's transform.
10. **Names are made unique among siblings**, not globally: two wheels may both contain a node named
    `Tire`, and renaming one of them would misdescribe the file.

## §6 The render seam

The plan names a `(source_node, primitive)` pair; the renderer must return a `MeshId` for that pair.
`IAssetLibrary` gains one call beside `load_gltf`:

```cpp
struct ImportedPrimitive
{
    std::uint32_t source_node = 0;   // the glTF node index — matches PlannedEntity::source_node
    std::uint32_t primitive = 0;
    MeshId mesh = INVALID_MESH;
    Render::Material material;
};

std::size_t load_gltf_scene(const char* path, ImportedPrimitive* out, std::size_t capacity);
```

Two properties matter, and both differ from `load_gltf`:

- **It reports provenance.** Matching the plan to the uploaded meshes by walk order would require
  two independent parsers to agree on traversal forever; they would not, and the failure would be a
  wheel wearing a door's material. The glTF node index is a fact of the file, derived independently
  on both sides.
- **It does not bake the node's world transform into the vertices.** The entity's transform carries
  the placement now, and baking it as well would apply it twice. This also makes the upload
  shareable: four wheel nodes referencing one glTF mesh yield one `MeshId` rather than four, which
  is what lets the opaque pass's per-geometry grouping (`opaque_pass.cpp:454`) actually group them.

`load_gltf` keeps its current behavior and its callers unchanged; the two entry points share the
existing per-path cache (`asset_library.hpp:227-257`), keyed additionally on which of the two
produced the entry, since their vertices differ.

## §7 Editor wiring

Deliberately narrow, because the asset inspector is the next sub-project (§2).

- **Drag and drop.** A `.gltf` or `.glb` tile dropped on a Hierarchy row instantiates under that
  entity; dropped on Hierarchy empty space or on the viewport, it instantiates at the scene root.
  Both use the payload and helper that already exist (§1) — this adds drop targets, not a protocol.
- **Double click.** The file is inspected first. When it declares a skin it goes to the animated
  preview as it does today; otherwise it is instantiated into the scene. This is where §9's naming
  correction lands.
- **Reimport.** A context-menu action on a root entity that carries `ModelSourceParameters`:
  destroys the subtree, re-runs the plan, and rebuilds it under the same entity, preserving that
  entity's own name and transform. Edits made inside the subtree are lost, and the menu item's
  tooltip says so before the click rather than after.
- **Undo.** An instantiation and a reimport are each one `Authoring::CommandHistory` step. Twenty
  entities appearing from one drag must disappear from one undo.

## §8 Errors and the import report

Every failure produces either an entity or a report line. None is silent.

```cpp
struct ModelImportReport
{
    std::uint32_t nodes = 0, entities = 0;
    std::uint32_t primitives_imported = 0, primitives_skipped = 0;
    std::uint32_t lights_imported = 0, lights_skipped_directional = 0;
    std::uint32_t cameras_imported = 0;
    std::uint32_t skinned_nodes_skipped = 0;
    std::uint32_t pivots_dropped = 0;
    std::uint32_t materials = 0;
    std::vector<std::string> warnings;
};
```

An unreadable file creates nothing and logs one warning naming the path and cgltf's own reason. A
node whose primitives are all non-triangle becomes a plain transform and is counted, the same
distinction `GLTFMeshImportReport` already draws (`mesh_import.hpp:63-70`). A `.meta` that fails to
parse is reported and the project default is used for that import — not silently, because an artist
whose settings stopped applying needs to know it was the file and not the setting.

A partial import stays partial and says what is missing. Reporting a model that arrived as half its
parts as a success is the failure mode `documentation-style-guide.md` names under "Honest about
gaps", and it is the same reason `GLTFMeshImportReport::primitives_skipped` exists rather than the
importer quietly ignoring what it could not read.

## §9 Not every glTF is a character

The correction is a rename plus a branch, and it is in scope here because §7 changes the same call
site:

- `has_character_extension` becomes `is_model_extension` (`project_panel.cpp:62-68`), and its
  comment stops describing every glTF as "a rigged character asset".
- `open_character_in_preview` becomes `open_model_asset` (`:85-104`) and branches on
  `GLTFSceneDescription::skin_count` rather than on the extension.
- The log line stops asserting a category the file has not claimed. A static model reports what
  arrived — nodes, primitives, materials, and anything skipped — instead of "Loaded character".
- The failure line stops saying "Could not load '…' as a rigged character" for a file that was never
  offered as one.
- `docs/` is swept for the same claim and corrected wherever prose asserts that a glTF import is a
  character import.

## §10 Testing

The weight is on the pure layer, which needs no device:

- `plan_model_instantiation` unit tests over synthetic descriptions: a three-level hierarchy
  produces the right parent indices; a two-primitive node splits into two named children with
  identity transforms and its own pivot intact; `preserve_pivots == false` folds a dropped node's
  transform into its children and the composed world transform is unchanged; `scale_factor` reaches
  the root and no other entity; a directional light is skipped and counted; sibling name collisions
  disambiguate while cousins keep their shared name.
- `.meta` write-read-compare round trip, including a fully defaulted settings object and one with
  every field set, plus the one-way migration from a project document's `overrides`.
- `import_gltf_scene` integration test against a real asset through `SE_TEST_ASSET_DIR`, the
  mechanism `tests/unit/test_animation_morph_import.cpp` already uses: node count, parent links, and
  names. This closes part of the coverage gap `engine/asset/gltf/README.md` states about itself.
- `ModelSourceParameters` round trip through `test_scene_serializer_roundtrip.cpp`, beside the
  existing component cases.

## §11 Open verification items

Recorded rather than assumed, and closed during implementation before the code that depends on them
is written:

1. **Does a cooked collider respect its entity's scale?** §5 applies `scale_factor` to the root
   transform, which is correct for the visual mesh. A collider cooked from file-space geometry is
   only correct at that scale if the physics side applies the entity's scale too. If it does not,
   `generate_colliders` must carry the scale into the cook instead, and this document is amended
   rather than the discrepancy shipped.
2. **Does cgltf expose `KHR_lights_punctual` in the vendored version?** §4.1 assumes
   `cgltf_node::light`. If the pinned version does not, importing lights moves to §12 and
   `import_lights` is not offered rather than offered and inert.

## §12 Future work, explicitly deferred

- **The asset inspector.** Project-panel selection fills the Inspector with the asset's importer
  sections, and the Bake window and Cooking Override modal fold into it. The next sub-project, and
  the direct consumer of §4.2.
- **Rigged models in a hierarchy.** A skinned node inside an imported tree (§2).
- **glTF feature coverage audit.** A claim-by-claim comparison of what the engine reads against
  glTF 2.0 and the `KHR_*` extensions, which is what "as complete as Unity's FBX support" has to
  mean before it can be checked.
- `static_mesh_authoring.md` §10's two deferrals are taken up here: multi-primitive import by §5,
  drag and drop by §7. That section's claim that "no `ImGui::BeginDragDropSource` exists today" is
  stale — one was added since — and is corrected as part of this work.

## §13 Roadmap

P0 — §3 to §10 — **designed, not started.** The implementation plan is written separately, per the
precedent of `docs/superpowers/plans/2026-08-05-static-mesh-authoring.md`.
