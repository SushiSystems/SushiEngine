# Terrain

This file covers planetary terrain: the cube-sphere quadtree and its precision discipline, the
editable layer stack, the height-source and pack formats, the per-frame node selection, and the
render path that draws a body as one instanced call.

## 1. Planetary terrain (foundation and vertical slice, phases P0–P2b)

The engine's ground has always been analytic: `PlanetParameters` is two colours and a roughness,
and `sky.frag` paints a body from noise about its pole. That is enough to read as a lit sphere
from orbit and nothing like enough to stand on, which is why `docs/design/atmosphere_system.md`
§15 records the missing terrain height field as the blocker for its Phase D.
`docs/design/solar_system_overhaul.md` is the design that closes it;
`engine/domain/terrain/include/SushiEngine/terrain/` is its first phase, and it is host-only — no
Vulkan header sits beneath any of it, because the server, the atmosphere's nest, and a unit test
all have to be able to ask what the ground is doing without a renderer present.

A body is a **cube-sphere quadtree**. Six square faces, each subdivided uniformly, gives square
tiles, integer address arithmetic, and no pole singularity — the three properties an
equirectangular grid cannot supply at once. `tile_address.hpp` is that arithmetic and nothing
else: `TileAddress` (a face, a depth, a cell), parent, child, the same-face neighbour, a packed
key, and the sample layout a tile carries — a 129×129 grid plus a one-texel apron of the
neighbours' data, so a central-difference normal or a bilinear tap at a tile's edge never needs a
second tile resident.

`cube_sphere.hpp` is where a tile acquires a position. Grid coordinates are warped by
`tan(t·π/4)` before projection, which makes the angular size of a cell nearly uniform across a
face instead of varying by about 1.4× between its centre and its corners, and the warp fixes ±1
exactly so adjacent faces meet. The projection onto a body's reference ellipsoid is a
componentwise scale of a unit direction, which is the exact surface rather than an approximation
of it, and elevation displaces along the **geodetic** normal — the datum every elevation model
states its heights against, and on Earth up to 0.19° away from the radial direction.

The same file answers the question that makes single-precision geometry possible at planet scale.
A vertex on Earth is 6.37 × 10⁶ m from the body centre and float32 carries about 1.2 × 10⁻⁷ of
relative precision, so any planet-space quantity built in float32 arrives about 0.76 m off —
against a 0.075 m cell at the deepest addressable level. Subtracting the camera afterwards does
not recover it; the error is already in the operand.

`normalized_difference` never forms the large quantity: given a node centre held in double on the
host and a small offset from the grid, the identity `|c| − |g| = (|c|² − |g|²)/(|c| + |g|)`
rearranges the difference of the two normalizations into a form with no subtraction of
nearly-equal quantities left in it. The result holds about a micrometre at depth 20, in float32,
with one code path for every depth. Because the ellipsoid scaling is linear it commutes with the
difference, so `ellipsoid_point_delta` inherits the property rather than re-deriving it.

Ground is **editable**, and that is a property of the pipeline rather than a feature on top of it.
A `TerrainLayer` (`layer_stack.hpp`) is a small authored record — a footprint on the sphere, an
operation, a profile — not an edited raster: a crater is a direction, a radius, and a
bowl-and-rim profile; a building pad is a flatten. Records are bytes rather than megabytes, which
is what lets an edit replicate over a network, serialise into a scene, and be undone.
`TerrainLayer::order` is explicit and unique within a stack, and a duplicate is refused rather
than resolved, so the composed ground is a pure function of the *set* of layers and not of the
order they arrived in — the property a server and a client need to agree on a collidable surface.

**Where an edit comes from.** A stack nothing writes to is a stack that composes nothing, so the
records have an authoring surface:
`engine/domain/terrain/include/SushiEngine/terrain/terrain_authoring.hpp` declares
`ITerrainAuthoring` — insert, rewrite, remove, reorder, and the last frame's selection — and
`PlanetTerrain` (`engine/presentation/render/source/terrain/planet_terrain.*`) implements it,
reached from a host through `ISceneView::terrain_authoring()` and drawn by the editor's Terrain
window (`applications/editor/source/terrain/`).

The interface exists rather than the stack being handed out directly because an edit is two
things, not one: the record changes, and every tile already compiled from the old record is now
wrong. Each mutator marks the footprints it touched, and the next frame turns those into a queue
of resident tiles and re-stages them into the slots they already hold — re-staging rather than
evicting, because a resident tile keeps its slot, so no frame in flight has an image pulled out
from under a draw it has already queued and no device idle is needed. The queue is drained under
the same per-frame upload budget as a streaming miss and ahead of it, since ground that is wrong
is worse than ground that is coarse. The stack belongs to the body it was authored against and is
dropped when the view travels to another one.

`height_source.hpp` is the seam that keeps real and invented bodies on one code path: a baked
pak, a procedural generator, and a higher-resolution regional inset are three implementations of
`IHeightSource`, and no consumer learns which one answered. Its `data_depth` is the other half of
the contract and the more important one — it says how deep the *measurement* goes, so the system
can always distinguish ground it measured from ground it invented.

`height_function.hpp` composes the two, and is the authoritative definition of a body's ground:
the physics collision patches are evaluated from it, the headless server evaluates it with no
renderer, and the tile compile shader that lands with the render path is a *port* of it held to a
stated tolerance by a conformance test rather than assumed to agree. It evaluates whole tiles
rather than points, deliberately — a point query would invite exactly the per-column sampling
pattern that made the shipped weather non-spatial.

**The asset.** `se planet bake` writes one `.planet` pack per body per quality tier, and
`pack_format.hpp` is the authority on its layout — the Python writer under
`cli/sushiengine/services/planet/` is a transcription of that header, not a shared schema, for the
same reason the climatology asset gives: a schema shared between a tool and an engine header is a
third thing to keep in step with both. `PlanetPack::adopt` takes bytes and validates all of them
before accepting any — magic, version, the tile geometry the pack was baked for, that the index
ascends strictly (which is what licenses the binary search), and that every payload lies inside
the blob at the length its codec requires. A pack that fails leaves the object unloaded rather
than partly loaded, and an unloaded pack is not an error: the body falls back to the analytic
ground, which is what shipped before terrain existed.

Elevations are 16-bit fractions of a per-tile range rather than floats. That halves the asset and
costs nothing that matters — the quantisation step is the tile's own relief over 65535 — and it is
what makes the bake's accuracy claim a *number*: the bake re-reads its own output and compares
against the source raster, refusing to report success if anything came back further off than that
step. `PackHeightSource` is the policy over the format: when the quadtree descends past the depth
the pack stores, it resamples the nearest stored ancestor, which is where measurement stops and
where detail synthesis will later begin.

Two disciplines in the baker are worth naming, because both guard against errors that look like
success. It reports the depth the **source** supports rather than the depth it was asked for, so
nothing downstream mistakes resampled levels for measurement. And it verifies the raster's grid
convention against known landmarks before baking anything — a longitude read backwards produces a
planet that is entirely plausible to look at and wrong everywhere.

**Which patches get drawn.** `engine/domain/terrain/include/SushiEngine/terrain/quadtree.hpp`
produces the cut of a body's quadtree for a frame, and the shape of the algorithm is the
interesting part. It does not descend the tree; it *refines* it. The six root faces already cover
the body, and selection repeatedly replaces whichever node is furthest over the screen-space error
target with its four children. Because splitting a cut yields a cut, the result covers every point
of the body exactly once at every stage — including the stage a node budget stops it at, which is
what turns "we ran out of budget" into coarser terrain rather than a hole. A recursive descent
cannot make that promise: it commits to refining a subtree before it knows whether it can afford
to emit all of it.

Each selected node carries the camera-relative frame the single-precision vertex path consumes and
the distance band over which it morphs into its parent, so nothing downstream recomputes either.
The selection is host-side and double-precision because a node centre is a planet-scale coordinate
and this is the one place in the terrain path allowed to hold one — and because the same cut is
what the collision patch set and the builder's placement queries need, neither of which has a
renderer to ask.

Node bounds come from the pack index rather than from tile decodes
(`IHeightSource::tile_bounds`, a capability with a default): a cull tests far more nodes than it
draws, so a bounding volume that cost a tile read would be more expensive than the drawing it
saves.

**Getting it onto the screen.** A body is one draw: the visible nodes go into a storage buffer and
`TerrainPass` (`engine/presentation/render/source/passes/terrain_pass.*`) issues a single
instanced call of a shared 33×33 lattice, so the host cost of terrain is the selection and nothing
else. Set 0 is full at 32 bindings, so terrain owns **set 2** and builds its own pipeline layout
around the scene's sets 0 and 1 — which is also why it binds the bindless heap itself rather than
through `SceneLayout`: Vulkan set compatibility requires identical push-constant ranges, and
terrain's is not the mesh path's. Because
`engine/presentation/render/shaders/terrain.vert` matches `mesh.vert`'s output signature exactly,
`pbr.frag` shades terrain without knowing it is terrain, and terrain inherits the whole lighting,
shadowing, IBL and tone-mapping path rather than duplicating any of it.

Inheriting the shading path means inheriting its *whole* descriptor set, and that is a sharper
obligation than it sounds. Set 0 is a push-descriptor set, so a pass that writes only the
descriptors its own shaders name leaves the rest undefined — and `pbr.frag` samples them anyway,
which costs the device rather than a pixel.
`engine/presentation/render/source/passes/shading_set.hpp`
is therefore one shared pair: `declare_shading_set` registers the graph reads,
`write_shading_set` fills the twenty-six descriptors, and the opaque, transparent and terrain
passes all call both. The same obligation covers the material array: terrain pushes a material per
frame rather than naming index zero, because an index into an array nothing was pushed to reads
bytes, not a default.

Heights live in a 2D array image of fixed-size slots, indexed by `TileResidency`
(`engine/domain/terrain/include/SushiEngine/terrain/tile_residency.hpp`) — an LRU over *anonymous*
storage, which is what lets a node whose own tile has not arrived bind a coarser ancestor's slot
through a scaled UV rectangle and draw a correct, coarser surface instead of nothing. Eviction
spares every slot bound within the frames-in-flight window, because the write and the read are in
different submissions and the frame graph derives barriers within one submission only.

The frame seam is `engine/presentation/render/source/terrain/terrain_frame.hpp`, and it is the
piece that took the longest to get right. Terrain lives in the body's **fixed** frame — where the
elevations were baked and where the ellipsoid sits at the origin — while the scene is anchored at
an observer's surface point. `Environment::planet_body_axes` carries the rotation between them,
filled by the ephemeris; the crossing itself (camera position, the shader's matrix, the frustum
rotated into body-fixed axes) is header-only and Vulkan-free so that its three silent conventions
can be tested without a device. Terrain was also the first consumer able to detect the
prime-meridian frame error described in
[the astro coordinate spaces](domain-astro.md#1-the-solar-system-ephemeris-gravity-and-frames),
for the reason that makes such errors survive: until something has an opinion about *where a real
place is*, a uniformly rotated planet under a uniformly rotated sky is indistinguishable from a
correct one.

Where terrain draws, `sky.frag`'s analytic ellipsoid stands down for that frame. The per-pixel
depth test is not sufficient on its own: the reference ellipsoid wins wherever real elevations dig
below it, which on the Moon is every mare. Because the selection is an exact cover, switching the
analytic ground off for a body that has terrain leaves no pixel uncovered; a body with no pack
keeps it.

What is not here yet: the streamer, terrain in the depth prepass (so Hi-Z and GTAO do not see it),
the collision patch set, and sub-Nyquist detail synthesis. Each is a later phase in the design
document, and each has an exit criterion there rather than in prose. Layers are also not
persisted: the scene file carries entities, the environment and the sky, and a layer stack an
author builds lives only in the view that drew it, so it is lost on a reload and on a trip to
another body. The record was designed to serialise — that is what makes it a record — and giving
it an owner in the scene is the step that closes the loop.
