# Solar System Overhaul — real planetary terrain, from a metre to orbit

**Status:** in progress — P0, P1 and P2a landed 2026-08-01; P2b draws a frame but does not meet its
exit criterion, and §20.1's punch list is P2c; P3 to P11 are unbuilt (§20).

This document specifies the terrain and surface regime for every body in the solar system: where the
elevation comes from, how it reaches the GPU, how it is drawn seamlessly from human scale to orbital
distance, how it is collided with, how it is *edited* at run time, and what it publishes to the
systems that have been waiting on it. Design authority for this pass was delegated by the owner on
2026-08-01, along with ten scoping decisions recorded in §0.2.

**Companion documents.** `atmosphere_system.md` (§15's recorded blocker — *"no terrain height field
exists in the engine"* — is discharged by §14 here; §16's surface-property provider is §14.2);
`unified_hazard_model.md` (the `Execution` vocabulary and the determinism classes §13 uses);
`render_pipeline_refactor.md` (Phase 7's LUT stack and Phase 10's GPU-driven path, both shipped, are
consumed rather than re-planned); `SUSHILOOP.md` (the determinism contract §13 answers to);
`physics_system.md` (the height-field collider §12 feeds).

---

## 0. The verdict, and the decisions it stands on

### 0.1 In one paragraph

A planet is a **cube-sphere quadtree of height tiles**, baked once from public elevation data by
`se planet bake` and *compiled* per node at run time into a fixed pool of GPU slots. The compile
step is where a base tile, an ordered stack of edit layers, and synthesized sub-Nyquist detail
become one height field — which is what makes terrain editable at run time without a second
pipeline, and what makes a crater, a road cut, and a dam the same mechanism. Geometry is CDLOD: one
33×33 grid mesh, instanced once per visible node, morphed per vertex so there are no cracks and no
popping, and projected onto the ellipsoid through a **cancellation-free difference form** (§9) that
holds sub-millimetre accuracy at depth 20 without a single double on the GPU. The authoritative
height — the one physics and the server evaluate — is plain C++ over the same base and the same
layers, with no GPU in the loop, because the server has no GPU and the atmosphere's nest runs
headless. Everything below the data's resolution is synthesis, everything at or above it is
measurement, and the document never blurs which is which.

### 0.2 The ten decisions

Recorded so they are premises rather than re-litigated preferences.

| # | Decision | Consequence |
|---|---|---|
| D1 | Scope is terrain + the surface regime. Water, vegetation, and the builder get **seams**, not implementations. | §15 publishes three interfaces and stops. |
| D2 | Terrain is **layered and run-time editable**. | §6 exists; the tile compile is mandatory, not an optimisation. |
| D3 | Determinism is **hybrid**: base + layers are authoritative, sub-Nyquist detail is not bit-exact across CPU and GPU. | §13 maps this onto UHM's `Bitwise`/`Tolerant`/`Cosmetic` classes. |
| D4 | The camera must be seamless from **1 m to orbit** — walking, driving, flying, and orbital. | §9 (precision) and §10 (the LOD ladder hand-off) are load-bearing, not polish. |
| D5 | Assets are produced by **`se planet bake`**, from public sources, on the user's machine. | §5.4; the same shape as `se climatology bake`, provenance included. |
| D6 | Baseline hardware is a **GTX 1060 6 GB at 1080p60**. | §17's budgets are absolute, not aspirational; the tier features are additive only. |
| D7 | The height source is an **interface**, so procedurally generated bodies use one code path with real ones. | §5.1's `IHeightSource`; costs nothing now, costs a rewrite later. |
| D8 | Water: produce **bathymetry and a land/sea mask**, do not draw a water surface. | §14.2 satisfies the atmosphere's surface-property provider; the ocean is a later document. |
| D9 | First body is the **Moon**. | §20's P0–P5 run on it: no atmosphere, no ocean, no vegetation, no clouds to confound a terrain bug. |
| D10 | **Vertical slice first.** | P2 puts walkable ground on screen before the baker is complete or the streaming is asynchronous. |

---

## 1. Ground truth this design stands on

Verified against the tree on 2026-08-01. Nine facts are load-bearing; nothing below invents a
capability the engine does not have or duplicates one it does.

**G1 — There is no terrain, and the absence is already a recorded blocker.** `PlanetParams`
(`render/environment.hpp:189`) is two colours and a roughness; `sky.frag`'s `surface_albedo`
(`render/shaders/sky.frag:298`) paints a body from noise and its pole, and `relief_normal` (`:321`)
fakes relief by perturbing a normal. `atmosphere_system.md` §15 names this exactly: *"Blocker: no
terrain height field exists in the engine … Orography, surface type, valley fog, föhn, rain shadows,
and terrain-driven turbulence — all of Phase D and part of Phase B's surface model — cannot start
until the terrain system provides one."*

**G2 — The body LOD ladder is already declared, and its two near rungs are empty.**
`Render::BodyLod` (`render/environment.hpp:697`) is `Point → Disk → Impostor → Mesh → Surface`, and
its own comment states *"The far-field sky pass handles Point/Disk/Impostor today; Mesh/Surface are
the near-field hand-off."* `Astro::SURFACE_HANDOFF_ALTITUDE_RADII = 10.0`
(`astro/ephemeris.hpp:188`) already picks the dominant body and switches `planet_surface_visible`
off past the hand-off. Terrain does not invent a regime; it fills two declared ones.

**G3 — The frame stack for standing on an arbitrary body is complete and body-parametric.**
`astro/surface_frame.hpp` supplies `geodetic_to_body_fixed`, `body_fixed_to_geodetic`,
`geodetic_normal`, and `local_tangent_basis` for any catalogued body; `astro/body_orientation.hpp`
supplies the IAU spin `W(t)`; `docs/architecture/domain-astro.md` §1's three coordinate spaces
(solar / planet / local) and the sphere-of-influence rebase already exist. Terrain consumes this and
adds no frame of its own.

**G4 — The renderer is a Vulkan 1.4 frame graph with a bindless heap, and its per-frame descriptor
set is full.** `render/graph/render_graph.hpp` derives barriers, aliasing, culling, and
async-compute submissions from declarations. `Scene::SceneLayout` (`render/scene/scene_layout.hpp`)
is set 0 with **32 bindings, all named** — `POST_BINDING = 31` is documented as *"the last
frame-global binding the guaranteed 32-entry push set has room for."* Terrain therefore **cannot**
add a frame-global binding; it takes its own set 2, exactly as the GPU-driven instance path
(`INSTANCE_SET = 2`, `:331`) and the meshlet path already do.

**G5 — The device tiers are known, and terrain must not require any of them.**
`render/rhi/vulkan/vulkan_device.cpp:153` enables exactly one core feature, `samplerAnisotropy`:
**tessellation shaders, sparse residency, and multi-draw-indirect are not enabled.** Mesh shaders
(`supports_mesh_shader()`), ray query, host image copy, and shading-rate images are queried and
optional. The base terrain path uses none of them; §8.5 states which are additive tiers.

**G6 — The physics height-field collider exists and takes a borrowed view.**
`physics/collision/height_field_manifold.hpp:72`'s `HeightFieldView<T>` is
`{const T* heights, columns, rows, cell_size_x, cell_size_z, center, orientation}` — a *borrowed*
pointer, row-major, with a corner-anchored placement. It needs no cooking step and no ownership
transfer, which is precisely what a streaming tile can satisfy. `test_height_field_compound.cpp`
already exercises it.

**G7 — The asset-bake pattern is established, down to the packaging.**
`cli/sushiengine/services/climatology/` is a package of `sources.py` (a `Source` table carrying
`url`, `describes`, and `attribution`), `reanalysis.py`, `landmask.py`, and `asset.py`; heavy
dependencies live behind a `[project.optional-dependencies]` extras group so `se` stays installable
without them (`cli/pyproject.toml`). The consumer is a value type that *adopts bytes* and degrades
rather than failing (`sim/climatology_asset.hpp:70`). `se planet bake` is this shape again, and §5.4
does not re-derive it.

**G8 — Floating-origin types exist and are unconsumed.** `WorldVector3`, `SectorCoord`, and
`FloatingOriginVector3` are in `core/blas_placeholder.hpp:229-291`;
`docs/architecture/foundation.md` §2 records them as *"the SushiLoop M0 foundation … not yet
consumed by any simulation code."* §9 explains why terrain does **not** consume them either, and
what it uses instead.

**G9 — The precision hazard at planet scale is measured, not theoretical.** The project memory
records `sky.frag`'s `ray_ellipsoid` trapping at ~6.4 × 10⁶ m in float32, and the shader carries a
hand-built remedy: `scene.planet_precision` holds CPU-computed double-precision intermediates
(`sky.frag:193-221`) precisely because the naive form loses catastrophic significance. §9
generalises that remedy instead of rediscovering it.

---

## 2. Design principles

**T1 — Measurement and synthesis never blur.** Every height carries, implicitly, the depth at which
its data source ran out. Above that depth it is measurement; below it, synthesis. The distinction is
in the format (§5.2), in the determinism class (§13), and in what the editor shows. A system that
cannot say which is which will eventually claim accuracy it does not have.

**T2 — The authority is host code with no GPU in it.** The server is headless; the atmosphere's nest
and the physics solver run without a swapchain; a unit test must be able to ask for an elevation.
Therefore the *definition* of terrain height is C++ (`terrain/height_function.hpp`), and the GPU
compile shader is a **port** of it held to a stated tolerance by a conformance test — the same
discipline the engine already applies to skinning, where the Vulkan path ships and the SYCL kernel
is the correctness oracle.

**T3 — Editability is a property of the pipeline, not a feature bolted to it.** A design where tiles
are loaded and drawn cannot later grow a dam that floods a valley. A design where tiles are
*compiled* gets craters, roads, cuts, and fills for the cost of one more input to a shader that had
to exist anyway. D2 is therefore paid at P0, not deferred.

**T4 — One code path per concept, parameterised by body.** No `if (body == Earth)`. The Moon's
crater layer op and Earth's road op are the same interface; the Moon's geologic unit and Earth's
land-cover class are the same surface class; bathymetry is elevation below zero. D9 exists to
*prove* this — a second body that needed new code would mean the first one was written wrong.

**T5 — Nothing regresses, and every claim is a number.** Terrain enters an engine with a measured
frame budget. §17 states absolute costs on the D6 baseline, and each phase in §20 carries an exit
criterion that is measured rather than asserted.

**T6 — The engine owns the vocabulary; data sources are backends.** `IHeightSource` (D7) is named by
the height function; a pak file, a procedural generator, and a high-resolution regional inset are
three implementations. No consumer of terrain knows which one answered.

---

## 3. The model at a glance

```
   BAKE (offline, `se planet bake`, once per body per quality tier)
   public DEM + imagery + land cover ──► cube-face reprojection ──► tile pyramid ──► .planet pak
                                          (geoid removed, §5.3)      (§5.2)

   ─────────────────────────────────────────────────────────────────────────────────────────

   HOST (per frame, no GPU; also the whole of the headless/server path)

     TileAddress algebra ──► quadtree selection ──► visible node list ──► per-node frame (double)
     (§4)                    (§7.1)                                       (§9)
                                    │                                           │
     IHeightSource ─┐               ├──► collision patch set ──► HeightFieldView │  (§12)
     LayerStack   ──┼──► height_function ──► ITerrainField grid sample ──► atmosphere nest (§14)
     DetailField  ──┘        (§2/T2)

   ─────────────────────────────────────────────────────────────────────────────────────────

   DEVICE (per frame)

     TileStreamer ──► TileCache slots ──► TerrainCompilePass (compute) ──► compiled height/class
     (§7.3)          (§7.2)               (§6.3)  base + layers + detail
                                                          │
                          ┌───────────────────────────────┴───────────────────┐
                     TerrainDepthPass                                  TerrainOpaquePass
                     (Hi-Z, prepass)                    one instanced draw, CDLOD morph (§8)
                                                        material synthesis in fragment (§11)
```

Three tiers, one waist. The waist is the height function: the host defines it, the compile shader
ports it, and everything else — geometry, collision, weather, the builder — reads one of the two.

---

## 4. The geometry model: a cube-sphere quadtree

### 4.1 Why the cube

The parameterisation has to give square tiles, integer address arithmetic, a quadtree with no
special cases, and no singularity. Four candidates:

| Scheme | Verdict |
|---|---|
| Equirectangular (lat/lon) quadtree | **Rejected.** The poles are a singularity: tiles converge to zero width, texel density diverges, and every pole crossing is a special case. It is the natural *data* format and the worst *runtime* format. |
| Icosahedral / geodesic grid | **Rejected.** Near-uniform area, but triangular cells have no clean quadtree address algebra, no square tile for a texture pool, and no natural (u,v) for a grid mesh. It buys uniformity and pays in every other system. |
| HEALPix | **Rejected.** Equal-area and quadtree-addressable, but its cells are diamonds in a curvilinear frame; the tooling cost is real and the win over a tangent-warped cube is ~5 % in area uniformity. |
| **Cube-sphere with tangent warp** | **Chosen.** Six square faces, each a plain quadtree over (u,v) ∈ [−1,1]², integer addressing, square tiles, no singularity, and — with the warp — near-uniform cell size. |

### 4.2 The tangent warp

The naive cube-to-sphere map `n(f) = normalize(f)` compresses cells toward the face corners: the
solid angle of a face-centre cell is roughly 1.4× that of a corner cell. The fix is to warp the
uniform grid parameter *t* ∈ [−1,1] before projecting:

```
u = tan(t · π/4)
```

so that `atan(u) = t·π/4` is uniform in angle. This drops the area ratio to ≈ 1.06 — good enough
that no system downstream needs to compensate for it. The warp is applied on the host when a tile
address is turned into face coordinates, and is baked into the pak's sampling grid, so the run-time
hot path never evaluates a `tan`.

### 4.3 Addresses

```cpp
struct TileAddress
{
    std::uint8_t  face;   // 0..5
    std::uint8_t  depth;  // 0..MAX_DEPTH
    std::uint32_t x, y;   // < (1u << depth)
};
```

Parent, child, and neighbour are integer arithmetic; a neighbour that crosses a face edge is a table
of six rotations, computed once. `MAX_DEPTH` is 20, which §9 shows is within the precision budget
and which, on Earth, is a 0.075 m render-grid cell — an order of magnitude below anything the data
or the synthesis can justify, so the limit is never the binding constraint.

### 4.4 Node geometry and the ellipsoid

A node's four corners map to the reference ellipsoid by scaling the unit direction componentwise by
the semi-axes: `P(n) = (a·nx, a·ny, c·nz)`. This is the exact ellipsoid, and its geodetic normal is
`normalize(Px/a², Py/b², Pz/c²)`. **Elevation displaces along the geodetic normal**, not along the
radial direction, because that is how every DEM's heights are defined; on Earth the two differ by up
to 0.19°, which at a 3 km mountain is a 10 m horizontal error — visible, and free to avoid.

### 4.5 Resolution ladder

With a **129 × 129 height tile** (128 intervals) and a **33 × 33 render grid** (32 cells) per node:

| Depth | Earth height texel | Earth render cell | Moon height texel |
|---|---|---|---|
| 0 | 78.3 km | 313 km | 21.3 km |
| 5 | 2.45 km | 9.8 km | 666 m |
| 7 | 611 m | 2.45 km | 167 m |
| 11 | 38.2 m | 153 m | 10.4 m |
| 14 | 4.78 m | 19.1 m | 1.30 m |
| 17 | 0.60 m | 2.39 m | 0.16 m |
| 20 | 0.075 m | 0.30 m | 0.020 m |

The tile is deliberately **4× finer than the render grid at the same node**. The extra resolution is
not wasted: it is what the fragment shader reads for the macro normal, and it is what keeps a node's
silhouette honest while its geometry stays cheap.

---

## 5. The data model

### 5.1 The source interface (D7)

```cpp
namespace SushiEngine::Terrain
{
    /// Where a tile's measured elevation comes from. A baked pak, a procedural
    /// generator, and a high-resolution regional inset are three implementations;
    /// no consumer of terrain learns which one answered.
    class IHeightSource
    {
    public:
        virtual ~IHeightSource() = default;

        /// The deepest node depth this source has real data for, at this address.
        /// Below it the height function synthesizes (§6.4) rather than resampling.
        virtual std::uint8_t data_depth(TileAddress address) const = 0;

        /// Fills a (129+2)² R16-equivalent patch of elevations in metres above the
        /// reference ellipsoid, including the one-texel apron. Returns false when the
        /// address is outside this source's coverage.
        virtual bool sample_tile(TileAddress address, float* heights_metres,
                                 TileStatistics& statistics) const = 0;
    };
}
```

Three implementations ship: `PackHeightSource` (§5.2), `InsetHeightSource` (a regional pak that
answers only inside its footprint and only below the global source's depth — this is how a 30 m
Copernicus tile or a 1 m HiRISE DTM enters without a second pipeline), and `GeneratedHeightSource`
(a procedural body). A `CompositeHeightSource` picks the deepest source that covers an address.
**This is the whole of D7's cost**, and it is the reason a fictional moon in Sushiverse renders
through the same pass as the real one.

### 5.2 The pak format

One file per body per quality tier, `assets/planet/<body>.<tier>.planet`. A value type adopts its
bytes and refuses a malformed blob whole (G7's pattern):

```
header    magic, version, body id, semi-axes, tier, data depth per channel,
          the source table (url, checksum, attribution) verbatim from the bake
index     one record per stored tile: address, byte offset, byte length, codec,
          min/max elevation                              <- min/max feeds Hi-Z culling (§7.1)
payload   height tiles   131² R16, delta-coded + zstd     (129² grid + 1-texel apron)
          colour tiles   128² BC7                          (satellite/orthoimagery)
          class tiles    128² RG8, two classes + weights   (stored to a shallower depth, §5.5)
```

Three deliberate choices. Heights are **integers, not floats**: R16 over a per-tile `(min, max)`
range gives ≤ 0.15 m quantisation on a 10 km range and compresses; a float tile does neither. The
**apron** is one texel of the neighbour's data, so central-difference normals and bilinear filtering
never need a second tile resident — the single largest source of seams in a naive implementation,
removed in the format rather than patched in the shader. And the **index carries min/max**, so a
node's bounding volume is exact and free, which is what lets terrain participate in the existing
Hi-Z occlusion pass.

### 5.3 What the bake has to get right

- **Geoid removal.** Earth DEMs are orthometric — heights above the EGM96 geoid, which departs from
  the ellipsoid by −107 m to +85 m. The bake applies the correction so the run time only ever sees
  heights above the ellipsoid, which is the only datum the renderer, the physics, and the atmosphere
  can share. Mars (areoid) and the Moon (selenoid) get the same treatment from their own PDS
  reference surfaces. Skipping this puts sea level up to 100 m off, which is exactly where
  coastlines are.
- **Resampling.** Source rasters are equirectangular; the cube face is not. Resampling uses
  area-weighted averaging on the way down the pyramid (never point sampling, which aliases ridges
  into noise) and bicubic on the way across the projection.
- **Void filling.** SRTM and LOLA have voids. They are filled from the next coarser source and
  **marked in the class tile**, so a later system can tell a measured plain from a filled one.
- **Provenance.** The source table — url, sha256, attribution string — is written into the header
  from the same table the downloader used, per G7. An asset whose provenance is typed out beside the
  downloader is an asset whose provenance is wrong within a month.

### 5.4 `se planet bake`

`cli/sushiengine/services/planet/` mirrors `services/climatology/` exactly: `sources.py` (the
`Source` table with `describes` and `attribution`), `dem.py`, `imagery.py`, `landcover.py`,
`cube.py` (reprojection), `pack.py` (the writer). Heavy dependencies (`rasterio`/`GDAL`, `numpy`,
`requests`) go behind a `planet` extras group so every other `se` command stays installable.

```
se planet bake --body moon  --quality standard
se planet bake --body earth --quality standard
se planet bake --body earth --inset 36,-122,500km --resolution 30m   # a named region at depth 11
se planet inspect assets/planet/moon.standard.planet
```

Sizes. The depth is **not** a tier constant: it is the deepest level the source's own
resolution supports on *that body*, which is why Earth needs two more levels than the Moon
for the same metric resolution — it is 3.7× the radius, so the same ground sample distance
sits two quadtree levels deeper. Marked figures are measured; the rest follow from the same
arithmetic.

| Tier | Body | Source | Depth (texel) | Tiles | Asset |
|---|---|---|---|---|---|
| `compact` | Moon | LOLA 16 ppd — 1.9 km, 33 MB download | 3 (2.7 km) | 510 | **17.5 MB** *(measured)* |
| `standard` | Moon | LOLA 64 ppd — 474 m, 530 MB download | 5 (666 m) | 8 190 | ≈ 281 MB |
| `standard` | Earth | ETOPO 2022 15″ — 460 m | 7 (611 m) | 131 070 | ≈ 4.5 GB |
| `full` | any | + regional insets at depth 11 | 11 (38 m) | — | +≈ 350 MB per 500 km region |

Baking *past* the source's depth is allowed and pointless: it stores resampled levels the
runtime can already produce on demand from the stored ancestor, and the asset still reports
the source's own depth so nothing downstream mistakes them for measurement. `--depth` exists
because a tier is a default, not a policy.

Baking a *global* 30 m pyramid would be 6 × 4¹¹ tiles ≈ **860 GB**. That is the arithmetic behind
the tiering, and it is why §18 names 460 m as the honest global ceiling rather than pretending
otherwise.

### 5.5 Sources

All public domain, all credential-free — the same standard `se climatology bake` holds itself to,
for the same reason: an asset nobody else can reproduce is an asset nobody else can check.

| Body | Elevation | Imagery | Surface class |
|---|---|---|---|
| Moon | LRO LOLA (118 m global) | LROC WAC mosaic (100 m) | USGS geologic units |
| Earth | ETOPO 2022 (15″ ≈ 460 m, includes bathymetry) + SRTM 1″ insets | Blue Marble Next Generation | MODIS MCD12Q1 |
| Mars | MOLA (463 m) + HRSC/HiRISE DTM insets | Viking + MOC mosaic | USGS geologic units |
| Mercury | MESSENGER MDIS DEM | MDIS mosaic | USGS geologic units |
| Venus | Magellan topography | Magellan radar | Magellan units |
| Gas giants | none — the ellipsoid *is* the 1-bar level | Cassini/Juno cylindrical mosaic | — |
| Pluto, Ceres, Vesta | New Horizons / Dawn (USGS Annex) | same | USGS units |

A gas giant has no terrain, and this design says so rather than inventing one: its `IHeightSource`
returns `data_depth = 0` and a flat tile, the quadtree stops descending, and what remains is a
textured ellipsoid with a cloud deck — which is what a gas giant is.

**On licensing.** Every source in the table is a NASA/NOAA/USGS product in the public domain, which
is what keeps the bake reproducible by anyone and the resulting asset checkable rather than merely
believable. Sources with better resolution but a licence attached — Copernicus DEM GLO-30 (30 m
global, free but attribution-obliged) is the notable one — are supported as *optional* named insets
rather than defaults, so a licence obligation is something a user opts into knowingly and the
attribution rides in the pak header (§5.3) either way. The engine's own Apache-2.0 terms cover the
code; they never cover an asset, and the header is where that distinction is made concrete.

---

## 6. The layer stack — what makes terrain editable (D2)

### 6.1 A layer is a record, not a raster

```cpp
struct TerrainLayer
{
    std::uint32_t    order;        // explicit; composition must not depend on iteration order
    LayerShape       shape;        // point+radius | polyline+width | polygon
    LayerOperation   operation;    // Crater | Flatten | Raise | Carve | Ramp | Blend
    LayerProfile     profile;      // the operation's parameters and falloff
    ClassOverride    surface;      // optional: force a surface class inside the footprint
};
```

A crater is a point, a radius, and a rim/bowl/ejecta profile. A road is a polyline, a width, and a
"flatten to the polyline's own elevation with a shoulder falloff". A dam is a polygon and a level. A
building pad is a flatten. **The record is kilobytes**, which is why layers replicate over the
network trivially, serialise into a scene without a binary blob, and give the builder undo/redo for
free. The alternative — storing edited *rasters* — would multiply the pak by the edited area and
make every edit a streaming problem.

`order` is explicit and required. Composition that depends on the order a spatial index happened to
return would be non-deterministic, and §13's authoritative claim would be false.

### 6.2 The index

Layers live in a loose quadtree over cube-face space, per body. A node's compile needs the layers
overlapping its footprint — typically zero, occasionally a handful. Editing a layer marks a
cube-face region dirty; resident tiles overlapping it are recompiled, at a bounded rate per frame,
and their collision patches re-evaluated. Nothing else in the system observes the edit.

### 6.3 The compile

`terrain_tile_compile.comp`, one workgroup per tile:

1. Read the base tile (from the cache slot the streamer filled), or upsample the parent's tile when
   the node is below the source's data depth.
2. Add synthesized detail (§6.4) for the depths past `data_depth`.
3. Apply each overlapping layer in `order`.
4. Write the compiled height into the node's slot, together with its recomputed `(min, max)`.

The compile is the reason D2 costs almost nothing: without editing, steps 1 and 2 still have to
happen somewhere, so step 3 is the only addition, and it is a loop over an almost-always-empty list.

### 6.4 Detail synthesis

Below the source's data depth there is no measurement, and the choice is between blur and invention.
This design invents, under three constraints:

- **A pure function of position**, never of node — evaluated identically wherever it is sampled, so
  tiles agree across their shared edges by construction rather than by fixing up seams.
- **Amplitude modulated by the terrain's own statistics**: the local slope and the source's own
  finest-level roughness, plus the surface class. A sea floor gets nothing, an alluvial plain gets
  almost nothing, a mountain flank gets ridged multifractal detail scaled by its slope. This is what
  keeps synthesis from reading as noise sprinkled on data.
- **Integer-hash value/gradient noise using only `+`, `-`, `*`** — no transcendentals, because
  `sin`/`exp` are not bit-identical across vendors and §13 needs the CPU and GPU forms to agree to a
  stated tolerance.

Detail is what carries the render grid from the data's 611 m to the metre scale D4 requires. It is
also, honestly, the part of the terrain that is not real, and §18 says so.

---

## 7. The runtime

### 7.1 Quadtree selection

Per frame, per body, on the host in double precision: start from the six root faces — already a
complete cover of the body — and repeatedly replace the node most over the screen-space error target
with its four children, while the budget allows. **Refinement rather than descent**, because
splitting a cut yields a cut: the selection covers every point exactly once at every stage,
*including* the stage the budget stops it at. A recursive descent cannot promise that, since it
commits to refining a subtree before it knows whether it can afford to emit the whole of it, and
running out part-way leaves a hole rather than a coarser patch. (This was found by the test that
pins the cover, not by looking at a picture — the recursive form was written first.)

Reject a node whose bounding volume — the tile's exact `(min, max)` band over its patch, from the
pack index — fails the frustum test or last frame's Hi-Z pyramid (the existing `OcclusionPass`).
Output is a flat list of visible nodes with their per-node frames (§9).

Cost is a few thousand node visits: measured target ≤ 0.3 ms on the D6 baseline. Selection must be
double throughout — a node centre is a planet-scale coordinate, and this is the one place the whole
system is allowed to hold one.

The same selection serves the depth prepass, the opaque draw, and the sun's shadow cascades (the
last at a shallower error target, since a cascade does not need the near LOD).

### 7.2 The tile cache

A fixed pool of GPU slots, three arrays, no sparse residency (G5):

| Pool | Format | Slots | VRAM |
|---|---|---|---|
| Height | R16, 131² | 4096 | 140 MB |
| Colour | BC7, 128² | 4096 | 67 MB |
| Class | RG8, 128² | 1024 | 34 MB |

Slots are evicted least-recently-**bound**, not least-recently-uploaded: what matters is whether a
slot is still being drawn from, and a tile twenty descendants are inheriting from is touched by all
twenty. The node buffer maps each visible node to its slot indices; a node whose tile is not yet
resident **inherits its nearest resident ancestor's slot with adjusted UVs**, which is what makes
streaming pop-free and lets the cache be smaller than the visible set without ever stalling a frame.
Class tiles are stored to a shallower depth (§5.5) and are inherited by design rather than as a
fallback — land cover has no information below 500 m; slope and elevation take over there (§11).

Two rules the bookkeeping owes the frame, both of which are silent when broken. Binding through an
ancestor must touch that ancestor, or a tile nothing draws *directly* is evicted out from under
every descendant reading it. And a slot already bound this frame must never be handed out again: a
full cache refuses and the requesting tile waits a frame, because the alternative is a node drawing
from an image that changed after it was queued.

The UV rectangle is the third. A tile's own grid occupies texels 1 to 129 of a 131-texel image, so
even the identity case is `offset = 1.5/131, scale = 128/131` rather than `(0, 1)`; an inherited
rectangle scales that by the depth difference and offsets it by where the node sits in its ancestor.
Half a texel of error there shifts terrain everywhere it inherits — a shimmer at LOD boundaries that
no screenshot explains — so it is a pure function tested by computing the geographic point it lands
on both ways, rather than four lines inside an upload path.

### 7.3 Streaming

Disk read and decompression run on worker threads behind a bounded queue, ordered by the node's
screen-space priority. Upload uses its own ring — **not** `TextureLibrary`, which is path-keyed,
material-oriented, and mip-residency-based, and whose lifecycle is wrong for a slot pool. Where the
device offers `supports_host_image_copy()` the upload skips staging entirely; otherwise it stages,
exactly as the texture path already chooses once at bring-up.

Per-frame work is bounded: at most *N* tiles uploaded and *M* compiled. Terrain never blocks a
frame; it renders whatever is resident, inherits for the rest, and catches up.

---

## 8. The render path

### 8.1 One draw call for a planet

All visible nodes go into one storage buffer; the terrain draws **one instanced call** of the shared
33 × 33 grid mesh with `instance_count = node_count`. Per-node data — the frame from §9, the slot
indices, the morph parameters, the depth — is read by `gl_InstanceIndex`. A planet is one draw, and
the CPU cost of terrain is the selection and nothing else.

### 8.2 CDLOD morph

Each vertex carries a morph weight from its distance to the camera, and its grid position is
interpolated toward the position it would have on the parent's grid. Because the morph is applied to
the grid parameter **before** the projection, it is geometric: neighbouring nodes at different
depths agree exactly along their shared edge, and there is no popping as a node splits. This is what
removes cracks — no skirts, no stitching strips, no T-junction repair. The height sample is morphed
the same way, by sampling the tile at both the fine and the parent-aligned coordinate and blending
with the same weight.

### 8.3 Descriptor layout

Set 0 is full (G4). Terrain therefore owns **set 2**:

| Binding | Contents |
|---|---|
| 0 | Node buffer (per-node frames, slot indices, morph parameters) |
| 1 | Height slot array (R16, 2D array) |
| 2 | Colour slot array (BC7) |
| 3 | Class slot array (RG8) |
| 4 | Body parameters (semi-axes, class palette → bindless material indices) |

Set 0 and set 1 stay identical to every other scene pipeline, so terrain is lit, shadowed, fogged,
and tone-mapped by the shading path that already exists — no parallel lighting code. This is the
same arrangement `SceneLayout::gpu_pipeline_layout()` already uses for GPU-driven draws.

### 8.4 Passes

`TerrainDepthPass` (into the existing depth prepass target, so Hi-Z and GTAO see terrain) and
`TerrainOpaquePass`. Both are ordinary `IRenderPass` implementations that declare their resources
and record draws; the frame graph derives every barrier. `TerrainCompilePass` is a compute pass that
may declare `PassQueue::AsyncCompute`, which the graph honours when the device and the frame allow.

### 8.5 Tiers, all additive

- **Mesh shaders** (`supports_mesh_shader()`): a task shader can descend the quadtree on the GPU and
  cull per-meshlet, removing the host selection from the critical path. Worth doing *after* the base
  path is measured, and never as a requirement — the host quadtree has to exist regardless, because
  physics and the builder need it.
- **Ray query** (`supports_ray_query()`): terrain in the ray-traced shadow tier.
- **Shading-rate image**: terrain at grazing distance is the ideal candidate for a coarse rate.

None of these are on the D6 baseline path.

---

## 9. Precision — the part that decides whether this works

### 9.1 The problem

A vertex on Earth's surface has a magnitude of 6.37 × 10⁶ m. Float32 carries ≈ 1.2 × 10⁻⁷ relative
precision, so **any** planet-space quantity computed in float32 lands with ≈ 0.76 m of error. At
depth 17 the render cell is 2.4 m. The terrain would boil.

Note what does *not* solve it. Camera-relative positions alone do not, if the position is *derived*
on the GPU from a planet-space parameterisation — the error is introduced before the subtraction.
The floating-origin types (G8) do not either: they solve absolute world positions for the
simulation, and terrain's problem is the *evaluation of a curved surface*, which is a different
question. And doubles on the GPU are not an answer at 1/32 rate.

### 9.2 The form

Split the face coordinate into a node centre `c` (held in double on the host) and a small offset `δ`
from the grid, `f = c + δ`. Then

```
P(f) − camera  =  [ P(c) − camera ]  +  A · ( n(c + δ) − n(c) )  +  h · N(c)
```

The first bracket is computed **on the host in double** and is camera-relative, so its magnitude is
bounded by the visible range — float32 is ample. The last term is the elevation along the geodetic
normal, small by construction. Everything turns on evaluating the middle term without cancellation,
since `n(c+δ)` and `n(c)` are both ≈ 1 and differ by ≈ θ.

Expand it exactly:

```
n(c+δ) − n(c) =  δ/|g|  −  c · (2 c·δ + |δ|²) / ( |g| · |c| · (|c| + |g|) ),      g = c + δ
```

derived from `|c| − |g| = (|c|² − |g|²)/(|c| + |g|)` and `|c|² − |g|² = −(2 c·δ + |δ|²)`. **No
subtraction of nearly-equal quantities survives.** Every term is either small (δ, and the dot
products of δ) or a well-conditioned ratio of order-one quantities. The float32 error in `c` only
ever multiplies a factor of order θ, so the absolute position error is ≈ 1.2 × 10⁻⁷ · θ · a.

**Measured 2026-08-01** (P0 landed), worst case over all six faces and nine tiles per face on
Earth's ellipsoid, float32 against a double reference:

| Depth | Render cell | Difference form | Naive float32 | Ratio |
|---|---|---|---|---|
| 8 | 1223 m | 4.2 × 10⁻³ m | 0.82 m | 197× |
| 12 | 76.4 m | 2.2 × 10⁻⁴ m | 1.16 m | 5 361× |
| 16 | 4.78 m | 1.5 × 10⁻⁵ m | 1.12 m | 74 119× |
| 18 | 1.19 m | 4.7 × 10⁻⁶ m | 1.04 m | 220 791× |
| 20 | 0.299 m | 8.6 × 10⁻⁷ m | 1.04 m | 1 202 751× |

The two columns say different things, and the difference between what they say is the whole
argument. The naive form's error is **constant in depth** — about a metre, set by float32's absolute
resolution at 6.37 × 10⁶ m and wholly indifferent to how small the tile is. It is already a quarter
of a cell at depth 16 and three and a half cells at depth 20. The difference form's error is
**proportional to the cell** — a constant 3 × 10⁻⁶ of it at every depth measured, which is the scale
invariance the derivation predicts and the property that actually matters: there is no depth at
which it degrades, so `MAX_TILE_DEPTH` is bounded by what the data and the synthesis justify rather
than by arithmetic.

Nine floating-point operations more than the naive form, no doubles, no branches, one code path from
depth 0 to depth 20. It is the same remedy `sky.frag` already applies by hand at `sky.frag:193-221`
(G9), generalised and given a derivation instead of a comment.

### 9.3 Depth buffer

The renderer already uses reverse-Z with an infinite far plane (`docs/architecture/domain-astro.md`
§1), which is what makes a 1 m rock and a 6 000 km horizon coexist in one depth buffer. Terrain
inherits it and requires nothing further.

### 9.4 The frame the data lives in — and the bug it exposed

Everything above happens in the body's **fixed** frame: the elevations were baked against real
selenographic coordinates, and the ellipsoid sits at its origin. The scene is anchored somewhere
else entirely — at an observer's surface point, +Y along their geodetic up. One rotation joins them,
and terrain is the first consumer that can *tell* whether that rotation is right.

It was not. The engine measured each body's spin from the wrong reference direction, and putting a
real map on a planet is what made it visible:

- `body_rotation_angle` returns the IAU angle **W**, which is measured from the ascending node of
  the body's equator on the **J2000 equator**.
- `ecliptic_to_body_equatorial` builds a frame whose +X is the node with the **ecliptic**.

Both are perfectly good frames; they are not the *same* frame, so W was never an angle in the one it
was being added to. The gap is a fixed property of each body's pole and it is not small — **52.68°
for the Moon, −40.86° for Mars, −117.65° for Venus** — and for Earth the same class of mismatch cost
0.31° (35 km at the equator), because the sky basis used sidereal time while the body-fixed
conversion used W.

`Astro::prime_meridian_angle(body, jd)` is now the single answer to "where is the prime meridian",
and the four places that needed it — `fill_environment_sky`, `scene_frame_for`, and both directions
of the simulation's body-fixed pose conversion — go through it. Earth resolves to Greenwich sidereal
time, so the home sky does not move; every other body moves to where it should always have been. All
63 existing astro tests pass unchanged.

Earth's path is not *bit*-identical, and the difference is worth stating rather than glossing: the
meridian is now wrapped before the longitude is added instead of after, which is the same angle
modulo a turn but not the same double. Measured over 4000 epochs and 37 places, the worst direction
moves 6.3 × 10⁻¹⁶ rad — 1.3 × 10⁻¹⁰ arcseconds, or four nanometres at Earth's radius.

The check that decides this is deliberately **external to the conversion**, because every
self-consistency test in the world passes on a frame that is uniformly rotated. The Moon is tidally
locked: from selenographic (0, 0) the Earth must stand within the libration cone of the zenith,
about 10°. Before: 29°–40° of elevation. After: **80°–89°**.

`Environment` carries the result to the renderer as `planet_body_axes` — three columns, the third of
which *is* `planet_pole`, taken rather than recomputed — plus `planet_body`, the index a consumer
holding per-body data needs in order to pick the right one. The seam stays astro-free: they are
plain vectors and a plain int, and the ephemeris is the only thing that fills them.

---

## 10. The LOD ladder hand-off (G2, D4)

`BodyLod` already declares five rungs. Terrain fills the two empty ones and simplifies a third:

| Rung | Today | With terrain |
|---|---|---|
| `Point`, `Disk` | Sky pass, analytic | Unchanged. |
| `Impostor` | Sky pass, procedural `surface_albedo` | A single shallow terrain node with its colour tile — real geography instead of noise. |
| `Mesh` | **Empty** | The quadtree at a depth set by the body's angular size. Earth seen from the Moon has real continents. |
| `Surface` | Analytic ground in `sky.frag:634-660` | The quadtree at full depth. |

The hand-off is surgical, and it exploits a fact already true: the sky pass samples depth (it is one
of its six pass-local image bindings, per `scene_layout.hpp:142-151`), and `sky.frag` already writes
`ground_hit = ground_t > 0 && ground_t < geometry_t` — the analytic ground loses to anything nearer.
**The analytic ground is not deleted; it becomes the far-field fallback**, which is the right role
for it and costs nothing to keep.

That per-pixel test alone is not sufficient, though, and the reason is worth stating because it is
not obvious: *the analytic ground wins wherever the real elevations dig below the reference
ellipsoid*. On the Moon that is every mare — two kilometres of basin floor, over which the reference
sphere is the nearer surface and would be drawn on top of the terrain inside it. So the switch is
made once per frame rather than per pixel: when the selection produced nodes, the frame's scene
block turns the analytic ground off for that body (`Scene::suppress_analytic_ground`). This is sound
because §7.1's cut is an *exact cover* — the quadtree always spans the whole body, coarsely at the
horizon but never with a hole — so there is no pixel the analytic ground would have been needed for.
A body with no pack keeps it, unchanged.

`planet_surface_visible` and `SURFACE_HANDOFF_ALTITUDE_RADII` keep their present meanings. The one
new behaviour is that a *non-dominant* body may also carry terrain at the `Mesh` rung, which is what
makes the solar system look like a solar system from inside it.

---

## 11. Surface materials — the answer at one metre

Satellite imagery is ~500 m per texel. At 1 m it is a single flat colour. The design blends three
things:

1. **The colour tile** (satellite) supplies the low-frequency truth: this valley is olive, that
   ridge
   is ochre, this ice is blue-white. It dominates beyond ~2 km.
2. **The class tile** supplies up to two surface classes and their weights per texel, derived at
   bake time from land cover (Earth) or geologic units (everywhere else), refined at compile time by
   slope and elevation — a 40° slope is rock whatever the land-cover raster says, and above the snow
   line it is snow.
3. **The material palette** is a per-body set of ≤ 16 tiling PBR materials in the bindless heap
   (albedo/normal/ORM), sampled with stochastic tiling so the repeat is invisible, and blended by
   **height blend** rather than linear blend — gravel shows through sand where the gravel's own
   height map is higher, which is what makes a transition read as a transition instead of a
   crossfade.

The step that makes it hold together: **albedo preservation.** The blended material's local average
is divided out and the satellite colour multiplied in, so a mountain is the same colour from orbit
and from its own slope. Without it, the near field and the far field are two different planets, and
the crossfade between them is the most visible artefact the system can produce.

Below the data, the same class field drives the detail synthesis of §6.4 — so the geometry the
player walks on and the material on it come from one description of what that ground *is*.

---

## 12. The physics seam

`HeightFieldView<T>` (G6) takes a borrowed pointer, so the feed is direct: a small resident set of
**collision patches** around active physics bodies, each a row-major grid of heights in the local
East-North-Up frame at that patch's centre, with the view's `center`/`orientation` placing it.

Two properties make this correct rather than convenient:

- **Patches are evaluated on the host, from the height function — never read back from the GPU.** A
  readback would put a frame of latency and a sync point into the physics step, and would not exist
  at all on the server. This is T2, and it is the reason the authority is host code.
- **A planar patch is exact enough.** Over a 256 m patch, the ellipsoid's departure from its own
  tangent plane is d²/2r = 1.3 mm on Earth, 4.7 mm on the Moon. Below any contact tolerance, so a
  patch needs no curvature term.

Patch residency follows the physics bodies, not the camera — which matters the moment anything is
simulated off-screen. Re-evaluation is triggered by a body leaving its patch or by a layer edit
(§6.2) invalidating one.

---

## 13. Determinism and the network (D3)

Mapped directly onto UHM's classes (`unified_hazard_model.md` §4.3), because the vocabulary already
exists and this is exactly the split it was built for:

| Quantity | Class | Why |
|---|---|---|
| Base tile + ordered layer stack | `Bitwise` | Integer-quantised data and an explicitly ordered composition of analytic profiles. Reproducible on any machine, and the only thing the server has to agree with the client about. |
| Base + layers + synthesized detail | `Tolerant` | The host and the GPU evaluate the same function in two languages. Agreement is pinned by a conformance test to a stated tolerance, not asserted. |
| Colour, class refinement, material blend, normals | `Cosmetic` | Never feeds a `Bitwise` node — which UHM §7's rule 2 makes a debug-build assertion rather than a convention. |

The consequence for netcode is the useful one: **layers are what replicate.** They are kilobytes,
ordered, and version-stamped; a client that has the same pak and the same layer stack computes the
same authoritative ground. No height data crosses the wire. A player who has not baked the `full`
tier still agrees with the server about the ground, because the tier changes the *resolution* of the
synthesis, not the authority — which is a property worth stating explicitly, and a constraint on
§6.4: **detail amplitude must be a function of the data depth actually present, and the
authoritative surface must be defined at the `standard` tier's depth.**

No `Execution::Handoff` registration is needed initially: terrain data flows host → device one way,
and the sim-side consumer (collision patches) is produced by the same host code. When the sim domain
eventually wants GPU-compiled tiles, §6.3's compile becomes a `Handoff` publisher and the tier
machinery already designed in UHM §6.3 applies unchanged.

---

## 14. The atmosphere seam — discharging a recorded blocker

### 14.1 What was blocked

`atmosphere_system.md` §15 blocks Phase D (orographic lift, rain shadows, föhn, valley fog, sea and
lake breezes, terrain-driven turbulence) and part of Phase B's surface model on a queryable
elevation field. §16 additionally records that the seam worth building is *"a provider of surface
properties, so a terrain or ocean system could publish a real land/sea mask"*.

### 14.2 What terrain publishes

```cpp
/// The read interface every non-render consumer of terrain uses.
/// Grid sampling, never point queries: atmosphere_system.md §1.1 records
/// point-query-per-column as the exact bottleneck that made the shipped
/// weather non-spatial, and this interface is shaped so that mistake is
/// not expressible.
class ITerrainField
{
public:
    virtual ~ITerrainField() = default;

    /// Fills a caller-provided grid — the nest's own grid, in the nest's own
    /// frame — with elevation, slope, surface class, and the land/sea mask.
    virtual void sample_grid(const GeographicGrid& grid, TerrainSamples& out) const = 0;

    /// The deepest measured depth over a region, so a consumer can state the
    /// resolution its own result is entitled to claim.
    virtual std::uint8_t data_depth(const GeographicRegion& region) const = 0;
};
```

`sample_grid` runs on the host from the height function, so it works headless — which the nest
requires. `data_depth` exists so the atmosphere can report the orography resolution it actually ran
at instead of implying a resolution it did not have.

D8's land/sea mask falls out of the same call: elevation below zero *is* sea, and the class tile
distinguishes lake from ocean from ice shelf. That satisfies §16's surface-property provider without
a water surface existing.

---

## 15. Seams published for systems not built here (D1)

Three interfaces, each defined now so the systems that consume them do not force a terrain rewrite:

- **Scatter** (vegetation, rocks, debris): `ITerrainField::sample_grid` plus a tile lifecycle signal
  — `on_tile_resident(TileAddress, ...)` / `on_tile_evicted(TileAddress)`. A scatter system
  populates and releases against tile lifetime, which is the only cadence that stays bounded.
- **Water**: the bathymetry and mask above, plus the shoreline contour a compile can emit per tile.
- **The builder**: `LayerStack::insert/remove/reorder`, the dirty-region signal, and a
  ray-against-terrain query for placement. The builder owns *what* to author; terrain owns *how* an
  authored record becomes ground. That boundary is what lets the builder be written later without
  reopening this document.

---

## 16. SOLID

- **SRP** — one reason to change each: the address algebra (`cube_sphere.hpp`), the source
  interface, the layer stack, the height function, the pak format, the tile cache, the streamer, the
  compile pass, the draw passes, the collision patch set. The quadtree does not know about Vulkan;
  the cache does not know about layers; the format does not know about the frame graph.
- **OCP** — a new body is data (`se planet bake --body …`); a new data source is an `IHeightSource`;
  a new edit kind is a `LayerOperation`; a new surface class is a palette entry. None of these open
  a shipped file.
- **LSP** — `PackHeightSource`, `InsetHeightSource`, `GeneratedHeightSource`, and
  `CompositeHeightSource` are interchangeable behind `IHeightSource`, and *installable* — a
  procedural body is configured, not compiled in.
- **ISP** — the renderer sees tiles and node frames; gameplay and the atmosphere see
  `ITerrainField`; physics sees `HeightFieldView`; the builder sees `LayerStack`. No consumer can
  reach state it has no business touching, and the point-query bottleneck of `atmosphere_system.md`
  §1.1 is structurally impossible to reintroduce.
- **DIP** — the height function depends on `IHeightSource`, not on a pak; the render passes depend
  on the frame graph's declarations, not on each other; Vulkan stays in `render/`, and
  `include/SushiEngine/terrain/` includes no graphics header at all.

---

## 17. The performance contract (GTX 1060 6 GB, 1080p internal, 60 Hz — D6)

Budgets, not measurements: nothing here is built yet, and each is the proof obligation of the phase
that makes it true.

| Stage | Where | Budget |
|---|---|---|
| Quadtree selection, ~900 nodes | CPU (host, double) | ≤ 0.30 ms |
| Collision patch evaluation | CPU (host) | ≤ 0.20 ms |
| Tile compile, ≤ 16 tiles/frame | GPU compute (async-eligible) | ≤ 0.60 ms |
| Tile upload | GPU transfer | ≤ 0.30 ms |
| Depth prepass, ~1.8 M triangles | GPU | ≤ 0.70 ms |
| Opaque draw + material synthesis | GPU | ≤ 1.60 ms |
| Shadow cascades (shallower selection) | GPU | ≤ 0.50 ms |
| **Total** | | **≤ 3.7 ms GPU, ≤ 0.5 ms CPU** |

VRAM: 241 MB tile cache + 64 MB material palette + ~8 MB buffers = **313 MB**, held under a
320 MB ceiling.
Disk: 320 MB (`compact`) / 4.5 GB (`standard`) per body.

For calibration against the frame it lands in: the atmosphere's whole simulation stack budgets ≤ 2.6
ms (`atmosphere_system.md` §12), so terrain is the larger of the two and has to be held to a number
rather than to an adjective.

**The error target is the lever, and it is now measured rather than assumed.** Node counts on the
Moon through a 60°, 16:9 frustum, at 2 048 triangles per node (measured 2026-08-01, P2a):

| Error target | 5 km altitude | 50 km | 500 km | Triangles |
|---|---|---|---|---|
| 1 px | 4 337 | 8 056 | 9 769 | 8.9 – 20.0 M |
| 2 px | 1 958 | 2 154 | 2 751 | 4.0 – 5.6 M |
| **4 px** | **1 272** | **621** | **884** | **1.3 – 2.6 M** |
| 6 px | 923 | 396 | 506 | 0.8 – 1.9 M |

The first draft of this document paired a 2 px target with a 1.6 M triangle budget; those are not
the same setting, and the measurement is what caught it. **The D6 baseline ships at 4 px**, which is
where the triangle count meets the budget above; 2 px is a quality tier for hardware that has the
fill rate for 5 M triangles. Two properties make this a comfortable knob rather than a cliff: the
count is close to flat across five orders of magnitude of altitude (CDLOD's scale invariance), and
the cost falls roughly as the square of the target.

The same table is why a frustum is not an optimisation here. Without one the selection covers the
body's far side too — 14 000 nodes at 50 km against 2 154 with one — so frustum rejection is part of
the selection's cost model, not a saving on top of it.

Three claims that must be *measured*, not assumed, and are exit criteria in §20:

1. **No frame exceeds budget during streaming.** Tile inheritance (§7.2) means a frame never waits;
   the gate is a capture showing bounded per-frame upload and compile under a fast descent from
   orbit to the ground.
2. **The one-draw-call claim holds.** A capture must show one instanced draw per body per pass, not
   one per node.
3. **The host cost is selection and nothing else.** A profile must show no per-node CPU work in the
   draw path.

---

## 18. Named limits

Stated so they are decisions rather than discoveries.

- **Global measured resolution ends at ~460 m (Earth), ~118 m (Moon), ~463 m (Mars).** Everything
  finer is either a user-selected inset or synthesis. No amount of engineering changes this; it is
  what the public data is.
- **Below the data, the terrain is invented.** Plausible, statistically matched to its surroundings,
  deterministic — and not a measurement. The editor must be able to show the boundary.
- **A height field has one surface per point: no caves, no overhangs, no natural arches.** These are
  mesh content placed on terrain, not terrain. This is a consequence of the representation and is
  not worked around.
- **The geoid is removed at bake time and never modelled at run time.** Sea level is the ellipsoid
  plus a per-body constant.
- **No erosion, no dynamic geology.** Terrain does not respond to the weather simulation. Rivers and
  glaciers, if they appear, are layers and materials — authored or generated once, not simulated.
- **Detail is `Tolerant`, not `Bitwise`, between host and device.** §13's conformance test states
  the tolerance; the authoritative surface is defined without detail.
- **The `full` tier's insets are regional, not global.** A planet uniformly at 30 m is 857 GB and is
  not on the roadmap at any phase.
- **Gas giants have no terrain.** The ellipsoid is the 1-bar level.
- **Terrain is absent from the depth prepass, so Hi-Z and GTAO do not see it.** §8.4's
  `TerrainDepthPass` is not built. The consequence is bounded and cosmetic: GPU occlusion culling
  cannot use terrain as an occluder, and ambient occlusion has no terrain contribution. Terrain
  itself still depth-tests against the meshes correctly, because it draws after them.
- **One body carries terrain at a time.** `PlanetTerrain` follows `Environment::planet_body`, and
  changing it idles the device — right for a scene-level event, wrong for the `Mesh` rung, where a
  non-dominant body also wants terrain. P9 is where that stops being one object.
- **A body's pack is loaded synchronously, on the render thread.** 17.5 MB once, on first sight of
  the body. P3 is where the loader becomes asynchronous; until then the first frame on a new world
  is long.

---

## 19. Rejected alternatives

Recorded so they are not re-proposed.

- **Equirectangular, icosahedral, or HEALPix tiling.** §4.1.
- **Hardware tessellation.** The feature is not enabled (G5), it adds a pipeline stage and a
  patch-size constraint, CDLOD morph reaches the same quality at lower cost, and — decisively — it
  does not address §9, which is the actual hard problem.
- **Sparse residency / virtual texturing for tiles.** Not enabled (G5), and an explicit slot pool
  with inheritance is simpler, portable, and gives exact control over the budget D6 demands.
- **Ray-marching the height field instead of drawing geometry.** No silhouette at grazing angles, no
  cheap collision, no path to the builder, and it fights the depth buffer the rest of the frame
  shares.
- **Baking a global 30 m pyramid.** 857 GB. §5.4.
- **Storing baked normal maps.** They carry no information the height field does not; the memory is
  better spent on a deeper height pyramid.
- **Storing edited rasters instead of layer records.** Multiplies the pak by the edited area, makes
  every edit a streaming problem, and makes network replication of edits impossible. §6.1.
- **Reading collision heights back from the GPU.** A frame of latency, a sync point in the physics
  step, and it does not exist at all on a headless server. §12.
- **Reusing `TextureLibrary` for tiles.** It is path-keyed, material-oriented, and mip-residency
  based; a slot pool with inheritance is a different lifecycle wearing the same words.
- **Doing the cube-to-sphere projection on the GPU in planet space.** §9.1: the error is introduced
  before the camera subtraction, and no amount of camera-relative bookkeeping afterwards recovers
  it.
- **Terrain as an ECS entity with a mesh component.** A planet is not an instance; it has its own
  selection, its own residency, and its own draw. The editor's existing `create_terrain` (a flat box
  with a plane collider, `docs/architecture/domain-physics.md` §1.3) stays what it is — a local
  authoring primitive, and unrelated to this system.
- **Consuming the floating-origin types (G8) for terrain.** They solve absolute simulation
  positions; §9's problem is the evaluation of a curved surface, and the per-node double frame
  solves it exactly. When SushiLoop consumes the sector types, terrain's node frames compose with
  them without change.

---

## 20. Phased roadmap

Vertical slice first (D10), Moon first (D9). Each phase's exit is a measurement or a test, never an
assertion.

| Phase | Delivers | Exit criterion |
|---|---|---|
| **P0** Algebra and authority — **landed 2026-08-01** | `cube_sphere.hpp`, `tile_address.hpp`, `IHeightSource`, `layer_stack.hpp`, `height_function.hpp`. Host only, no Vulkan, no data. | Unit tests: projection round-trips; §9.2's difference form is within 1 mm of a double reference at depth 20; address algebra including face-crossing neighbours; layer composition is order-stable under shuffled insertion. **Met** — 22 tests, plus §9.2's measured table. |
| **P1** The baker — **landed 2026-08-01** | `se planet bake` / `se planet inspect`, `pack_format.hpp`, `PackHeightSource`. | A pack whose sampled elevations match LOLA within the quantisation bound; provenance present in the header; the reader refuses a truncated blob whole. **Met** — worst deviation 0.092 m against a 0.184 m step on the compact lunar tier; 12 further tests drive every refusal from synthesized bytes. |
| **P2a** Node selection — **landed 2026-08-01** | `quadtree.hpp`: the refinement cut, camera-relative node frames, bounding volumes from the pack's bands, frustum rejection, morph ranges, a hard node budget. Host only. | The emitted set is a proper cut — every face covered exactly once, no node inside another — under refinement, under a binding budget, and against the real lunar pack. **Met**, 10 tests; and the node-count table in §17 is measured rather than assumed. |
| **P2b** The vertical slice — **draws 2026-08-02, exit not met** | `tile_residency.hpp`, `terrain.vert` + `terrain_common.glsl`, `TerrainLayout` (set 2), `TileCache`, `PlanetTerrain`, `TerrainPass`, `terrain_frame.hpp`, and the frame seam of §9.4. Registered in `VulkanSceneView` after the opaque pass; the analytic ground stands down where terrain draws. | Real lunar topography on screen. No cracks at LOD boundaries. Camera from 100 km to 10 m with no popping. GPU cost measured against §17. **The frame seam is closed and tested** (15 further tests, 69 in the terrain group): the tidal-lock check puts the Earth within 11° of the zenith from selenographic (0, 0), and an observer's own zenith reads back their own coordinates to 1e-9°. **The first rendered frame did not meet the exit criteria**, and §20.1 records what it showed and why: the camera is inside the shell (nothing places it on the terrain), the shipped tier is 2.7 km per texel, and the material is one flat colour until P7. §20.1's punch list is P2c. |
| **P3** The whole body, streaming | Six faces, async loader, inheritance, LRU, bounded per-frame work. | Fly anywhere on the Moon; no frame exceeds budget during a fast orbital descent; resident set stays under the 320 MB ceiling. |
| **P4** Collision | Collision patch set, `HeightFieldView` feed, patch residency following physics bodies. | Walk and drive on lunar terrain. A headless run reproduces the client's authoritative heights exactly. |
| **P5** The layer stack lands | Crater and flatten operations; edit → dirty region → recompile → re-cook. | Place a crater in the editor: it appears, it is collided with, and the frame budget holds. |
| **P6** Earth | Second body. Bathymetry, land/sea mask, geoid removal, ETOPO + BMNG. | Earth renders with real coastlines; `sky.frag` hands off cleanly; **no body-specific branch was added** — the T4 proof. |
| **P7** Material synthesis | Class tiles, material palette, height blend, albedo preservation, distance blend. | One metre from the ground reads as ground; the same mountain is the same colour from orbit and from its slope. |
| **P8** Detail synthesis | Deterministic sub-Nyquist detail, host and device forms. | Conformance test pins host-vs-device agreement to a stated tolerance; detail is visually absent on flat terrain and present on slopes. |
| **P9** The `Mesh` rung | Terrain on non-dominant bodies at a depth set by angular size. | Earth from the Moon shows real continents; the `Impostor` rung's procedural path retires. |
| **P10** The atmosphere seam | `ITerrainField`, grid sampling, `data_depth` reporting. | `atmosphere_system.md` §15's terrain blocker is struck; Phase D can begin. |
| **P11** Tier: GPU quadtree | Task/mesh-shader descent, gated on `supports_mesh_shader()`. | Measured reduction in host selection cost with byte-identical node selection against the host path. |

P0–P5 are the slice: after P5 there is a real, walkable, editable planetary surface. P6–P8 make it
Earth and make it beautiful. P9–P11 finish the ladder and the seams.

---

## 20.1 What the first rendered frame showed (2026-08-02)

P2b draws. It does not yet look like a planet, and the gap is worth writing down precisely
rather than filing as "shader work", because two of the three causes are not shading at all.

Observed, standing on the Moon: *a terrible skin that will not leave you alone — turn any
direction and it is still there, looking 2D rather than 3D*; and *at human scale, nothing
draws past about five metres*.

**1. The camera is inside the terrain shell.** This is the dominant cause and it is
structural, not cosmetic. `fill_environment_sky` anchors the scene origin to the observer's
point on the **reference ellipsoid**, at altitude zero — the construction predates terrain
and is correct for an analytic ground, which *is* the ellipsoid. The real lunar surface at
that point is somewhere between −9 km and +11 km away. Nothing yet resolves the observer's
altitude against the height field, so the camera starts buried (or, in a basin, kilometres
above nothing).

Being underground would normally draw nothing. It draws a skin instead because the terrain
pipeline sets `VK_CULL_MODE_NONE` — deliberately, per `terrain_pass.hpp`, "until the first
rendered frame confirms the winding", on the argument that a winding mistake with culling on
is an invisible planet and with culling off is a slightly slower one. That trade was right
for bring-up and is now backwards: with culling off, *being inside the body* renders the
shell's interior in every direction, which is exactly the reported symptom, and it hides the
real problem behind a plausible-looking one.

**2. The shipped lunar tier has 2.7 km texels.** `moon.compact.planet` stores 510 tiles,
which is 6·(1+4+16+64) — depths 0 through 3, and nothing below. At depth 3 a tile's texel is
**2665 m**. `PackHeightSource` resamples the nearest stored ancestor below that, so every
level the selector descends to past depth 3 is interpolation, not measurement. §18 already
names the Moon's *measured* floor as ~118 m; what it did not say is that the tier actually
baked is more than twenty times coarser than that floor. The `standard` tier (LOLA 64 ppd)
is 666 m per texel and is not baked; nothing in the baker reaches 118 m yet.

So even with the camera correctly placed, standing on the Moon shows a smooth, featureless
surface — there is no measured terrain at human scale to show. The render lattice is not the
constraint (at the selector's `maximum_depth` of 12 a cell is 21 m); the data is.

**3. Terrain shades as one flat colour, and will look worse than what it replaced until
P7.** The other bodies are `sky.frag`'s analytic ground, which at least has a procedural
`surface_albedo` pattern. Terrain currently pushes a single material from
`PlanetParams::ground_albedo`, because §11's class tiles, height blend and distance blend are
P7 and sub-Nyquist detail is P8. Between here and there, real geometry with no material will
read as *worse* than fake geometry with a procedural one at anything closer than a few
kilometres. That is the expected shape of the work rather than a regression, and it is stated
here so it is not mistaken for one.

### P2c — the punch list before the vertical slice can be judged

In order, because each one makes the next one observable:

1. **Put the camera on the ground.** Resolve the observer's altitude through
   `HeightFunction` rather than assuming the ellipsoid. This is the same query P4's collision
   patches need, so it is P4's first half brought forward — not throwaway work. Until it
   lands, every visual judgement about terrain is made from inside it.
2. **Turn back-face culling on**, once a frame from orbit confirms the winding. Then being
   underground draws nothing, which is a legible failure instead of a misleading one.
3. **Bake and ship a deeper tier.** `standard` at minimum; reaching §18's stated 118 m floor
   needs a source the baker does not yet know about.
4. **Show the selection.** `PlanetTerrain::statistics()` already reports node count, uploads,
   inherited count and depth, and nothing displays them. Every diagnosis above took an
   argument from a screenshot that a HUD line would have answered directly.
5. **Terrain in the depth prepass**, so Hi-Z and GTAO see it (§18).
6. Only then P7 materials and P8 detail, which is where "looks like a planet" actually lives.

---

## 21. Risks and open questions

| Risk | Mitigation |
|---|---|
| CDLOD morph does not fully close cracks in practice (the classic failure is morphing the grid but not the height sample) | P2's exit criterion is explicitly "no cracks"; §8.2 morphs both. If a residual remains, a one-cell downward skirt is the fallback, at the cost of a thin sliver of overdraw. |
| Host-vs-device detail agreement is worse than expected (fma contraction, differing rounding of the noise's integer hash) | §13 already classes it `Tolerant`, not `Bitwise`; the authoritative surface excludes detail, so the worst case is a cosmetic mismatch, not a gameplay divergence. |
| 800 nodes × 2 048 triangles is optimistic for the D6 baseline | The screen-space error target is a direct knob; the grid can drop to 17 × 17 with more nodes at equal quality. Measured at P2, before anything depends on the number. |
| The `standard` bake is 4 GB and takes hours | The `compact` tier exists precisely so the first-run experience does not require it, and P1 ships `compact` first. |
| Layer recompilation cost spikes when a large layer is edited | Recompilation is bounded per frame like every other tile operation (§7.3); a large edit resolves over several frames with inheritance covering the gap. |
| Set 2 collides with a future GPU-driven terrain path wanting the instance set | Terrain's set 2 is its own layout, built alongside `SceneLayout`'s rather than inside it; §8.3. |

**Open questions for the owner** — each is a decision, not a blocker, and each has a stated default:

1. **Layer persistence and replication format.** Layers are the builder's output and the network's
   payload. Default: they serialise into the scene file through the existing scene serializer, with
   the builder owning a separate library asset for reusable authored pieces. Confirm when the
   builder is specified.
2. **How far the `full` tier's insets go.** Default: user-named regions with a 500 km cap per inset,
   because a 30 m global bake is off the table (§18). If Sushiverse wants specific hero regions (a
   city, a mountain range) at 1 m, that is a named inset list, not a policy change.
3. **Whether the Moon's permanently-shadowed and Earth's polar regions get special treatment.** The
   data quality drops sharply above ~85° latitude on several bodies. Default: fill from the coarser
   source, mark it in the class tile, and say so.
4. **The editor's terrain surface.** Default, per the project's component-inspector convention: a
   Planet panel beside the environment panels, plus a bake/inspect readout; the builder gets its own
   surface later, and neither ever previews into the Scene view.

---

## 22. Dependencies and blockers

- **Nothing blocks P0–P5.** Every capability they need exists: the frame graph, the bindless heap,
  the depth prepass, Hi-Z occlusion, the astro frame stack, and `HeightFieldView`.
- **P10 unblocks, rather than being blocked by, `atmosphere_system.md` Phase D** and the surface-
  property provider its §16 defers.
- **`render_pipeline_refactor.md` Phase 7 (LUT stack) and Phase 11 (async compute)** are both
  shipped
  and are consumed as-is: terrain receives aerial perspective and fog through set 0, and
  `TerrainCompilePass` declares async-compute eligibility that the graph honours or ignores.
- **UHM** supplies the determinism vocabulary §13 uses. No new request to SushiRuntime, and no
  `Handoff` registration in the phases above.
- **`se` CLI** gains a `planet` command group and a `planet` extras group; `CLI_GUIDE.md` is updated
  in the same change, per `CONTRIBUTING.md` §5.
- **The pak is not committed, and this deliberately departs from the climatology precedent.**
  `assets/atmosphere/climatology.set0` *is* committed — it is 3.4 MB, and committing it is what lets
  a fresh clone run with a real mean state. A planet pak is 250 MB at `compact` and 4 GB at
  `standard`, so the applicable in-tree precedent is the other one: `assets/hrtf/*.sofa` is
  gitignored beside an `assets/hrtf/README.md` that says where the data comes from and how to get
  it. `assets/planet/` follows that — ignored, with a README naming the sources and the bake
  command. The consequence is stated plainly rather than discovered: **a fresh clone has no terrain
  until `se planet bake` has run**, and `PackHeightSource` reports its absence the way
  `load_climatology` reports a missing asset — as a fact about which source is in use, not as an
  error to abort on. With no pak, a body falls back to the analytic ground of §10, which is exactly
  what ships today.
