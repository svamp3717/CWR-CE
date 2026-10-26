# Model-free road ways

Worlds may define roads as **OSM-style shared nodes and ways**, with no road
segment P3D models required. The original object-based road system is still
loaded as before; a world may use either or both formats.

## World data

Create `<world>.ways` alongside `<world>.wrp` (in a filesystem directory
or a mounted bank). The format is UTF-8, line oriented:

```text
# Positions are local world X/Z coordinates in metres, not latitude/longitude.
node 1 100 150
node 2 160 150
node 3 220 180
node 4 160 200

# way ID, width (metres), donor model OR existing texture, shared node IDs
way 1001 8.0 donor=data3d\YOUR_ROAD.p3d 1 2 3
way 1002 6.0 data\silnice.pac 2 4
```

Each `node` has a unique integer ID and a finite X/Z world position.
Each `way` has an integer ID, a width between 0.5 and 50 metres, the
**donor P3D model or texture path**, and at least two node IDs. A way can refer to nodes declared later in the file. Duplicate
node IDs, unknown records, missing references, invalid dimensions, and
overly large files are rejected or skipped and reported in logs.

The third field supports two backwards-compatible forms:

* `donor=data3d\\YOUR_ROAD.p3d` (or just a path ending with `.p3d`):
  the engine loads the existing road model **only as a texture/material
  donor**, not as a placed road object. The donor's visible LOD supplies the
  rendered texture, and its roadway LOD supplies the road-contact texture
  (roughness, dust, and sound environment). If there is no roadway texture,
  the visible texture is used for both.
* For the **three original CWA road skins** `o\\road\\sil_new.paa`,
  `o\\road\\ces_hned.paa`, and `o\\road\\asf_new.paa`, the engine also
  finds the matching stock `sil25`, `ces25`, or `asf25` P3D when a
  texture-only way is loaded. The explicitly requested texture remains
  authoritative; the P3D contributes native vertex-lighting hints,
  downward-normal orientation, surface flags and UVs **only when the donor
  uses that same texture**. Existing `.ways` files need not be rewritten.
  Custom texture paths retain their earlier fallback.
* `data\\YOUR_ROAD.pac` / `.paa`: use the explicitly provided texture for
  both visible road geometry and ground-contact effects. Older `.ways`
  files using texture paths still work, but their default 0–1 UV mapping
  cannot reproduce donor-specific offsets, mirroring or tiling. For the
  closest visual match to an existing road model use `donor=...p3d`.

This is an **OSM-inspired native map representation**, not an OSM XML / PBF
importer. Convert geographic OSM coordinates to local terrain coordinates,
choose widths and map road texture paths, and write the node/way records.

## Rendering and simulation

* Each way is subdivided into terrain-following sections approximately
  2 metres long and triangulated across its width in 1.5-metre strips.
  Both asphalt and overlay ribbons retain the original clockwise
  front-face winding required by `RenderPassDescriptor::FrontFaceMode::CW`.
  The legacy `PolyVertices::GetNormal` helper calculates its geometric
  normal with the opposite cross-product order, so it must not be used
  as the front-face test. Changing road polygons to make that helper
  return +Y made every generated way invisible. Lighting normals are
  still independently sampled upward from terrain.
  The geometry samples the terrain at every vertex with a 7.5 cm clearance,
  rather than connecting two floating or buried edges. Long ways are split
  into render chunks that fit legacy vertex-index limits.
* The donor's **dominant upward-facing surface material**, measured by the
  sum of its polygon areas, supplies the base asphalt. Distinct narrow
  shoulder, verge, or painted-overlay materials are drawn as separate
  terrain-draped bands so a multi-texture P3D no longer becomes one colour.
  Generated faces preserve their donor P3D polygon render flags, including
  horizontal UV clamping, ordered-alpha blending and on-surface routing.
  For example, the original `wg_osmways4` donors use `0x00024102`
  (OnSurface, IsAlpha, ClampU, IsAlphaOrdered). The engine previously replaced
  this with generic NoClamp/NoZWrite flags, changing road sampling even
  when the correct donor texture was selected. The generated ribbon retains
  its own NoZWrite protection so a terrain-draped alpha layer cannot clip
  later geometry by modifying the depth buffer. A previous experiment
  splitting road rendering across two draw passes was reverted because it
  introduced clipping regressions. Repeated donor faces using the same
  texture are not blended over themselves.
  The generated road mesh also borrows the loaded donor's **decoded**
  per-vertex ClipFlags and normal orientation. Original stock road P3Ds have
  negative-Y visual vertex normals even though the renderer draws their
  clockwise faces from above. Previously every generated ribbon used positive-Y
  normals and generic ClipAll vertex hints, changing its lighting and colour.
  This correction does not flip polygon winding or affect wheel physics.
  Source ODOL and MLOD bit flags have different encodings; raw flag integers
  must not be copied directly into runtime mesh flags.
  The generated sections also resolve the original
  `CfgTextureToMaterial` mapping and retain material references through
  editor-preview transitions. The generator reconstructs donor UV tiling
  and offsets instead of
  assigning a generic 0–1 mapping. Generated roads use the Shape/material
  rendering path, not the terrain-colour-palette renderer. For the closest
  visual match, specify a `donor=...p3d` rather than a `.paa` or `.pac`
  texture. Some donor layouts with complex non-strip overlay geometry still
  require more general face projection.
* UVs repeat the donor's **visible** road texture along the way. Vehicle
  `GroundCollision` contacts and `RoadSurfaceY` use the donor's **roadway**
  texture, so existing friction/roughness, dust, ground sounds, and effects
  behave like actual roads rather than the grass underneath.
* Terrain subdivision/resampling, terrain-cache loading and terrain height
  edits rebuild the road mesh after the height changes.
* The 2D map renders outlines from the original unsampled ways in the
  standard map road colour, including at overview zoom levels.
* Road segments feed the legacy RoadNet path search. Shared nodes connect
  branches (including intersections); disconnected ways are not connected
  merely because they cross visually. The generated road footprint also marks
  the existing road geography flag.
* Existing WRP road objects and P3D models remain untouched and coexist with
  generated ways. Omitting `<world>.ways` retains the original behaviour.
  Existing world files are not rewritten.

## Limitations

* Generated ways currently follow terrain. Elevated bridges, tunnels, road
  banking, kerbs, lane markings, and arbitrary grade-separated intersections
  still require explicit modelling or future extensions.
* Terrain elevation changes refresh road vertex heights and normals **in
  place**, keeping road-link pointers stable for AI; reloading a WRP creates
  a new graph. Repeat geography initialization for editor preview reuses
  the graph when the same world is still loaded.
* Generated-way A* search uses an indexed visited-node table with the
  road pointer initialized before insertion; otherwise searches could
  revisit or miss links and fall back to cross-country routing. This avoids
  quadratic path lookups with dense, 2-metre road segments. The start of a
  car route accepts the same allowed road-approach tolerance as the end.
  For connected roads, wheeled and tracked vehicles now preserve an actual
  graph-derived polyline as strategic goals rather than reducing the route
  to start and finish and driving through fields. About every 18 metres
  (and at sharp turns), the AI gets a strategic goal. Operative vehicle
  steering independently follows the full graph route to the actual
  destination, sampled at approximately 6 metres to stay close to curved
  pavement without processing every 2-metre graph link every frame.
  It does not append the legacy straight-line strategic chords or report
  a waypoint completed merely upon reaching an intermediate road point.
  Generated road plans are explicitly marked so P3D-only plans keep their
  original behavior.
  Editor waypoints up to 35 metres from a road can still use the road and
  leave it for the final approach; vehicles can join a road from within
  12 metres. If no connected way can be found, legacy terrain A* remains
  the fallback.
  Generated road links are not locked by parked vehicles: locking several
  contiguous ~2 m links would disconnect the only road route. Terrain-cell
  occupancy and vehicle collision avoidance still apply, and legacy P3D
  road-link locking is unchanged.
* The world log reports the resolved base texture, contact texture, donor
  UV availability, and number of overlay bands once per donor, to help
  diagnose asset-specific differences.
* Changing way definitions or donor textures requires reloading the world.
* Generated ribbons use the model/material Shape renderer;
  very large road datasets will benefit from spatial batching and a dedicated
  instanced rendering path.

## Worldgen compatibility

The original `<world>.ways` syntax and loading behavior are unchanged.
**No Worldgen update or map regeneration is required** to test the stock-skin
lighting fix; rebuild CWR-CE and use the same existing PBO. If Worldgen is
later enhanced to emit `.ways` directly or supply custom donor mappings,
that would be a separate optional generator feature. Current Worldgen's
WRP/P3D object workflow is independent of engine-side `.ways` rendering.

At world load, each unique road source is logged as `Generated road material`,
including `normalDown`, `vertexClip`, `donorFlags`, and `wayFlags`.
For recognized stock skin paths with an available matching donor P3D,
`vertexStyle=true` and `normalDown=true` are expected. If they are false,
inspect the model bank and asset path resolution before changing UV or lighting
rules again.

## Source isolation

The isolated integration branch keeps new OSM-style parsing, tessellation,
donor-texture processing, road-junction wiring, and terrain-height refresh in
`engine/Poseidon/World/Terrain/RoadWays.cpp`. Original `Roads.cpp`
contains only the small hooks needed to connect the existing P3D network with
the generated network. The project CMake configuration discovers this new
source through `GLOB_RECURSE CONFIGURE_DEPENDS`.

Existing worlds with no `.ways` file retain their original P3D road system.
The production-tested procedural road/AI code is preserved; this branch is
primarily a source reorganization and still needs a full build and in-game
regression test.
