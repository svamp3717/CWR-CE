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
  The geometry samples the terrain at every vertex with a 7.5 cm clearance,
  rather than connecting two floating or buried edges. Long ways are split
  into render chunks that fit legacy vertex-index limits.
* The donor's **dominant upward-facing surface material**, measured by the
  sum of its polygon areas, supplies the base asphalt. Distinct narrow
  shoulder, verge, or painted-overlay materials are drawn as separate
  terrain-draped bands so a multi-texture P3D no longer becomes one colour.
  The generator reconstructs donor UV tiling and offsets instead of
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
* Generated-way A* search uses an indexed visited-node table. This avoids
  quadratic path lookups with dense, 2-metre road segments. The start of a
  car route accepts the same allowed road-approach tolerance as the end.
  For connected roads, wheeled and tracked vehicle strategic planning can
  use the road graph directly before falling back to the legacy terrain
  search, which may reject narrow roads on otherwise impassable cells.
* The world log reports the resolved base texture, contact texture, donor
  UV availability, and number of overlay bands once per donor, to help
  diagnose asset-specific differences.
* Changing way definitions or donor textures requires reloading the world.
* Generated ribbons currently use the compatibility terrain mesh renderer;
  very large road datasets will benefit from spatial batching and a dedicated
  instanced rendering path.
