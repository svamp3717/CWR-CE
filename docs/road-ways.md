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

# way ID, width (metres), original roadway texture, shared node IDs
way 1001 8.0 data\silnice.pac 1 2 3
way 1002 6.0 data\silnice.pac 2 4
```

Each `node` has a unique integer ID and a finite X/Z world position.
Each `way` has an integer ID, a width between 0.5 and 50 metres, the
**texture path used by the existing road P3D's roadway faces**, and at least
two node IDs. A way can refer to nodes declared later in the file. Duplicate
node IDs, unknown records, missing references, invalid dimensions, and
overly large files are rejected or skipped and reported in logs.

This is an **OSM-inspired native map representation**, not an OSM XML / PBF
importer. Convert geographic OSM coordinates to local terrain coordinates,
choose widths and map road texture paths, and write the node/way records.

## Rendering and simulation

* Each way is subdivided into terrain-following sections no more than roughly
  8 metres long. A continuous textured ribbon is generated at runtime using
  local tangent/miter joins. It is drawn with the terrain, without P3D meshes.
* UVs repeat the specified existing road texture. Road-surface queries return
  that *same Texture object*, so the existing texture-based ground material,
  sound and surface-effect logic continues to apply.
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
* Height or texture edits after a world is loaded require a road-network
  rebuild to regenerate the ribbon mesh.
* Generated ribbons currently use the compatibility terrain mesh renderer;
  very large road datasets will benefit from spatial batching and a dedicated
  instanced rendering path.
