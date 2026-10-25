
#include <Poseidon/Core/Application.hpp>
#include <Poseidon/World/Terrain/Landscape.hpp>
#include <Poseidon/World/Terrain/Roads.hpp>
#include <float.h>
#include <Poseidon/Foundation/Common/FltOpts.hpp>
#include <Poseidon/Foundation/Containers/Array2D.hpp>
#include <Poseidon/Foundation/Containers/StreamArray.hpp>
#include <Poseidon/Foundation/Framework/Log.hpp>
#include <Poseidon/Foundation/Math/MathOpt.hpp>
#include <Poseidon/Foundation/Memory/MemAlloc.hpp>
#include <Poseidon/IO/Streams/QBStream.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace Poseidon
{
using namespace Foundation;

DEFINE_FAST_ALLOCATOR(RoadLink)

DEFINE_FAST_ALLOCATOR(RoadListFull)

RoadLink::RoadLink(Object* object, Vector3* pos, int c) : _object(object), _locks(0), _nCon(c)
{
    PoseidonAssert(c <= NCon);
    for (int i = 0; i < c; i++)
    {
        _pos[i] = pos[i];
    }
    for (int i = 0; i < NCon; i++)
    {
        _con[i] = nullptr;
    }
}

RoadLink::RoadLink(Vector3Par from, Vector3Par to, float width, Texture* texture)
    : _locks(0), _nCon(2), _wayHalfWidth(width * 0.5f), _wayTexture(texture)
{
    _pos[0] = from;
    _pos[1] = to;
    for (int i = 0; i < NCon; ++i)
        _con[i] = nullptr;
}

void RoadLink::UpdateWayHeights(Landscape* land)
{
    if (!IsWay())
        return;
    for (int i = 0; i < _nCon; ++i)
        _pos[i][1] = land->SurfaceY(_pos[i].X(), _pos[i].Z()) + RoadWaySurfaceOffset;
}

// Project in the horizontal plane: generated ways follow terrain rather than
// a model's roadway LOD. This is also used by the collision surface query.
static float WayDistance2XZ(float x, float z, Vector3Par from, Vector3Par to)
{
    const float dx = to.X() - from.X();
    const float dz = to.Z() - from.Z();
    const float len2 = dx * dx + dz * dz;
    if (len2 <= 1e-8f)
        return std::numeric_limits<float>::max();
    const float t = std::clamp(((x - from.X()) * dx + (z - from.Z()) * dz) / len2, 0.0f, 1.0f);
    const float ox = x - from.X() - t * dx;
    const float oz = z - from.Z() - t * dz;
    return ox * ox + oz * oz;
}

bool RoadLink::IsInside(Vector3Val pos, float size) const
{
    if (_wayHalfWidth > 0)
        return WayDistance2XZ(pos.X(), pos.Z(), _pos[0], _pos[1]) <= Square(_wayHalfWidth + floatMax(size, 0));
    Object* obj = _object;
    if (!obj)
    {
        return false;
    }
    float minDist2 = 1e10;
    LODShape* lShape = obj->GetShape();
    Shape* shape = lShape->RoadwayLevel();
    if (!shape)
    {
        RptF("No roadway in road %s", obj->GetShape()->Name());
        return false;
    }
    float centerDist2 = (pos - obj->Position()).SquareSizeXZ();
    if (centerDist2 > Square(lShape->BoundingSphere() + size))
    {
        return false;
    }
    Matrix4 invTrans = obj->GetInvTransform();
    Vector3 rPos = invTrans.FastTransform(pos);
    for (Offset f = shape->BeginFaces(); f < shape->EndFaces(); shape->NextFace(f))
    {
        const Poly& face = shape->Face(f);
        float dist2 = face.DistanceFromTop(*shape, rPos);
        saturateMin(minDist2, dist2);
    }
    return minDist2 < Square(size);
}

inline RoadList& RoadNet::SelectRoadList(float x, float z)
{
    int xx = toIntFloor(x * InvLandGrid);
    int zz = toIntFloor(z * InvLandGrid);
    if (!InRange(xx, zz))
    {
        // find nearest in-range square and use it
        if (xx < 0)
        {
            xx = 0;
        }
        else if (xx > LandRangeMask)
        {
            xx = LandRangeMask;
        }
        if (zz < 0)
        {
            zz = 0;
        }
        else if (zz > LandRangeMask)
        {
            zz = LandRangeMask;
        }
        PoseidonAssert(InRange(xx, zz));
    }
    return _roads(xx, zz);
}

// build road net information

RoadNet::RoadNet() = default;
RoadNet::~RoadNet() = default;

void RoadNet::Scan(Landscape* land)
{
    // Array2D::Dim preserves old elements when dimensions match! A rebuild
    // otherwise keeps dead/duplicate SRef<RoadLink> objects and cross-links
    // them to new ways, eventually corrupting AI road traversal.
    _roads.Clear();
    _roads.Dim(LandRange, LandRange);
    // insert all road objects into the road list
    int x, z;
    for (x = 0; x < LandRange; x++)
    {
        for (z = 0; z < LandRange; z++)
        {
            const ObjectList& list = land->GetObjects(z, x);
            for (int o = 0; o < list.Size(); o++)
            {
                Object* obj = list[o];
                if (obj->GetType() != Network)
                {
                    continue;
                }
                // consider only networks
                // scan cross point name pairs
                const int MaxPos = 4;
                static const char* pairs[MaxPos * 2] = {
                    "LB", "PB", "LE", "PE", // normal
                    "LD", "LH", "PD", "PH", // crossing
                };
                Shape* shape = obj->GetShape()->MemoryLevel();
                Vector3 pos[MaxPos];
                Vector3 avg = obj->Position();
                // float avgDistLR=0;
                int c = 0;
                if (!shape)
                {
                    // synthetise LB, PB, LE, PE points from nothing if possible
                    // use geometry min-max
                    Shape* geom = obj->GetShape()->GeometryLevel();
                    if (!geom)
                    {
                        continue;
                    }
                    LOG_DEBUG(World, "Road shape {} missing connection points", obj->GetShape()->Name());
                    Vector3Val min = geom->Min();
                    Vector3Val max = geom->Max();
                    Vector3Val cnt = (min + max) * 0.5;

                    Vector3 sposB = Vector3(cnt.X(), max.Y(), min.Z());
                    Vector3 sposE = Vector3(cnt.X(), max.Y(), max.Z());
                    obj->PositionModelToWorld(sposB, sposB);
                    obj->PositionModelToWorld(sposE, sposE);
                    pos[0] = sposB;
                    pos[1] = sposE;
                    c = 2;
                    // Vector3Val rPos=shape->NamedPosition(r);
                }
                else
                {
                    for (c = 0; c < sizeof(pairs) / sizeof(*pairs) / 2; c++)
                    {
                        const char *l = pairs[c * 2], *r = pairs[c * 2 + 1];
                        if (shape->FindNamedSel(l) < 0)
                        {
                            break;
                        }
                        if (shape->FindNamedSel(r) < 0)
                        {
                            break;
                        }
                        Vector3Val lPos = shape->NamedPosition(l);
                        Vector3Val rPos = shape->NamedPosition(r);
                        // float distLR=lPos.Distance(rPos);
                        // avgDistLR+=distLR;
                        Vector3 spos = (lPos + rPos) * 0.5;
                        obj->PositionModelToWorld(spos, spos);
                        pos[c] = spos;
                    }
                    // avgDistLR/=c;
                }
                RoadLink* elem = nullptr;
                if (c <= 4)
                {
                    elem = new RoadLink(obj, pos, c);
                }
                if (!elem)
                {
                    Fail("Bad road element.");
                    continue;
                }

                // insert into corresponding square
                RoadList& list = SelectRoadList(avg.X(), avg.Z());
                list.Add(elem);
            }
        }
    }
}

namespace
{
struct ParsedWay
{
    float width;
    std::string texture;
    std::vector<long long> nodes;
};

// OSM-like text sidecar: 'node ID X Z' and
// 'way ID WIDTH TEXTURE NODE_ID NODE_ID [...]'.
// References can appear before or after their node declarations.
bool ReadWayDefinitions(const std::string& data,
                        std::unordered_map<long long, Vector3>& nodes,
                        std::vector<ParsedWay>& ways)
{
    std::istringstream file(data);
    std::string line;
    size_t lineCount = 0;
    while (std::getline(file, line))
    {
        if (++lineCount > 200000 || line.size() > 8192)
            return false;
        const size_t comment = line.find('#');
        if (comment != std::string::npos)
            line.resize(comment);
        std::istringstream row(line);
        std::string type;
        if (!(row >> type))
            continue;
        if (type == "node")
        {
            long long id;
            float x, z;
            std::string extra;
            if (!(row >> id >> x >> z) || (row >> extra) ||
                !std::isfinite(x) || !std::isfinite(z))
                return false;
            if (nodes.size() >= 100000 || !nodes.emplace(id, Vector3(x, 0, z)).second)
                return false;
        }
        else if (type == "way")
        {
            long long id, nodeId;
            ParsedWay way;
            if (!(row >> id >> way.width >> way.texture) ||
                !std::isfinite(way.width) || way.width < 0.5f || way.width > 50.f)
                return false;
            while (row >> nodeId)
            {
                if (way.nodes.size() >= 4096)
                    return false;
                way.nodes.push_back(nodeId);
            }
            if (!row.eof() || way.nodes.size() < 2 || ways.size() >= 10000)
                return false;
            ways.push_back(std::move(way));
        }
        else
            return false; // reject typos, never silently drop road definitions
    }
    return true;
}

// Do not take the *first* face of a P3D model. Roads may start with an
// underside or a shoulder/edge face, with a different (often dark) texture.
// GetAreaTop measures the projected, upward-facing area, so the dominant
// driveable face determines the surface appearance.
struct RoadDonorAppearance
{
    Texture* texture = nullptr;
    int special = 0;
    bool hasUV = false;
    float uX = 0, uZ = 0, uC = 0;
    float vX = 0, vZ = 0, vC = 0;
    float xMin = 0, xMax = 0, zMin = 0;
};

// Recover the source P3D face's *actual* UV projection. Blindly mapping U
// to [0..1] and V to distance/width sampled the wrong part of road textures
// whose donor meshes tile, mirror or offset their UV coordinates.
void ReadDonorUV(RoadDonorAppearance& out, const Shape& shape, const Poly& poly)
{
    if (!out.texture || poly.N() < 3)
        return;
    out.xMin = 1e10f;
    out.xMax = -1e10f;
    out.zMin = 1e10f;
    for (int k = 0; k < poly.N(); ++k)
    {
        const Vector3 p = shape.Pos(poly.GetVertex(k));
        out.xMin = floatMin(out.xMin, p.X());
        out.xMax = floatMax(out.xMax, p.X());
        out.zMin = floatMin(out.zMin, p.Z());
    }
    if (out.xMax - out.xMin < 0.001f)
        return;

    const int i0 = poly.GetVertex(0);
    const Vector3 p0 = shape.Pos(i0);
    const float u0 = out.texture->UToLogical(shape.U(i0));
    const float v0 = out.texture->VToLogical(shape.V(i0));
    for (int k = 2; k < poly.N(); ++k)
    {
        const int i1 = poly.GetVertex(k - 1), i2 = poly.GetVertex(k);
        const Vector3 p1 = shape.Pos(i1), p2 = shape.Pos(i2);
        const float dx1 = p1.X() - p0.X(), dz1 = p1.Z() - p0.Z();
        const float dx2 = p2.X() - p0.X(), dz2 = p2.Z() - p0.Z();
        const float determinant = dx1 * dz2 - dx2 * dz1;
        if (std::fabs(determinant) < 1e-5f)
            continue;

        const float du1 = out.texture->UToLogical(shape.U(i1)) - u0;
        const float du2 = out.texture->UToLogical(shape.U(i2)) - u0;
        const float dv1 = out.texture->VToLogical(shape.V(i1)) - v0;
        const float dv2 = out.texture->VToLogical(shape.V(i2)) - v0;
        out.uX = (du1 * dz2 - du2 * dz1) / determinant;
        out.uZ = (dx1 * du2 - dx2 * du1) / determinant;
        out.vX = (dv1 * dz2 - dv2 * dz1) / determinant;
        out.vZ = (dx1 * dv2 - dx2 * dv1) / determinant;
        out.uC = u0 - out.uX * p0.X() - out.uZ * p0.Z();
        out.vC = v0 - out.vX * p0.X() - out.vZ * p0.Z();
        out.hasUV = std::isfinite(out.uX) && std::isfinite(out.uZ) &&
            std::isfinite(out.vX) && std::isfinite(out.vZ) &&
            std::isfinite(out.uC) && std::isfinite(out.vC);
        if (out.hasUV)
            return;
    }
}

RoadDonorAppearance FindDonorTopFace(Shape* shape)
{
    RoadDonorAppearance result;
    if (!shape)
        return result;

    float largestTopArea = 0.f;
    Offset selected = shape->EndFaces();
    for (Offset face = shape->BeginFaces(); face < shape->EndFaces(); shape->NextFace(face))
    {
        const Poly& polygon = shape->Face(face);
        Texture* texture = polygon.GetTexture();
        if (!texture)
            continue;
        const float topArea = polygon.GetAreaTop(*shape);
        if (topArea > largestTopArea)
        {
            largestTopArea = topArea;
            selected = face;
            result.texture = texture;
            result.special = polygon.Special() | shape->Special();
        }
    }

    // Older P3Ds sometimes reverse the winding of their visible top faces.
    // Pick the largest horizontal face regardless of winding, not the first
    // textured face (which might still be an underside/kerb).
    if (!result.texture)
    {
        float largestXZArea = 0.f;
        for (Offset face = shape->BeginFaces(); face < shape->EndFaces(); shape->NextFace(face))
        {
            const Poly& polygon = shape->Face(face);
            if (!polygon.GetTexture() || polygon.N() < 3)
                continue;
            const Vector3 origin = shape->Pos(polygon.GetVertex(0));
            float area = 0.f;
            for (int k = 2; k < polygon.N(); ++k)
            {
                const Vector3 a = shape->Pos(polygon.GetVertex(k - 1)) - origin;
                const Vector3 b = shape->Pos(polygon.GetVertex(k)) - origin;
                area += std::fabs(a.X() * b.Z() - a.Z() * b.X()) * 0.5f;
            }
            if (area > largestXZArea)
            {
                largestXZArea = area;
                selected = face;
                result.texture = polygon.GetTexture();
                result.special = polygon.Special() | shape->Special();
            }
        }
    }
    if (selected != shape->EndFaces())
        ReadDonorUV(result, *shape, shape->Face(selected));
    return result;
}

bool IsDonorName(const std::string& name)
{
    if (name.compare(0, 6, "donor=") == 0)
        return true;
    if (name.size() < 4)
        return false;
    const std::string suffix = name.substr(name.size() - 4);
    return suffix == ".p3d" || suffix == ".P3D";
}

Vector3 WayPerpendicular(Vector3Par from, Vector3Par to)
{
    const float dx = to.X() - from.X();
    const float dz = to.Z() - from.Z();
    const float length = std::sqrt(dx * dx + dz * dz);
    if (length <= 1e-4f)
        return VZero;
    return Vector3(-dz / length, 0, dx / length);
}
} // namespace

void RoadNet::LoadWays(Landscape* land)
{
    _waySegments.clear();
    _wayMapSegments.clear();
    _wayMeshes.clear();
    const char* name = land->GetName();
    if (!name || !*name)
        return;
    std::string path(name);
    const size_t dot = path.find_last_of('.');
    if (dot != std::string::npos &&
        (path.substr(dot) == ".wrp" || path.substr(dot) == ".WRP"))
        path.resize(dot);
    path += ".ways";
    if (!QIFStreamB::FileExist(path.c_str()))
        return; // maps without sidecars keep their P3D roads unchanged

    QIFStreamB file;
    file.AutoOpen(path.c_str());
    if (file.fail())
        return;
    const int byteCount = file.rest();
    if (byteCount <= 0 || byteCount > 8 * 1024 * 1024)
    {
        LOG_WARN(World, "Invalid road way file size: {}", path);
        return;
    }
    std::string contents(static_cast<size_t>(byteCount), '\0');
    file.read(contents.data(), byteCount);
    if (file.fail())
    {
        LOG_WARN(World, "Unable to read road ways: {}", path);
        return;
    }

    std::unordered_map<long long, Vector3> nodes;
    std::vector<ParsedWay> ways;
    if (!ReadWayDefinitions(contents, nodes, ways))
    {
        LOG_WARN(World, "Invalid road way definitions: {}", path);
        return;
    }

    // Generate textured terrain-following ribbons and short road links. The
    // latter join the original P3D links using RoadNet::Connect's endpoint logic.
    for (const ParsedWay& way : ways)
    {
        std::vector<Vector3> points;
        bool valid = true;
        for (long long nodeId : way.nodes)
        {
            auto it = nodes.find(nodeId);
            if (it == nodes.end())
            {
                valid = false;
                break;
            }
            if (points.empty())
            {
                points.push_back(it->second);
                continue;
            }
            Vector3 from = points.back(); // copy: appending below can reallocate points
            Vector3Val to = it->second;
            const float dx = to.X() - from.X(), dz = to.Z() - from.Z();
            const float length = std::sqrt(dx * dx + dz * dz);
            if (!std::isfinite(length) || length > 100000.f ||
                from.X() < 0.f || from.Z() < 0.f ||
                to.X() < 0.f || to.Z() < 0.f ||
                from.X() >= land->GetLandRange() * land->GetLandGrid() ||
                from.Z() >= land->GetLandRange() * land->GetLandGrid() ||
                to.X() >= land->GetLandRange() * land->GetLandGrid() ||
                to.Z() >= land->GetLandRange() * land->GetLandGrid())
            {
                valid = false;
                break;
            }
            if (length < 0.01f)
                continue;
            // Four-metre terrain cells and sharp hills need finer than the old
            // 8-metre ribbon sections or terrain triangles poke through.
            const float stepLength = floatMin(2.0f, land->GetTerrainGrid() * 0.25f);
            const int steps = std::max(1, static_cast<int>(std::ceil(length / stepLength)));
            if (steps > 8192 || points.size() + static_cast<size_t>(steps) > 16000)
            {
                valid = false;
                break;
            }
            for (int i = 1; i <= steps; ++i)
            {
                const float t = static_cast<float>(i) / steps;
                points.emplace_back(from.X() + dx * t, 0, from.Z() + dz * t);
            }
        }
        if (!valid || points.size() < 2)
        {
            LOG_WARN(World, "Skipping way with missing nodes or excessive length in {}", path);
            continue;
        }

        Ref<Texture> texture;
        Ref<Texture> surfaceTexture;
        RoadDonorAppearance visual;
        int donorSpecial = 0;
        if (IsDonorName(way.texture))
        {
            const std::string donorPath = way.texture.compare(0, 6, "donor=") == 0
                ? way.texture.substr(6) : way.texture;
            Ref<LODShapeWithShadow> donor = Shapes.New(donorPath.c_str(), false, true);
            if (donor)
            {
                // The top visual face supplies the exact road appearance.
                // The donor roadway LOD supplies ground contact properties.
                if (donor->NLevels() > 0)
                {
                    visual = FindDonorTopFace(donor->LevelOpaque(0));
                    texture = visual.texture;
                    donorSpecial = visual.special;
                }
                surfaceTexture = FindDonorTopFace(donor->RoadwayLevel()).texture;
            }
            if (!surfaceTexture)
                surfaceTexture = texture;
            if (!texture)
                texture = surfaceTexture;
        }
        else
        {
            texture = GlobLoadTexture(way.texture.c_str());
            surfaceTexture = texture;
        }
        if (!texture || !surfaceTexture)
        {
            LOG_WARN(World, "Skipping road way with missing donor/texture: {}", way.texture);
            continue;
        }

        // Keep original unsampled edges for the 2D map.
        for (size_t i = 1; i < way.nodes.size(); ++i)
        {
            const Vector3 from = nodes.at(way.nodes[i - 1]);
            const Vector3 to = nodes.at(way.nodes[i]);
            if ((to - from).SquareSizeXZ() > 0.0001f)
                _wayMapSegments.push_back({from, to, way.width});
        }

        // Tessellate across road width as well as along the centerline.
        // Two edge-only vertices made the middle of a road cut into hills,
        // even when both edges were draped onto the ground.
        const int across = std::clamp(static_cast<int>(std::ceil(way.width / 1.5f)), 2, 40);
        const int rowSize = across + 1;
        // Chunk before reaching the legacy 16-bit vertex-index limit.
        const size_t maxRows = static_cast<size_t>(16000 / rowSize);
        std::vector<float> uvDistance(points.size(), 0.f);
        for (size_t i = 1; i < points.size(); ++i)
        {
            const float dx = points[i].X() - points[i - 1].X();
            const float dz = points[i].Z() - points[i - 1].Z();
            uvDistance[i] = uvDistance[i - 1] + std::sqrt(dx * dx + dz * dz);
        }
        for (size_t first = 0; first + 1 < points.size(); )
        {
            const size_t last = std::min(first + maxRows, points.size()); // exclusive
            const int rows = static_cast<int>(last - first);
            auto mesh = std::make_unique<Shape>();
            mesh->Reserve(rows * rowSize);
            mesh->ReserveFaces((rows - 1) * across * 2);
            mesh->RegisterTexture(texture, 1.0f);

            Poly poly;
            poly.Init();
            poly.SetTexture(texture);
            // Road P3Ds frequently use alpha-blended top textures: dirt,
            // asphalt and shoulders are intentionally blended over terrain.
            // The original ribbon was forced opaque, exposing dark texels
            // instead of their alpha-composited colour.
            int surfaceFlags = NoClamp | NoZWrite;
            if ((donorSpecial & (IsAlpha | IsTransparent)) != 0 ||
                texture->GetAlphaClass() == AlphaStats::Blend)
                surfaceFlags |= IsAlpha;
            poly.SetSpecial(surfaceFlags);
            for (size_t i = first; i < last; ++i)
            {
                const Vector3 before = i == 0 ? WayPerpendicular(points[0], points[1])
                    : WayPerpendicular(points[i - 1], points[i]);
                const Vector3 after = i + 1 == points.size() ? before
                    : WayPerpendicular(points[i], points[i + 1]);
                Vector3 perp = before + after;
                const float norm = std::sqrt(perp.SquareSizeXZ());
                if (norm < 1e-4f)
                    perp = after;
                else
                    perp *= 1.0f / norm;
                const float dot = std::fabs(perp.X() * after.X() + perp.Z() * after.Z());
                const float halfWidth = way.width * 0.5f / floatMax(dot, 0.4f);
                for (int k = 0; k <= across; ++k)
                {
                    const float u = static_cast<float>(k) / across;
                    Vector3 pos = points[i] + perp * (halfWidth * (1.f - 2.f * u));
                    float dx = 0.f, dz = 0.f;
                    pos[1] = land->SurfaceY(pos.X(), pos.Z(), &dx, &dz) + RoadWaySurfaceOffset;
                    Vector3 normal(-dx, 1, -dz);
                    normal.Normalize();
                    float uu = u, vv = uvDistance[i] / way.width;
                    if (visual.hasUV)
                    {
                        // Stretch the donor's top-surface cross-section over the
                        // user-selected width, while repeating the donor's
                        // longitudinal UV slope in physical metres.
                        const float donorX = visual.xMin + (visual.xMax - visual.xMin) * u;
                        const float donorZ = visual.zMin + uvDistance[i];
                        uu = visual.uC + visual.uX * donorX + visual.uZ * donorZ;
                        vv = visual.vC + visual.vX * donorX + visual.vZ * donorZ;
                    }
                    mesh->AddVertexFast(pos, normal, ClipAll,
                                        texture->UToPhysical(uu), texture->VToPhysical(vv));
                }

                if (i > first)
                    for (int k = 0; k < across; ++k)
                    {
                        const int prev = static_cast<int>(i - first - 1) * rowSize + k;
                        const int next = static_cast<int>(i - first) * rowSize + k;
                        poly.SetN(3);
                        poly.Set(0, prev); poly.Set(1, next); poly.Set(2, prev + 1);
                        mesh->AddFace(poly);
                        poly.Set(0, prev + 1); poly.Set(1, next); poly.Set(2, next + 1);
                        mesh->AddFace(poly);
                    }
            }
            mesh->CalculateMinMax();
            mesh->FindSections(true);
            _wayMeshes.push_back(std::move(mesh));
            first = last - 1; // overlap one row for continuous sections
        }

        for (size_t i = 1; i < points.size(); ++i)
        {
            Vector3 from = points[i-1], to = points[i];
            from[1] = land->SurfaceY(from.X(), from.Z()) + RoadWaySurfaceOffset;
            to[1] = land->SurfaceY(to.X(), to.Z()) + RoadWaySurfaceOffset;
            RoadWaySegment segment{from, to, way.width, texture, surfaceTexture};
            _waySegments.push_back(segment);
            SelectRoadList((from.X() + to.X()) * 0.5f, (from.Z() + to.Z()) * 0.5f)
                .Add(new RoadLink(from, to, way.width, surfaceTexture));
        }
    }
    LOG_DEBUG(World, "Loaded {} generated road way segments from {}", _waySegments.size(), path);
}

bool RoadNet::WaySurface(float x, float z, float tolerance, Texture** texture) const
{
    if (_waySegments.empty())
        return false;
    const int xx = toIntFloor(x * InvLandGrid);
    const int zz = toIntFloor(z * InvLandGrid);
    for (int zi = zz - 1; zi <= zz + 1; ++zi)
        for (int xi = xx - 1; xi <= xx + 1; ++xi)
        {
            if (!InRange(xi, zi))
                continue;
            const RoadList& roads = _roads(xi, zi);
            for (int i = 0; i < roads.Size(); ++i)
            {
                const RoadLink* link = roads[i];
                if (!link->IsWay())
                    continue;
                if (link->IsInside(Vector3(x, 0, z), tolerance))
                {
                    if (texture)
                        *texture = link->WayTexture();
                    return true;
                }
            }
        }
    return false;
}

Vector3 RoadLink::GetCenter() const
{
    Vector3 center = VZero;
    for (int i = 0; i < _nCon; i++)
    {
        center += _pos[i];
    }
    center *= 1.0 / _nCon;

    return center;
}

float RoadLink::NearestConnectionDist2(Vector3Val pos) const
{
    float dist2Min = 1e10;
    for (int i = 0; i < _nCon; i++)
    {
        float dist2 = (_pos[i] - pos).SquareSizeXZ();
        if (dist2 < dist2Min)
        {
            dist2Min = dist2;
        }
    }
    return dist2Min;
}

// copy and add connection

void RoadLink::AddConnection(Vector3Par pos, RoadLink* con)
{
    if (_nCon >= NCon)
    {
        Fail("No more slot to add connection");
        return;
    }
    _con[_nCon] = con;
    _pos[_nCon] = pos;
    _nCon++;
}

static bool IsConnected(const RoadLink* tItem, const RoadLink* item, int maxDepth)
{
    --maxDepth;
    for (int jj = 0; jj < tItem->NConnections(); jj++)
    {
        const RoadLink* cItem = tItem->Connections()[jj];
        if (cItem == item)
        {
            return true;
        }
        if (maxDepth > 0 && cItem)
        {
            if (IsConnected(cItem, item, maxDepth))
            {
                return true;
            }
        }
    }
    return false;
}

#define DIAGS 0
#define DIAG_COUNT 0
void RoadNet::Connect()
{
#if DIAG_COUNT
    int count[4];
    memset(count, 0, sizeof(count));
#endif
    // connect as necessary
    const float maxDistCon = 1.0;
    for (int x = 0; x < LandRange; x++)
    {
        for (int z = 0; z < LandRange; z++)
        {
            RoadList& list = _roads(x, z);
            for (int o = 0; o < list.Size(); o++)
            {
                RoadLink* road = list[o];
                int nCon = road->NConnections();
#if DIAG_COUNT
                count[nCon - 1]++;
#endif
                const Vector3* pos = road->PosConnections();
                RoadLink* const* conn = road->Connections();
                // if there is already some connection, skip it
                for (int i = 0; i < nCon; i++)
                {
                    if (!conn[i])
                    {
                        Vector3Val val = pos[i];
                        // search all other roads for same position
                        // there should be max. one present
                        for (int xx = x - 1; xx <= x + 1; xx++)
                        {
                            for (int zz = z - 1; zz <= z + 1; zz++)
                            {
                                if (InRange(xx, zz))
                                {
                                    RoadList& search = _roads(xx, zz);
                                    for (int si = 0; si < search.Size(); si++)
                                    {
                                        RoadLink* sRoad = search[si];
                                        if (sRoad == road)
                                        {
                                            continue;
                                        }
                                        int sCon = sRoad->NConnections();
                                        const Vector3* sPos = sRoad->PosConnections();
                                        for (int ii = 0; ii < sCon; ii++)
                                        {
                                            float dist2 = (sPos[ii] - val).SquareSizeXZ();
                                            if (!sRoad->Connections()[ii] && dist2 < Square(maxDistCon))
                                            {
#if DIAGS
                                                LOG_DEBUG(World, "Connect {:x}:{} - {:x}:{}", (uintptr_t)road, i,
                                                          (uintptr_t)sRoad, ii);
#endif
                                                // connection found - mark it

                                                // connect both together
                                                road->SetConnection(i, sRoad);
                                                sRoad->SetConnection(ii, road);
                                                goto ConnFound;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    ConnFound:;
                    }
                }
            }
        }
    }
#if DIAG_COUNT
    {
        // some diagnostic information
        for (int i = 0; i < 4; i++)
        {
            LOG_DEBUG(World, "Road {} dirs: {}", i + 1, count[i]);
        }
    }
#endif
    // second pass: try to connect road unconnected points to something

    const float maxDistConPass2 = 7.0;
    for (int x = 0; x < LandRange; x++)
    {
        for (int z = 0; z < LandRange; z++)
        {
            RoadList& list = _roads(x, z);

            int n = list.Size();
            for (int i = 0; i < n; i++)
            {
                RoadLink* item = list[i];
                // OSM ways connect only through shared endpoints. Do not
                // snap nearby parallel lanes or grade-separated crossings.
                if (item->IsWay())
                    continue;
                for (int j = 0; j < item->NConnections(); j++)
                {
                    if (!item->Connections()[j])
                    {
                        // something not connected
                        // try to find some connection point
                        // scan all near road points (+-1)
                        Vector3 pos = item->PosConnections()[j];
                        for (int xx = x - 1; xx <= x + 1; xx++)
                        {
                            for (int zz = z - 1; zz <= z + 1; zz++)
                            {
                                if (!InRange(xx, zz))
                                {
                                    continue;
                                }
                                //
                                const RoadList& tList = _roads(xx, zz);
                                int nn = tList.Size();
                                for (int ii = 0; ii < nn; ii++)
                                {
                                    // check connection with all connection points of tItem
                                    RoadLink* tItem = tList[ii];
                                    if (tItem->IsWay() || tItem == item)
                                    {
                                        continue;
                                    }
                                    // skip item
                                    // skip items that are directly connected
                                    if (IsConnected(tItem, item, 1))
                                    {
#if DIAGS
                                        char buf[256];
                                        PositionToAA11(pos, buf);
                                        LOG_DEBUG(World, "{}: Already Connected {} to {}", buf, item->GetObject()->ID(),
                                                  tItem->GetObject()->ID());
#endif
                                        continue;
                                    }

                                    float nearestDist2 = 1e10;
                                    int nearestJJ = -1;
                                    Vector3 nearestPos;
                                    for (int jj = 0; jj < tItem->NConnections(); jj++)
                                    {
                                        // check distance to item point
                                        Vector3Val tPos = tItem->PosConnections()[jj];
                                        float dist2 = pos.Distance2Inline(tPos);
                                        if (dist2 < nearestDist2)
                                        {
                                            nearestDist2 = dist2;
                                            nearestJJ = jj;
                                            nearestPos = tPos;
                                        }
                                    } // for (jj)
                                    if (nearestDist2 < Square(maxDistConPass2))
                                    {
// connect with nearestJJ
#if DIAGS
                                        char buf[256];
                                        PositionToAA11(pos, buf);
                                        LOG_DEBUG(World, "{}: Connect {} with {} dist {:.2f}", buf,
                                                  item->GetObject()->ID(), tItem->GetObject()->ID(),
                                                  sqrt(nearestDist2));
#endif
                                        if (!tItem->Connections()[nearestJJ])
                                        {
                                            // connection is empty
                                            item->SetConnection(j, tItem);
                                            tItem->SetConnection(nearestJJ, item);
                                        }
                                        else
                                        {
                                            // connection not empty - we need to add another one
                                            // LOG_DEBUG(World, "  not empty");
                                            item->SetConnection(j, tItem);
                                            tItem->AddConnection(nearestPos, item);
                                            // create back connection
                                        }
                                        goto Connected;
                                    }
                                    else
                                    {
#if DIAGS
                                        if (nearestDist2 < Square(20))
                                        {
                                            char buf[256];
                                            PositionToAA11(pos, buf);
                                            LOG_DEBUG(World, "{}: No Connect {} to {} dist {:.2f}", buf,
                                                      item->GetObject()->ID(), tItem->GetObject()->ID(),
                                                      sqrt(nearestDist2));
                                        }
#endif
                                    }
                                } // for (ii)
                            } // for(xx,zz)
                        }
                    Connected:;
                    }
                } // for (j)
            } // for(i)
        } // for(x,z)
    }
}

// At a shared OSM node more than two road edges may meet. Legacy connection
// slots hold one neighbour per endpoint, so add extra slots for each additional
// branch without inserting junction P3D models.
void RoadNet::ConnectWayJunctions()
{
    if (_waySegments.empty())
        return;
    for (int x = 0; x < LandRange; ++x)
        for (int z = 0; z < LandRange; ++z)
        {
            const RoadList& list = _roads(x, z);
            for (int i = 0; i < list.Size(); ++i)
            {
                RoadLink* link = list[i];
                if (!link->IsWay())
                    continue;
                for (int end = 0; end < 2; ++end)
                {
                    const Vector3 p = link->PosConnections()[end];
                    for (int xx = x - 1; xx <= x + 1; ++xx)
                        for (int zz = z - 1; zz <= z + 1; ++zz)
                        {
                            if (!InRange(xx, zz))
                                continue;
                            const RoadList& near = _roads(xx, zz);
                            for (int j = 0; j < near.Size(); ++j)
                            {
                                RoadLink* other = near[j];
                                if (other == link || !other->IsWay())
                                    continue;
                                bool sameNode = false;
                                for (int k = 0; k < 2; ++k)
                                    if ((p - other->PosConnections()[k]).SquareSizeXZ() < 0.01f * 0.01f)
                                        sameNode = true;
                                if (!sameNode)
                                    continue;
                                bool aToB = false, bToA = false;
                                for (int k = 0; k < link->NConnections(); ++k)
                                    aToB |= link->Connections()[k] == other;
                                for (int k = 0; k < other->NConnections(); ++k)
                                    bToA |= other->Connections()[k] == link;
                                if ((!aToB && link->NConnections() >= link->MaxConnections()) ||
                                    (!bToA && other->NConnections() >= other->MaxConnections()))
                                    continue;
                                if (!aToB)
                                    link->AddConnection(p, other);
                                if (!bToA)
                                    other->AddConnection(p, link);
                            }
                        }
                }
            }
        }
}

void RoadNet::Optimize()
{
    // merge small straight elements
}
void RoadNet::Compact()
{
    // optimize memory image
}

struct RoadNode
{
    RoadNode* parent;
    RoadLink* road;
    Vector3 pos;
    float cost;
    float heur;
    bool open;

    USE_FAST_ALLOCATOR
};

DEFINE_FAST_ALLOCATOR(RoadNode);

bool operator<(RoadNode& a, RoadNode& b)
{
    return a.cost + a.heur < b.cost + b.heur;
}

bool operator<=(RoadNode& a, RoadNode& b)
{
    return a.cost + a.heur <= b.cost + b.heur;
}

typedef HeapArray<RoadNode*, MemAllocSS> RoadOpenList;

class RoadNodeContainer : public AutoArray<SRef<RoadNode>>
{
  public:
    RoadNode* Find(RoadLink* link) const;
};

RoadNode* RoadNodeContainer::Find(RoadLink* link) const
{
    int i, n = Size();
    for (i = 0; i < n; i++)
    {
        RoadNode* node = Get(i);
        if (node->road == link)
        {
            return node;
        }
    }
    return nullptr;
}

inline float RoadCost(Vector3Par from, Vector3Par to)
{
    return (to - from).Size();
}

inline float RoadHeur(Vector3Par from, Vector3Par to)
{
    return (to - from).Size() * 0.5;
}
#define MAX_ITER 100
#define COST_NO_ROAD 10.0f

#undef DIAGS
#define DIAGS 0

bool RoadNet::SearchPath(Vector3Par from, Vector3Par to, RoadPathArray& path, float prec) const
{
    // maintain "open" list
    // (close list?)

    // search from...to given coordinates
    // consider - tolerance for from and to
    //	const float TO_TOLERANCE = 15.0;

    RoadNodeContainer container;
    RoadOpenList openList;

    static StaticStorage<RoadNode*> openListStorage;
    // use static storage to contain typical searches
    openList.SetStorage(openListStorage.Init(256));

    RoadNode *best, *cur;

#if DIAGS
    LOG_DEBUG(World, "Searching road path from {:.0f}, {:.0f} to {:.0f}, {:.0f}, precision {:.1f}", from.X(), from.Z(),
              to.X(), to.Z(), prec);
    LOG_DEBUG(World, " from on road: {}", IsOnRoad(from, prec) ? "Yes" : "No");
    LOG_DEBUG(World, " to   on road: {}", IsOnRoad(to, prec) ? "Yes" : "No");
#endif

    // 1. find nodes with distance from to < TO_TOLERANCE and place it to open list
    int xi, xx = toIntFloor(to.X() * InvLandGrid);
    int zi, zz = toIntFloor(to.Z() * InvLandGrid);
#if DIAGS
    int sumItems = 0;
#endif
    for (zi = zz - 1; zi <= zz + 1; zi++)
    {
        for (xi = xx - 1; xi <= xx + 1; xi++)
        {
            if (!InRange(xi, zi))
            {
                continue;
            }
            const RoadList& roadList = _roads(xi, zi);
#if DIAGS
            sumItems += roadList.Size();
#endif
            for (int i = 0; i < roadList.Size(); i++)
            {
                RoadLink* item = roadList[i];
                if (item->IsLocked())
                {
#if DIAGS
                    Object* obj = item->GetObject();
                    LOG_DEBUG(World, "Locked item {} ({:.1f},{:.1f})", (const char*)obj->GetDebugName(),
                              obj->Position().X(), obj->Position().Z());
#endif
                    continue;
                }
                if (item->IsInside(to, prec))
                {
                    best = new RoadNode;
                    container.Add(best);
                    best->parent = nullptr;
                    best->road = item;
                    best->pos = to;                                             // item->PosConnections()[jBest];
                    best->cost = item->GetCenter().Distance(to) * COST_NO_ROAD; // * sqrt(minDist2);
                    best->heur = RoadHeur(best->pos, from);
                    best->open = true;
                    openList.HeapInsert(best);
#if DIAGS >= 2
                    LOG_DEBUG(World, "Item {:.1f},{:.1f} open  from {}, to {}", item->GetCenter().X(),
                              item->GetCenter().Z(), item->IsInside(from, prec) ? "in " : "out",
                              item->IsInside(to, prec) ? "in " : "out");
#endif
                }
            }
        }
    }
#if DIAGS
    LOG_DEBUG(World, "  {} items placed at open list ({} candidates)", openList.Size(), sumItems);
#endif

    // 2. perform A*
    int iter = 0;
    while (true) // search cycle
    {
        bool ok = openList.HeapRemoveFirst(best);
        // A generated way can contain thousands of short segments. Keep the
        // legacy cap for old maps, scale the budget for polyline networks.
        const int maxIterations = _waySegments.empty() ? MAX_ITER
            : std::min(200000, std::max(MAX_ITER, static_cast<int>(_waySegments.size()) * 2));
        if (!ok || ++iter > maxIterations)
        {
            goto PathNotFound;
        }

        if (best->road->IsInside(from, 0))
        {
            // best is the first node of result path
            goto PathFound;
        }

        best->open = false;

        int i, n = best->road->NConnections();
        for (i = 0; i < n; i++) // generate successors
        {
            RoadLink* item = best->road->Connections()[i];
            if (!item)
            {
                continue;
            }
            if (item->IsLocked())
            {
#if DIAGS
                Object* obj = item->GetObject();
                LOG_DEBUG(World, "Locked item {} ({:.1f},{:.1f})", (const char*)obj->GetDebugName(),
                          obj->Position().X(), obj->Position().Z());
#endif
                continue;
            }

            Vector3Val pos = best->road->PosConnections()[i];
            float cost = best->cost + RoadCost(best->pos, pos);

            cur = container.Find(item);
            if (cur)
            {
                //			heuristic doesn't change
                if (cur->open && (cost < cur->cost))
                {
                    cur->parent = best;
                    cur->cost = cost;
                    openList.HeapUpdateUp(cur);
                }
                else
                {
                    // there is better path into item
                    continue;
                }
            }
            else
            {
                cur = new RoadNode;
                container.Add(cur);
                cur->parent = best;
                cur->road = item;
                cur->pos = pos;
                cur->cost = cost;
                cur->heur = RoadHeur(pos, from);
                cur->open = true;
                openList.HeapInsert(cur);
#if DIAGS >= 2
                LOG_DEBUG(World, "Item {:.1f},{:.1f} open  from {}, to {}", item->GetCenter().X(),
                          item->GetCenter().Z(), item->IsInside(from, prec) ? "in " : "out",
                          item->IsInside(to, prec) ? "in " : "out");
#endif
            }
        } // end of generate successors
    } // end of search cycle

// 3. build OperInfoResult list
PathNotFound:
    best = nullptr;
    {
        float minHeur = FLT_MAX;
        for (int i = 0; i < container.Size(); i++)
        {
            cur = container[i];
            if (cur->road->IsInside(from, prec))
            {
                if (cur->heur < minHeur)
                {
                    minHeur = cur->heur;
                    best = cur;
                }
            }
        }
    }

    if (best)
    {
#if DIAGS
        LOG_DEBUG(World, "  alternate start found");
#endif
        goto PathFound;
    }

#if DIAGS
    LOG_DEBUG(World, "  path not found at {} iters", iter);
#endif
    return false;

PathFound:
    // best is the first node of result path
#if DIAGS
    LOG_DEBUG(World, "  path found at {} iters:", iter);
#endif
    path.Clear();
    // check if from should be added to path
    RoadNode* next = best->parent;
    if (!next)
    {
        path.Add(from);
    }
    else
    {
        // check angle with first section of road (from..best..next)
        Vector3 nPos = next->pos;
        Vector3 bPos = best->pos;
        Vector3 nbPos = nPos - bPos;
        float cosAlpha = nbPos * (from - bPos);
        if (cosAlpha < 0)
        {
            path.Add(from);
        }
    }
    cur = best;
    while (cur)
    {
        path.Add(cur->pos);
#if DIAGS
        LOG_DEBUG(World, "  - {:.0f}, {:.0f}", cur->pos.X(), cur->pos.Z());
#endif
        cur = cur->parent;
    }
    return true;
}

const RoadLink* RoadNet::IsOnRoad(Vector3Par pos, float size) const
{
    int xx = toIntFloor(pos.X() * InvLandGrid);
    int zz = toIntFloor(pos.Z() * InvLandGrid);
    for (int zi = zz - 1; zi <= zz + 1; zi++)
    {
        for (int xi = xx - 1; xi <= xx + 1; xi++)
        {
            if (!InRange(xi, zi))
            {
                continue;
            }
            const RoadList& roadList = _roads(xi, zi);
            for (int i = 0; i < roadList.Size(); i++)
            {
                const RoadLink* roadLink = roadList[i];
                if (roadLink->IsInside(pos, size))
                {
                    return roadLink;
                }
            }
        }
    }
    return nullptr;
}

Vector3 RoadNet::GetNearestRoadPoint(Vector3Par pos) const
{
    // scan all roads near pos
    // check 1-1 connections
    float maxDist2 = Square(20);
    Vector3 best = pos;
    int xx = toIntFloor(pos.X() * InvLandGrid);
    int zz = toIntFloor(pos.Z() * InvLandGrid);
    for (int zi = zz - 1; zi <= zz + 1; zi++)
    {
        for (int xi = xx - 1; xi <= xx + 1; xi++)
        {
            if (!InRange(xi, zi))
            {
                continue;
            }
            const RoadList& roadList = _roads(xi, zi);
            for (int i = 0; i < roadList.Size(); i++)
            {
                const RoadLink* roadLink = roadList[i];
                // check distance from main connection (0..1)
                int nConn = roadLink->NConnections();
                const Vector3* pConn = roadLink->PosConnections();
                if (nConn < 2)
                {
                    continue;
                }
                Vector3Val beg = pConn[0];
                Vector3Val e = pConn[1] - beg;
                Vector3Val p = pos - beg;

                float tPrev = (e * p) / e.SquareSize();
                saturate(tPrev, 0, 1);
                Vector3 nearest = beg + tPrev * e;
                float dist2 = nearest.Distance2Inline(pos);
                if (dist2 < maxDist2)
                {
                    maxDist2 = dist2;
                    best = nearest;
                }
            }
        }
    }
    return best;
}

bool RoadNet::IsLocked(Vector3Par pos, float size) const
{
    int xx = toIntFloor(pos.X() * InvLandGrid);
    int zz = toIntFloor(pos.Z() * InvLandGrid);
    for (int zi = zz - 1; zi <= zz + 1; zi++)
    {
        for (int xi = xx - 1; xi <= xx + 1; xi++)
        {
            if (!InRange(xi, zi))
            {
                continue;
            }
            const RoadList& roadList = _roads(xi, zi);

            for (int i = 0; i < roadList.Size(); i++)
            {
                const RoadLink* roadLink = roadList[i];
                if (!roadLink->IsLocked())
                {
                    continue;
                }
                if (roadLink->NearestConnectionDist2(pos) < Square(size))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void RoadNet::RefreshWayHeights(Landscape* land)
{
    if (!land || _waySegments.empty())
        return;

    // The editor can modify terrain while units are present. Rebuilding
    // RoadNet here destroys RoadLink objects that AI still uses. Update only
    // vertical coordinates and normals: the X/Z road graph stays intact.
    for (auto& mesh : _wayMeshes)
    {
        for (int i = 0; i < mesh->NPos(); ++i)
        {
            Vector3& p = mesh->SetPos(i);
            float dX = 0, dZ = 0;
            p[1] = land->SurfaceY(p.X(), p.Z(), &dX, &dZ) + RoadWaySurfaceOffset;
            Vector3 n(-dX, 1, -dZ);
            n.Normalize();
            mesh->SetNorm(i) = n;
        }
        mesh->CalculateMinMax();
        mesh->InvalidateBuffer();
    }

    for (RoadWaySegment& seg : _waySegments)
    {
        seg.from[1] = land->SurfaceY(seg.from.X(), seg.from.Z()) + RoadWaySurfaceOffset;
        seg.to[1] = land->SurfaceY(seg.to.X(), seg.to.Z()) + RoadWaySurfaceOffset;
    }

    // Existing RoadLinks store positions by value, including extra branch
    // junction slots. Keep every pointer stable while updating heights.
    for (int x = 0; x < LandRange; ++x)
        for (int z = 0; z < LandRange; ++z)
        {
            RoadList& list = _roads(x, z);
            for (int i = 0; i < list.Size(); ++i)
            {
                RoadLink* link = list[i];
                if (!link->IsWay())
                    continue;
                // Link endpoints, including added intersection slots.
                link->UpdateWayHeights(land);
            }
        }
}

bool RoadNet::IsBuiltFor(const Landscape* land) const
{
    const char* name = land ? land->GetName() : nullptr;
    return land && _builtForLandscape == land && _builtForName == (name ? name : "");
}

void RoadNet::Build(Landscape* land)
{
    Scan(land);
    LoadWays(land);
    Connect();
    ConnectWayJunctions();
    Optimize();
    Compact();
    _builtForLandscape = land;
    const char* name = land->GetName();
    _builtForName = name ? name : "";
}

SRef<RoadNet> GRoadNet; // global instance

} // namespace Poseidon
