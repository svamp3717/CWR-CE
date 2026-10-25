
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

// Use the donor's original textures instead of guessing a road texture
// from its file name. Visual LOD supplies pixels/UV appearance; roadway
// LOD supplies the contact material (roughness, dust and sound environment).
Texture* FirstFaceTexture(Shape* shape)
{
    if (!shape)
        return nullptr;
    for (Offset face = shape->BeginFaces(); face < shape->EndFaces(); shape->NextFace(face))
        if (Texture* texture = shape->Face(face).GetTexture())
            return texture;
    return nullptr;
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
        if (IsDonorName(way.texture))
        {
            const std::string donorPath = way.texture.compare(0, 6, "donor=") == 0
                ? way.texture.substr(6) : way.texture;
            Ref<LODShapeWithShadow> donor = Shapes.New(donorPath.c_str(), false, true);
            if (donor)
            {
                texture = FirstFaceTexture(donor->LevelOpaque(0));
                surfaceTexture = FirstFaceTexture(donor->RoadwayLevel());
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

        auto mesh = std::make_unique<Shape>();
        mesh->Reserve(static_cast<int>(points.size() * 2));
        mesh->ReserveFaces(static_cast<int>((points.size() - 1) * 2));
        mesh->RegisterTexture(texture, 1.0f);

        Poly poly;
        poly.Init();
        poly.SetTexture(texture);
        poly.SetSpecial(NoClamp | NoZWrite);
        float distance = 0.f;
        for (size_t i = 0; i < points.size(); ++i)
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
            Vector3 center = points[i];
            center[1] = land->SurfaceY(center.X(), center.Z()) + RoadWaySurfaceOffset;
            if (i)
                distance += std::sqrt((points[i].X() - points[i-1].X()) * (points[i].X() - points[i-1].X())
                    + (points[i].Z() - points[i-1].Z()) * (points[i].Z() - points[i-1].Z()));
            Vector3 left = center + perp * halfWidth;
            Vector3 right = center - perp * halfWidth;
            // Conform both edges independently on sloped terrain. Sampling
            // only the centre would leave wide roads floating or buried.
            float leftDX = 0, leftDZ = 0, rightDX = 0, rightDZ = 0;
            left[1] = land->SurfaceY(left.X(), left.Z(), &leftDX, &leftDZ) + RoadWaySurfaceOffset;
            right[1] = land->SurfaceY(right.X(), right.Z(), &rightDX, &rightDZ) + RoadWaySurfaceOffset;
            Vector3 leftNormal(-leftDX, 1, -leftDZ);
            Vector3 rightNormal(-rightDX, 1, -rightDZ);
            leftNormal.Normalize();
            rightNormal.Normalize();
            const float v = texture->VToPhysical(distance / way.width);
            mesh->AddVertexFast(left, leftNormal, ClipAll, texture->UToPhysical(0), v);
            mesh->AddVertexFast(right, rightNormal, ClipAll, texture->UToPhysical(1), v);
            if (i)
            {
                const int prev = static_cast<int>((i - 1) * 2);
                const int next = static_cast<int>(i * 2);
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

void RoadNet::Build(Landscape* land)
{
    Scan(land);
    LoadWays(land);
    Connect();
    ConnectWayJunctions();
    Optimize();
    Compact();
}

SRef<RoadNet> GRoadNet; // global instance

} // namespace Poseidon
