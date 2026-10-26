#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <Poseidon/World/Terrain/Roads.hpp>
#include <Poseidon/Graphics/Dummy/TextureDummy.hpp>

using namespace Poseidon;

TEST_CASE("Generated road links use width and terrain-independent proximity", "[World][RoadWays]")
{
    // No P3D shape, roadway LOD or Object is constructed.
    RoadLink road(Vector3(10, 20, 15), Vector3(110, 25, 15), 8.f, nullptr);

    REQUIRE(road.IsWay());
    REQUIRE(road.NConnections() == 2);
    REQUIRE(road.IsInside(Vector3(35, 100, 18), 0.f));
    REQUIRE(road.IsInside(Vector3(35, 0, 21), 2.f));
    REQUIRE_FALSE(road.IsInside(Vector3(35, 20, 21), 0.f));
    REQUIRE_FALSE(road.IsInside(Vector3(130, 20, 15), 0.f));
    REQUIRE(road.GetCenter().X() == Catch::Approx(60.f));
    REQUIRE_FALSE(road.IsLocked());
    road.Lock();
    REQUIRE(road.IsLocked());
    road.Unlock();
    REQUIRE_FALSE(road.IsLocked());
}

TEST_CASE("Generated road links accept multiway junction connections", "[World][RoadWays]")
{
    RoadLink a(Vector3(0, 0, 0), Vector3(20, 0, 0), 6.f, nullptr);
    RoadLink b(Vector3(20, 0, 0), Vector3(20, 0, 20), 6.f, nullptr);
    RoadLink c(Vector3(20, 0, 0), Vector3(20, 0, -20), 6.f, nullptr);

    a.SetConnection(1, &b);
    b.SetConnection(0, &a);
    a.AddConnection(Vector3(20, 0, 0), &c);
    c.AddConnection(Vector3(20, 0, 0), &a);

    REQUIRE(a.NConnections() == 3);
    REQUIRE(a.Connections()[1] == &b);
    REQUIRE(a.Connections()[2] == &c);
    REQUIRE(a.PosConnections()[2].X() == Catch::Approx(20.f));
}

TEST_CASE("Way road contacts retain the donor roadway material", "[World][RoadWays][Surface]")
{
    TextureDummy donorSurface;
    donorSurface.SetRoughness(0.02f);
    donorSurface.SetDustness(0.0f);

    RoadLink road(Vector3(5, 0, 5), Vector3(45, 0, 5), 6.f, &donorSurface);
    REQUIRE(road.WayTexture() == &donorSurface);
    REQUIRE(road.WayTexture()->Roughness() == Catch::Approx(0.02f));
    REQUIRE(road.WayTexture()->Dustness() == Catch::Approx(0.0f));

    // This is an X/Z road footprint query; even a deep land-contact point
    // gets the same roadway texture instead of the terrain's grass texture.
    REQUIRE(road.IsInside(Vector3(20, -2, 6), 0.f));
    REQUIRE_FALSE(road.IsInside(Vector3(20, -2, 10), 0.f));
}

TEST_CASE("Way route strategic sampling keeps actual road bends", "[World][RoadWays][AI]")
{
    RoadPathArray route;
    route.Add(Vector3(0, 0, 0));
    route.Add(Vector3(10, 0, 0));
    route.Add(Vector3(20, 0, 0));
    route.Add(Vector3(20, 0, 10));
    route.Add(Vector3(20, 0, 20));
    route.Add(Vector3(20, 0, 30));

    std::vector<Vector3> goals;
    RoadNet::SampleWayRoute(route, goals);
    REQUIRE(goals.size() >= 3);
    REQUIRE(goals.front().X() == Catch::Approx(0));
    REQUIRE(goals.back().X() == Catch::Approx(20));
    REQUIRE(goals.back().Z() == Catch::Approx(30));

    bool retainedTurn = false;
    for (const Vector3& point : goals)
        if (point.X() == Catch::Approx(20) && point.Z() == Catch::Approx(0))
            retainedTurn = true;
    REQUIRE(retainedTurn); // not a straight line from start to destination
}

TEST_CASE("Way route sampling reduces links on a straight road", "[World][RoadWays][AI]")
{
    RoadPathArray route;
    for (int i = 0; i <= 100; ++i)
        route.Add(Vector3(i * 2.f, 0, 0));

    std::vector<Vector3> goals;
    RoadNet::SampleWayRoute(route, goals);
    REQUIRE(goals.size() < static_cast<size_t>(route.Size()));
    REQUIRE(goals.size() > 5);
    REQUIRE(goals.front().X() == Catch::Approx(0));
    REQUIRE(goals.back().X() == Catch::Approx(200));
}

TEST_CASE("Six-metre operative road sampling follows a gradual bend", "[World][RoadWays][AI]")
{
    // A quarter-circle has no single sharp (>45 degree) vertex, so a
    // corner-only algorithm would incorrectly shortcut across the grass.
    RoadPathArray route;
    constexpr float radius = 20.f;
    for (int i = 0; i <= 40; ++i)
    {
        const float angle = 1.570796327f * static_cast<float>(i) / 40.f;
        route.Add(Vector3(radius * std::cos(angle), 0, radius * std::sin(angle)));
    }
    std::vector<Vector3> goals;
    RoadNet::SampleWayRoute(route, goals, 6.f);
    REQUIRE(goals.size() >= 6);

    for (size_t i = 1; i < goals.size(); ++i)
    {
        Vector3 midpoint = (goals[i - 1] + goals[i]) * 0.5f;
        // A start-to-end chord would dip to 14.14m from the circle centre;
        // six-metre segments remain on the 20m-radius road.
        const float distance = std::sqrt(midpoint.X() * midpoint.X() +
                                         midpoint.Z() * midpoint.Z());
        REQUIRE(distance > 19.f);
    }
}

TEST_CASE("Procedural ways preserve original P3D alpha ordered clamp material", "[World][RoadWays][Rendering]")
{
    // wg_osmways4 road-donors.json describes the donor's original road
    // polygon flag 0x00024102. Copying only the texture discarded these
    // flags, changing colors even with the correct donor P3D.
    constexpr int original = 0x00024102;
    const int flags = RoadNet::WayFaceFlags(original, true, true);
    REQUIRE(flags == (original | NoZWrite));
    REQUIRE((flags & OnSurface) != 0);
    REQUIRE((flags & IsAlpha) != 0);
    REQUIRE((flags & IsAlphaOrdered) != 0);
    REQUIRE((flags & ClampU) != 0);
    REQUIRE((flags & NoClamp) == 0);
    REQUIRE((flags & NoZWrite) != 0);

    // Texture-only .ways entries still use their legacy fallback.
    REQUIRE(RoadNet::WayFaceFlags(0, false, true) == (NoClamp | NoZWrite | IsAlpha));
    REQUIRE(RoadNet::WayFaceFlags(0, false, false) == (NoClamp | NoZWrite));
}

TEST_CASE("Donor road faces cannot inherit hidden or water rendering", "[World][RoadWays][Rendering]")
{
    const int flags = RoadNet::WayFaceFlags(ClampU | OnSurface | IsHidden | IsWater, true, false);
    REQUIRE((flags & ClampU) != 0);
    REQUIRE((flags & OnSurface) != 0);
    REQUIRE((flags & IsHidden) == 0);
    REQUIRE((flags & IsWater) == 0);
}

TEST_CASE("Generated road faces preserve renderer clockwise front winding", "[World][RoadWays][Rendering]")
{
    // The graphics pipeline uses FrontFaceMode::CW, not the sign of
    // Poly::GetNormal (which calculates p2 cross p1). Reversing these
    // indices made every way invisible due to back-face culling.
    Shape mesh;
    mesh.Reserve(4);
    mesh.ReserveFaces(2);
    const Vector3 normal(0, 1, 0);
    mesh.AddVertexFast(Vector3(0, 0, 0), normal, ClipAll, 0, 0);
    mesh.AddVertexFast(Vector3(2, 0, 0), normal, ClipAll, 1, 0);
    mesh.AddVertexFast(Vector3(0, 0, 2), normal, ClipAll, 0, 1);
    mesh.AddVertexFast(Vector3(2, 0, 2), normal, ClipAll, 1, 1);

    Poly face;
    face.Init();
    RoadNet::AddWayQuad(mesh, face, 0, 2);
    REQUIRE(mesh.NFaces() == 2);
    REQUIRE(mesh.FaceIndexed(0).GetVertex(0) == 0);
    REQUIRE(mesh.FaceIndexed(0).GetVertex(1) == 2);
    REQUIRE(mesh.FaceIndexed(0).GetVertex(2) == 1);
    REQUIRE(mesh.FaceIndexed(1).GetVertex(0) == 1);
    REQUIRE(mesh.FaceIndexed(1).GetVertex(1) == 2);
    REQUIRE(mesh.FaceIndexed(1).GetVertex(2) == 3);

    for (int i = 0; i < mesh.NFaces(); ++i)
    {
        const Poly& triangle = mesh.FaceIndexed(i);
        // Poly::GetNormal has the reverse convention to the render frontend,
        // so this negative result is expected for correctly visible faces.
        REQUIRE(triangle.GetNormal(mesh).Y() < 0.f);
        // Lighting still uses deliberately upward-facing per-vertex normals.
        for (int v = 0; v < triangle.N(); ++v)
            REQUIRE(mesh.Norm(triangle.GetVertex(v)).Y() > 0.f);
    }
}
