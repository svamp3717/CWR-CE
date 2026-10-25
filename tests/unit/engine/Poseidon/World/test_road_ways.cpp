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
