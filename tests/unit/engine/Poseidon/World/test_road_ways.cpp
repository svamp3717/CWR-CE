#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <Poseidon/World/Terrain/Roads.hpp>

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
