#include <catch2/catch_test_macros.hpp>
#include <Poseidon/World/Terrain/Roads.hpp>

using namespace Poseidon;

TEST_CASE("Road junction grows beyond four connections without losing links", "[World][Roads]")
{
    Vector3 points[4] = {
        Vector3(0, 0, 0),
        Vector3(1, 0, 0),
        Vector3(2, 0, 0),
        Vector3(3, 0, 0),
    };

    RoadLink junction(nullptr, points, 4);
    RoadLink neighbour(nullptr, points, 1);

    junction.SetConnection(0, &neighbour);

    // An extra road joining an already occupied endpoint used to hit the
    // fixed four-slot limit, losing the reverse connection.
    for (int i = 4; i < 20; ++i)
    {
        junction.AddConnection(Vector3(static_cast<float>(i), 0, 0), &neighbour);
    }

    REQUIRE(junction.NConnections() == 20);
    CHECK(junction.Connections()[0] == &neighbour);
    CHECK(junction.PosConnections()[0].X() == 0.0f);
    for (int i = 4; i < junction.NConnections(); ++i)
    {
        CHECK(junction.Connections()[i] == &neighbour);
        CHECK(junction.PosConnections()[i].X() == static_cast<float>(i));
    }

    // Existing inline slots must remain writable after expansion.
    junction.SetConnection(1, &neighbour);
    CHECK(junction.Connections()[1] == &neighbour);
    CHECK(junction.PosConnections()[1].X() == 1.0f);

    // Geometry queries must see every expanded position, not just the four
    // original inline slots (otherwise they read past the inline array).
    CHECK(junction.GetCenter().X() == 9.5f);
    CHECK(junction.NearestConnectionDist2(Vector3(19, 0, 0)) == 0.0f);
}

TEST_CASE("Ordinary road segments stay within inline connection storage", "[World][Roads]")
{
    Vector3 points[2] = {Vector3(0, 0, 0), Vector3(1, 0, 0)};
    RoadLink road(nullptr, points, 2);
    CHECK(road.NConnections() == 2);
    CHECK(road.Connections()[0] == nullptr);

    road.AddConnection(Vector3(2, 0, 0), nullptr);
    road.AddConnection(Vector3(3, 0, 0), nullptr);

    REQUIRE(road.NConnections() == 4);
    CHECK(road.PosConnections()[2].X() == 2.0f);
    CHECK(road.PosConnections()[3].X() == 3.0f);
}
