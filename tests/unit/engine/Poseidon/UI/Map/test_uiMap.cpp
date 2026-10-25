#include <catch2/catch_test_macros.hpp>
#include <Poseidon/UI/Map/UIMap.hpp>
#include <Poseidon/UI/Map/MapElevationLabels.hpp>
#include <Poseidon/UI/Map/MapIconDensity.hpp>

TEST_CASE("uiMap header compiles", "[UI][Map]")
{
    SUCCEED();
}

TEST_CASE("Editor map decorative icon density grows with zoom", "[UI][Map][Performance]")
{
    using namespace Poseidon::MapDraw;
    REQUIRE(DecorativeIconBudget(0.0f) == 1);
    REQUIRE(DecorativeIconBudget(12.0f) == 1);
    REQUIRE(DecorativeIconBudget(24.0f) == 4);
    REQUIRE(DecorativeIconBudget(120.0f) == 100);
    REQUIRE(DecorativeIconBudget(400.0f) == 128);
}

TEST_CASE("Editor map decor sampling is stable and bounded for crowded cells", "[UI][Map][Performance]")
{
    using namespace Poseidon::MapDraw;
    CHECK(DecorativeIconStride(0, 4) == 1);
    CHECK(DecorativeIconStride(3, 4) == 1);
    CHECK(DecorativeIconStride(24, 4) == 6);
    CHECK(DecorativeIconStride(600, 6) == 100);
    // Sampling follows a fixed ordinal, not the frame number. A 600-object
    // cell produces only six decorative icons at this zoom.
    int count = 0;
    const int stride = DecorativeIconStride(600, 6);
    for (int i = 0; i < 600; ++i)
    {
        if ((i % stride) == 0)
            ++count;
    }
    CHECK(count == 6);
}

TEST_CASE("Map elevation labels hide at island overview zoom", "[UI][Map][Performance]")
{
    using namespace Poseidon::MapDraw;
    CHECK_FALSE(ShowElevationLabels(0.1f));
    CHECK_FALSE(ShowElevationLabels(1.99f));
    CHECK(ShowElevationLabels(2.0f));
    CHECK(ShowElevationLabels(10.0f));
}

TEST_CASE("Map elevation labels do not overlap across screen cell boundaries", "[UI][Map][Performance]")
{
    Poseidon::MapDraw::ElevationLabelGrid grid(10, 20, 320, 150);

    // outside viewport
    CHECK_FALSE(grid.TryAdd(9, 25));
    CHECK_FALSE(grid.TryAdd(15, 19));
    CHECK(grid.TryAdd(98, 55));
    // adjacent bucket, same screen footprint
    CHECK_FALSE(grid.TryAdd(100, 56));
    // close horizontally and vertically
    CHECK_FALSE(grid.TryAdd(175, 60));
    // far enough to read
    CHECK(grid.TryAdd(190, 60));
    // separate row
    CHECK(grid.TryAdd(101, 90));
    // outside viewport
    CHECK_FALSE(grid.TryAdd(330, 170));
}

TEST_CASE("Map elevation label budget is bounded by viewport size", "[UI][Map][Performance]")
{
    Poseidon::MapDraw::ElevationLabelGrid grid(0, 0, 800, 600);
    int count = 0;
    for (int y = 0; y < 600; ++y)
    {
        for (int x = 0; x < 800; ++x)
        {
            if (grid.TryAdd(x, y))
                ++count;
        }
    }
    CHECK(count <= 200);
    CHECK(count > 0);
}
