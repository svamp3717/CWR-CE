#include <catch2/catch_test_macros.hpp>
#include <Poseidon/UI/Map/UIMap.hpp>
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
