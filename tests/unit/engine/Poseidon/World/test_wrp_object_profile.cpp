#include <catch2/catch_test_macros.hpp>
#include <Poseidon/World/Terrain/WrpObjectProfile.hpp>

using namespace Poseidon;

TEST_CASE("WRP object profile extrapolates sampled stage times", "[World][Terrain][WRP][Performance]")
{
    WrpObjectProfile profile;
    const auto start = WrpObjectProfile::Clock::time_point{};
    const auto factoryEnd = start + std::chrono::microseconds(10);
    const auto idEnd = factoryEnd + std::chrono::microseconds(5);

    profile.Record(WrpObjectProfile::Factory, start, factoryEnd);
    profile.RecordFactoryKind(WrpObjectProfile::Forest, start, factoryEnd);
    profile.Record(WrpObjectProfile::IDCache, factoryEnd, idEnd);

    REQUIRE(profile.samples == 1);
    CHECK(profile.factorySamples[WrpObjectProfile::Forest] == 1);
    CHECK(profile.AverageFactoryUs(WrpObjectProfile::Forest) == 10.0);
    CHECK(profile.AverageFactoryUs(WrpObjectProfile::Road) == 0.0);
    CHECK(profile.EstimatedMs(WrpObjectProfile::Factory, 1000) == 10.0);
    CHECK(profile.EstimatedMs(WrpObjectProfile::IDCache, 1000) == 5.0);
    CHECK(profile.EstimatedMs(WrpObjectProfile::SurfaceHeight, 1000) == 0.0);
}

TEST_CASE("WRP object profile safely handles an empty sample", "[World][Terrain][WRP][Performance]")
{
    WrpObjectProfile profile;
    CHECK(profile.samples == 0);
    CHECK(profile.EstimatedMs(WrpObjectProfile::Factory, 5000000) == 0.0);
    CHECK(profile.AverageFactoryUs(WrpObjectProfile::Other) == 0.0);
}
