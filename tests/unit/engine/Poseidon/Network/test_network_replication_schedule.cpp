#include <catch2/catch_test_macros.hpp>
#include <Poseidon/Network/NetworkReplicationSchedule.hpp>
#include <cstdint>
#include <limits>

TEST_CASE("Server routine replication runs at a fixed 20 Hz", "[network][replication]")
{
    Poseidon::ReplicationCadence cadence;
    REQUIRE(cadence.ShouldRun(1000));
    REQUIRE(cadence.LastElapsedMs() == 50);
    REQUIRE_FALSE(cadence.ShouldRun(1049));
    REQUIRE(cadence.ShouldRun(1050));
    REQUIRE(cadence.LastElapsedMs() == 50);
    REQUIRE_FALSE(cadence.ShouldRun(1051));
    REQUIRE(cadence.ShouldRun(1101));
    REQUIRE(cadence.LastElapsedMs() == 51);
}

TEST_CASE("Replication cadence handles tick rollover and hitches", "[network][replication]")
{
    Poseidon::ReplicationCadence cadence;
    constexpr auto start = std::numeric_limits<std::uint32_t>::max() - 20;
    REQUIRE(cadence.ShouldRun(start));
    REQUIRE_FALSE(cadence.ShouldRun(10));
    REQUIRE(cadence.ShouldRun(29));
    REQUIRE(cadence.LastElapsedMs() == 50);
    REQUIRE(cadence.ShouldRun(1000));
    REQUIRE(cadence.LastElapsedMs() == Poseidon::ReplicationCadence::MaxBudgetMs);
    cadence.Reset();
    REQUIRE(cadence.ShouldRun(1001));
    REQUIRE(cadence.LastElapsedMs() == Poseidon::ReplicationCadence::PeriodMs);
}
