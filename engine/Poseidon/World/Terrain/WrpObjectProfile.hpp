#pragma once

#include <chrono>
#include <cstdint>

namespace Poseidon
{
// Samples one out of every 128 WRP placements. Measuring every object would
// distort timings on multi-million-object worlds. Estimates are approximate,
// not a replacement for the existing wall-clock WRP load measurements.
struct WrpObjectProfile
{
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    static constexpr int SamplePeriod = 128;

    enum Stage
    {
        Factory,
        IDCache,
        MatrixRepair,
        SetTransform,
        GroundPosition,
        SurfaceHeight,
        CellInsertion,
        OprwIDConflicts,
        StageCount
    };
    enum Kind
    {
        Forest,
        Road,
        StreetLamp,
        Building,
        Other,
        KindCount
    };

    uint64_t nanoseconds[StageCount] = {};
    uint64_t factoryNanoseconds[KindCount] = {};
    uint64_t factorySamples[KindCount] = {};
    uint64_t samples = 0;

    void Record(Stage stage, TimePoint start, TimePoint end)
    {
        nanoseconds[stage] += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
        if (stage == Factory)
            ++samples;
    }

    void RecordFactoryKind(Kind kind, TimePoint start, TimePoint end)
    {
        factoryNanoseconds[kind] += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
        ++factorySamples[kind];
    }

    double EstimatedMs(Stage stage, int totalObjects) const
    {
        return samples ? (static_cast<double>(nanoseconds[stage]) / samples) * totalObjects / 1000000.0 : 0.0;
    }

    double AverageFactoryUs(Kind kind) const
    {
        return factorySamples[kind]
            ? static_cast<double>(factoryNanoseconds[kind]) / factorySamples[kind] / 1000.0
            : 0.0;
    }
};
} // namespace Poseidon
