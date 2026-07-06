// Accumulated each frame, read+reset by benchmark code
#pragma once

#include <cstdint>

#ifdef _WIN32
#include <intrin.h>
#pragma intrinsic(__rdtsc)
#endif


namespace Poseidon
{
struct TerrainProfile {
    int segmentsDrawn;
    int segmentsCacheHit;
    int segmentsCacheMiss;
    int drawMeshCalls;
    int drawMeshClipped;
    int cacheSearchSteps;
    double drawGroundCycles;
    double generateSegCycles;
    // Scene::DrawObjectsAndShadowsPass1 breakdown, feeds the lnd:obj investigation.
    double pass1TotalCycles;
    double pass1CompactCycles;
    double pass1ComplexityCycles;
    double pass1BuildMergersCycles;
    double pass1OcclusionCycles;
    double pass1SortCycles;
    double pass1DrawCycles;
    double pass1DrawScalarCycles;
    double pass1DrawInstancedCycles;

    int pass1Objects;
    int pass1Mergers;
    int pass1ScalarObjects;
    int pass1InstancedRuns;
    int pass1InstancedObjects;

    // Instancing diagnostics. These explain why the lnd:obj path falls back to scalar drawing.
    int pass1BatchCandidateRuns;
    int pass1BatchAcceptedRuns;
    int pass1BatchAcceptedObjects;
    int pass1BatchRejectUnderThreshold;
    int pass1BatchRejectHeadNotBatchable;
    int pass1BatchRejectLocalLights;
    int pass1BatchRejectNotStatic;
    int pass1BatchRejectProxy;
    int pass1BatchRejectOnSurface;
    int pass1BatchRejectColored;
    int pass1BatchRejectCamera;
    int pass1BatchBreakShapeOrLodMismatch;
    int pass1BatchBreakPassMismatch;
    int pass1BatchBreakNotStatic;
    int pass1BatchBreakSpecialMismatch;
    int pass1BatchBreakDistanceBand;
    int pass1BatchBreakEngineLimit;
    int pass1BatchEndFailed;


    void Reset() { *this = {}; }

    static int64_t Now() {
#ifdef _WIN32
        return static_cast<int64_t>(__rdtsc());
#elif defined(__x86_64__) || defined(__i386__)
        unsigned int lo, hi;
        __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
        return (static_cast<int64_t>(hi) << 32) | lo;
#elif defined(__aarch64__)
        int64_t val;
        __asm__ __volatile__("mrs %0, cntvct_el0" : "=r"(val));
        return val;
#else
        return 0;
#endif
    }
};

extern TerrainProfile GTerrainProfile;

}  // namespace Poseidon
