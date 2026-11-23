#pragma once

#include <algorithm>
#include <cstdint>

namespace Poseidon
{
// The network transport itself remains asynchronous. This controls only
// ordinary server state-update preparation; queued reliable messages remain
// serviced on every frame.
class ReplicationCadence
{
  public:
    static constexpr std::uint32_t PeriodMs = 50; // 20 Hz
    static constexpr std::uint32_t MaxBudgetMs = 200;

    bool ShouldRun(std::uint32_t now)
    {
        if (!_started)
        {
            _started = true;
            _lastTick = now;
            _lastElapsed = PeriodMs;
            return true;
        }
        const std::uint32_t elapsed = now - _lastTick; // wrap-safe
        if (elapsed < PeriodMs)
            return false;

        _lastTick = now;
        _lastElapsed = std::min(elapsed, MaxBudgetMs);
        return true;
    }

    std::uint32_t LastElapsedMs() const { return _lastElapsed; }
    void Reset()
    {
        _started = false;
        _lastTick = 0;
        _lastElapsed = PeriodMs;
    }

  private:
    bool _started = false;
    std::uint32_t _lastTick = 0;
    std::uint32_t _lastElapsed = PeriodMs;
};
} // namespace Poseidon
