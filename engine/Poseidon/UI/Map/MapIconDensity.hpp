#pragma once

namespace Poseidon::MapDraw
{
// Number of decorative symbols to sample from a terrain cell. A fixed stride
// keeps the same subset as the map pans, with no per-frame allocations.
inline int DecorativeIconStride(int cellObjectCount, int maxSymbolsPerCell)
{
    if (cellObjectCount <= 0 || maxSymbolsPerCell <= 0 || cellObjectCount <= maxSymbolsPerCell)
        return 1;
    return 1 + (cellObjectCount - 1) / maxSymbolsPerCell;
}

// Bound icon density to projected land-cell area. At high zoom the editor can
// show more detail; zooming out won't issue thousands of overlapping draw calls.
inline int DecorativeIconBudget(float projectedCellPoints)
{
    if (!(projectedCellPoints > 0.0f))
        return 1;

    const float icons = (projectedCellPoints * projectedCellPoints) / (12.0f * 12.0f);
    if (icons < 1.0f)
        return 1;
    if (icons >= 128.0f)
        return 128;
    return static_cast<int>(icons);
}
} // namespace Poseidon::MapDraw
