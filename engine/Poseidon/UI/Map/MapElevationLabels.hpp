#pragma once

#include <algorithm>
#include <cstdlib>
#include <utility>
#include <vector>

namespace Poseidon::MapDraw
{
// At an island-wide overview a terrain cell is a fraction of a screen pixel.
// Elevation numbers would be illegible and needlessly expensive to draw.
inline bool ShowElevationLabels(float projectedLandCellPixels)
{
    return projectedLandCellPixels >= 2.0f;
}

// Limit height labels by their *screen* footprint rather than by map-cell count.
// Only a few hundred buckets are allocated, even for a world with millions of
// mountain points. Neighbour checks prevent labels at bucket edges overlapping.
class ElevationLabelGrid
{
    static constexpr int SpacingX = 90;
    static constexpr int SpacingY = 30;

    int _left;
    int _top;
    int _width;
    int _height;
    int _columns;
    int _rows;
    std::vector<int> _slots;
    std::vector<std::pair<int, int>> _positions;

  public:
    ElevationLabelGrid(int left, int top, int width, int height)
        : _left(left), _top(top), _width(std::max(width, 0)), _height(std::max(height, 0)),
          _columns((_width + SpacingX - 1) / SpacingX), _rows((_height + SpacingY - 1) / SpacingY),
          _slots(static_cast<size_t>(_columns) * _rows, -1)
    {
    }

    bool TryAdd(int pixelX, int pixelY)
    {
        if (pixelX < _left || pixelY < _top || pixelX >= _left + _width || pixelY >= _top + _height)
            return false;

        const int col = (pixelX - _left) / SpacingX;
        const int row = (pixelY - _top) / SpacingY;

        for (int y = std::max(0, row - 1); y <= std::min(_rows - 1, row + 1); ++y)
        {
            for (int x = std::max(0, col - 1); x <= std::min(_columns - 1, col + 1); ++x)
            {
                const int index = _slots[static_cast<size_t>(y) * _columns + x];
                if (index < 0)
                    continue;
                const auto& previous = _positions[index];
                if (std::abs(pixelX - previous.first) < SpacingX &&
                    std::abs(pixelY - previous.second) < SpacingY)
                    return false;
            }
        }

        _slots[static_cast<size_t>(row) * _columns + col] = static_cast<int>(_positions.size());
        _positions.emplace_back(pixelX, pixelY);
        return true;
    }
};
} // namespace Poseidon::MapDraw
