#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

namespace Poseidon
{
// Static X/Z bounding volume hierarchy for immutable road geometry.
// IDs are input positions; ordered queries preserve legacy draw/material order.
struct RoadWayBounds
{
    float minX, minZ, maxX, maxZ;

    bool Intersects(const RoadWayBounds& b) const
    {
        return minX <= b.maxX && maxX >= b.minX &&
               minZ <= b.maxZ && maxZ >= b.minZ;
    }

    void Include(const RoadWayBounds& b)
    {
        minX = std::min(minX, b.minX);
        minZ = std::min(minZ, b.minZ);
        maxX = std::max(maxX, b.maxX);
        maxZ = std::max(maxZ, b.maxZ);
    }
};

class RoadWaySpatialIndex
{
public:
    static constexpr std::size_t NotFound = std::numeric_limits<std::size_t>::max();

    void Clear()
    {
        _entries.clear();
        _nodes.clear();
    }

    void Build(const std::vector<RoadWayBounds>& bounds)
    {
        Clear();
        _entries.reserve(bounds.size());
        for (std::size_t i = 0; i < bounds.size(); ++i)
            _entries.push_back({bounds[i], i});
        if (!_entries.empty())
            BuildNode(0, _entries.size());
    }

    std::vector<std::size_t> Query(const RoadWayBounds& bounds) const
    {
        std::vector<std::size_t> result;
        if (!_nodes.empty())
            QueryNode(0, bounds, result);
        if (result.size() == _entries.size())
            std::iota(result.begin(), result.end(), std::size_t{0});
        else
            std::sort(result.begin(), result.end());
        return result;
    }

    // Allocation-free selection of the earliest matching entry. Match is
    // evaluated only for intersecting entry bounds, in no particular order.
    template<class Match>
    std::size_t FindFirst(const RoadWayBounds& bounds, Match&& match) const
    {
        std::size_t best = NotFound;
        if (!_nodes.empty())
            FindNode(0, bounds, match, best);
        return best;
    }

private:
    struct Entry { RoadWayBounds bounds; std::size_t id; };
    struct Node
    {
        RoadWayBounds bounds;
        std::size_t begin, end, left, right, firstId;
    };
    std::vector<Entry> _entries;
    std::vector<Node> _nodes;

    std::size_t BuildNode(std::size_t begin, std::size_t end)
    {
        auto bounds = _entries[begin].bounds;
        std::size_t firstId = _entries[begin].id;
        for (std::size_t i = begin + 1; i < end; ++i)
        {
            bounds.Include(_entries[i].bounds);
            firstId = std::min(firstId, _entries[i].id);
        }
        const std::size_t id = _nodes.size();
        _nodes.push_back({bounds, begin, end, NotFound, NotFound, firstId});
        if (end - begin > 8)
        {
            const bool splitX = bounds.maxX - bounds.minX >= bounds.maxZ - bounds.minZ;
            const std::size_t mid = begin + (end - begin) / 2;
            std::nth_element(_entries.begin() + begin, _entries.begin() + mid,
                _entries.begin() + end, [splitX](const Entry& a, const Entry& b)
                {
                    const float ca = splitX ? a.bounds.minX + a.bounds.maxX
                                            : a.bounds.minZ + a.bounds.maxZ;
                    const float cb = splitX ? b.bounds.minX + b.bounds.maxX
                                            : b.bounds.minZ + b.bounds.maxZ;
                    return ca == cb ? a.id < b.id : ca < cb;
                });
            const auto left = BuildNode(begin, mid);
            const auto right = BuildNode(mid, end);
            _nodes[id].left = left;
            _nodes[id].right = right;
        }
        return id;
    }

    void QueryNode(std::size_t id, const RoadWayBounds& bounds,
                   std::vector<std::size_t>& result) const
    {
        const auto& node = _nodes[id];
        if (!node.bounds.Intersects(bounds))
            return;
        if (node.left == NotFound)
        {
            for (auto i = node.begin; i < node.end; ++i)
                if (_entries[i].bounds.Intersects(bounds))
                    result.push_back(_entries[i].id);
            return;
        }
        QueryNode(node.left, bounds, result);
        QueryNode(node.right, bounds, result);
    }

    template<class Match>
    void FindNode(std::size_t id, const RoadWayBounds& bounds,
                  Match& match, std::size_t& best) const
    {
        const auto& node = _nodes[id];
        if (node.firstId >= best || !node.bounds.Intersects(bounds))
            return;
        if (node.left == NotFound)
        {
            for (auto i = node.begin; i < node.end; ++i)
            {
                const auto& entry = _entries[i];
                if (entry.id < best && entry.bounds.Intersects(bounds) && match(entry.id))
                    best = entry.id;
            }
            return;
        }
        auto left = node.left, right = node.right;
        if (_nodes[right].firstId < _nodes[left].firstId)
            std::swap(left, right);
        FindNode(left, bounds, match, best);
        FindNode(right, bounds, match, best);
    }
};
} // namespace Poseidon
