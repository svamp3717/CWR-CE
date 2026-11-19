#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace Poseidon
{
// Intrusive heap index removes HeapArray::Find (linear in the open list)
// from A* decrease-key. Comparisons/tie handling match the legacy heap.
template<class Node>
class RoadIndexedHeap
{
    std::vector<Node*> _items;

    void Swap(std::size_t i, std::size_t j)
    {
        std::swap(_items[i], _items[j]);
        _items[i]->heapIndex = i;
        _items[j]->heapIndex = j;
    }

    void Up(std::size_t i)
    {
        while (i)
        {
            const auto parent = (i - 1) / 2;
            if (*_items[parent] <= *_items[i])
                break;
            Swap(parent, i);
            i = parent;
        }
    }

    void Down(std::size_t i)
    {
        while (2 * i + 1 < _items.size())
        {
            auto child = 2 * i + 1;
            if (child + 1 < _items.size() && *_items[child+1] < *_items[child])
                ++child;
            if (*_items[i] <= *_items[child])
                break;
            Swap(i, child);
            i = child;
        }
    }

public:
    RoadIndexedHeap() { _items.reserve(256); }
    std::size_t Size() const { return _items.size(); }

    void HeapInsert(Node* node)
    {
        node->heapIndex = _items.size();
        _items.push_back(node);
        Up(node->heapIndex);
    }

    bool HeapRemoveFirst(Node*& node)
    {
        if (_items.empty())
            return false;
        node = _items.front();
        Swap(0, _items.size() - 1);
        _items.pop_back();
        node->heapIndex = static_cast<std::size_t>(-1);
        if (!_items.empty())
            Down(0);
        return true;
    }

    void HeapUpdateUp(Node* node)
    {
        const auto i = node->heapIndex;
        if (i < _items.size() && _items[i] == node)
            Up(i);
    }
};
} // namespace Poseidon
