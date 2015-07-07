/// @brief  Dijkstra's algorithm implementation
/// @author z.skowron

#ifndef Dijkstra_H
#define Dijkstra_H

// Pseudo kod algorytmu Dijkstry:
//
// Q = 0    - priority queue of nodes, sorted by shortest distance from start we have found so far
// Q += start node
// foreach node n from nodes Q ordered by distance, increasing:
//    if (n == end)
//       return done (and compute path using prev)
//    foreach edge (n, v, c) from neighbours of n:
//       if (dist(n) + c < dist(v))
//         dist(v) = dist(n) + c
//         v.prev = u
//         Q += v
// return path not found
//

#include <vector>
#include <queue>
#include <functional>
#include <algorithm>

#include "ZAssert.h"
#include "NeighbourListGraph.h"

/// @brief Represents distance from given node s and previous node on a shortest path from node s.
template<typename D = int>
struct NodeDistPrev
{
    NodeDistPrev()
        : dist(std::numeric_limits<D>::max())
        , prev(-1)
    {}

    NodeDistPrev(D dist, int prev)
        : dist(dist)
        , prev(prev)
    {}

    D dist;     // distance from start to this node, or std::numeric_limits<D>::max() if path doesn't exist
    int prev;   // which node is before this on the shortest path (or -1 if it doesn't exist)
};

/// @brief Returns path length or -1 if path was not found.
/// @param distPrev NodeDistPrev structure for every node.
/// @param path     Set to shortest path from start to end.
template<typename D = int>
D reconstructPath(const std::vector< NodeDistPrev<D> >& distPrev, int start, int end, std::vector<int>& path)
{
    path.clear();
    path.push_back(end);

    while (path.back() != start)
    {
        assert(path.back() < distPrev.size());  // we must have entries for all nodes.

        int prev = distPrev[path.back()].prev;
        if (prev == -1)
            return -1;

        path.push_back(prev);
    }

    std::reverse(path.begin(), path.end());

    return distPrev[end].dist;
}

/// @brief Returns distance from start to end or -1 if path was not found.
/// @param distPrev     Should be computed using dijkstra of bellmanFord algorithms.
template<typename D = int>
D pathCost(const std::vector< NodeDistPrev<D> >& distPrev, int end)
{
    assert(static_cast<size_t>(end) < distPrev.size());  // we must have entries for all nodes.
    if ((distPrev[end].dist == std::numeric_limits<D>::max()) && (distPrev[end].prev == -1))
        return -1;
    return distPrev[end].dist;
}


template<typename D = int>
struct DijkstraNode
{
    DijkstraNode(int id, D dist)
        : id(id)
        , dist(dist)
    {}

    int id;    // which node this is
    D dist;  // distance from start to this node

    bool operator<(const DijkstraNode& other) const
    {
        return this->dist < other.dist;
    }

    bool operator>(const DijkstraNode& other) const
    {
        return this->dist > other.dist;
    }
};

/// @brief Computes distances to all graph nodes from given start node.
/// @param end      if -1 compute distances to all nodes, otherwise end when finding shortest path to end.
/// @returns        Distances from start to all nodes, or int max if path was not found. Plus previous nodes from path from start (or -1 if path not found or node == start).
template<typename D = int>
std::vector< NodeDistPrev<D> > dijkstra(const NeighbourListGraph< WeightedEdge<D> >& graph, int start, int end)
{
    std::priority_queue< DijkstraNode<D>, std::vector< DijkstraNode<D> >, std::greater< DijkstraNode<D> > > nodesQueue;
    std::vector< NodeDistPrev<D> > distPrev(graph.numberOfNodes);

    nodesQueue.push(DijkstraNode<D>(start, 0));
    distPrev[start].dist = 0;

    while (!nodesQueue.empty())
    {
        DijkstraNode<D> node = nodesQueue.top();
        nodesQueue.pop();

        if (node.id == end)
        {
            break;
        }

        for (size_t i = 0; i < graph.neighbours[node.id].size(); ++i)
        {
            WeightedEdge<D> edge = graph.neighbours[node.id][i];
            if (distPrev[node.id].dist + edge.cost < distPrev[edge.v].dist)
            {
                distPrev[edge.v].dist = distPrev[node.id].dist + edge.cost;
                distPrev[edge.v].prev = node.id;
                nodesQueue.push(DijkstraNode<D>(edge.v, distPrev[edge.v].dist));
            }
        }
    }

    return distPrev;
}

/// @brief Returns path length or -1 if path was not found.
/// @param path     Set to shortest path from start to end.
template<typename D = int>
D dijkstraPath(const NeighbourListGraph< WeightedEdge<D> >& graph, int start, int end, std::vector<int>& path)
{
    std::vector< NodeDistPrev<D> > distPrev = dijkstra(graph, start, end);
    return reconstructPath(distPrev, start, end, path);
}

/// @brief Returns path length or -1 if path was not found.
template<typename D = int>
D dijkstraCost(const NeighbourListGraph< WeightedEdge<D> >& graph, int start, int end)
{
    std::vector< NodeDistPrev<D> > distPrev = dijkstra(graph, start, end);
    return pathCost(distPrev, end);
}

#endif // Dijkstra_H
