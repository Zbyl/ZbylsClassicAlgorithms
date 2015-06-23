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

template<typename D = int>
struct DijkstraDistPrev
{
    DijkstraDistPrev()
        : dist(std::numeric_limits<D>::max())
        , prev(-1)
    {}

    DijkstraDistPrev(D dist, int prev)
        : dist(dist)
        , prev(prev)
    {}

    D dist;  // distance from start to this node
    int prev;  // which node is before this on the shortest path
};

/// @brief Computes distances to all graph nodes from given start node.
/// @param end      if -1 compute distances to all nodes, otherwise end when finding shortest path to end.
/// @returns        Distances from start to all nodes, or int max if path was not found. Plus previous nodes from path from start (or -1 if not found).
template<typename D = int>
std::vector< DijkstraDistPrev<D> > dijkstra(const NeighbourListGraph<D>& graph, int start, int end)
{
    std::priority_queue< DijkstraNode<D>, std::vector< DijkstraNode<D> >, std::greater< DijkstraNode<D> > > nodesQueue;
    std::vector< DijkstraDistPrev<D> > distPrev(graph.numberOfNodes);

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
            WeightedEgde<D> edge = graph.neighbours[node.id][i];
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
D dijkstraPath(const NeighbourListGraph<D>& graph, int start, int end, std::vector<int>& path)
{
    std::vector< DijkstraDistPrev<D> > distPrev = dijkstra(graph, start, end);

    path.clear();
    path.push_back(end);

    while (path.back() != start)
    {
        int prev = distPrev[path.back()].prev;
        if (prev == -1)
            return -1;

        path.push_back(prev);
    }

    std::reverse(path.begin(), path.end());

    return distPrev[end].dist;
}

/// @brief Returns path length or -1 if path was not found.
template<typename D = int>
D dijkstraCost(const NeighbourListGraph<D>& graph, int start, int end)
{
    std::vector< DijkstraDistPrev<D> > distPrev = dijkstra(graph, start, end);
    if (distPrev[end].prev == -1)
        return -1;
    return distPrev[end].dist;
}

#endif // Dijkstra_H