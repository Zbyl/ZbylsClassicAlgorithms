/// @brief  BellmanFord's algorithm implementation
///         This algorithm will find shortest paths from source node to all other reachable nodes in the graph,
//          provided that there are no negative cycles in the graph.
/// @author z.skowron

#ifndef BellmanFord_H
#define BellmanFord_H

// Pseudo kod algorytmu Bellmana-Forda:
// http://wazniak.mimuw.edu.pl/index.php?title=Zaawansowane_algorytmy_i_struktury_danych/Wyk%C5%82ad_5
// http://rosalind.info/glossary/algo-bellman-ford-algorithm/
//
// BELLMAN-FORD(G, s)
//  for each node v
//    d(v) = inf
//    prev(v) = nil
//  for i=1 to |V|-1 do
//    for each edge (u,v) in G do
//    if d(v) > d(u) + w(u,v) then
//      d(v) > d(u) + w(u,v)
//      prev(v) = u
//  for each edge (u,v) in G do
//    if d(v) > d(u) + w(u,v) then
//      return NIL - there is a negative cycle in the graph! No solution found.
//  return (d, prev)
//

#include <vector>
#include <queue>
#include <functional>
#include <algorithm>

#include "ZAssert.h"
#include "EdgeListGraph.h"
#include "Dijkstra.h"

/// @brief Computes distances to all graph nodes from given start node.
/// @returns        Distances from start to all nodes, or int max if path was not found. Plus previous nodes from path from start (or -1 if path not found or node == start).
///                 Returns empty vector if graph contains negative cycle.
/// @note Use reconstructPath() and pathCost() to analyze the results of this function.
template<typename D = int>
std::vector< NodeDistPrev<D> > bellmanFord(const EdgeListGraph< WeightedEdge<D> >& graph, int start)
{
    std::vector< NodeDistPrev<D> > distPrev(graph.numberOfNodes);

    distPrev[start].dist = 0;

    for (int len = 1; len < graph.numberOfNodes; ++len)
    {
        bool changed = false;

        // In this iteration we know that all paths with length < len have already computed right distances.
        for (auto edge : graph.edges)
        {
            D distThroughEdge = distPrev[edge.u].dist;
            if (distThroughEdge < std::numeric_limits<D>::max())
                distThroughEdge += edge.cost;

            if (distPrev[edge.v].dist > distThroughEdge)
            {
                distPrev[edge.v].dist = distThroughEdge;
                distPrev[edge.v].prev = edge.u;
                changed = true;
            }
        }

        if (!changed)   // Small optimization - if nothing changed in this pass, nothing will change in another. This will help tremendously if graphs are big.
            break;
    }

    for (auto edge : graph.edges)
    {
        D distThroughEdge = distPrev[edge.u].dist;
        if (distThroughEdge < std::numeric_limits<D>::max())
            distThroughEdge += edge.cost;

        if (distPrev[edge.v].dist > distThroughEdge)
        {
            distPrev.clear();   // We have a negative cycle in the graph! Return empty results.
            break;
        }
    }

    return distPrev;
}

#endif // BellmanFord_H
