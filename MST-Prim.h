/// @brief  Minimal Spanning Tree - Prim's algorithm implementation
/// @author z.skowron

#ifndef MSTPrim_H
#define MSTPrim_H

// Pseudo kod algorytmu Prima:
//
// R = 0    - result edges (MST)
// W = 0    - priority queue of edges from visited nodes
// W += edges from some chosen node
// foreach edge (u, v) from edges W ordered by weight(u, v), increasing:
//    if (v not visited)
//       R += (u, v)
//       v = visited
//       W += edges from v;
// return R
//
// As a minor optimisations minDistances array is kept. It contains minimal distances from current node set to all other nodes.
// It is used to skip edges that won't be used in MST anyway.
//

#include <vector>
#include <queue>
#include <functional>
#include <limits>

#include "NeighbourListGraph.h"
#include "ZAssert.h"

template<typename C = int>
std::vector< WeightedEgde<C> > mstPrim(const NeighbourListGraph<C>& graph)
{
    std::vector< WeightedEgde<C> > acceptedEdges;
    std::priority_queue< WeightedEgde<C>, std::vector< WeightedEgde<C> >, std::greater< WeightedEgde<C> > > workingEdges;
    std::vector<bool> visited(graph.numberOfNodes);
    std::vector<int> minDistances(graph.numberOfNodes, std::numeric_limits<int>::max());

    visited[0] = true;
    minDistances[0] = 0;
    for (size_t i = 0; i < graph.neighbours[0].size(); ++i)
    {
        workingEdges.push(graph.neighbours[0][i]);
    }

    int numberOfNodesInMST = 1;
    while (numberOfNodesInMST < graph.numberOfNodes)
    {
        WeightedEgde<C> edge = workingEdges.top();
        workingEdges.pop();

        if (visited[edge.v])
            continue;

        visited[edge.v] = true;
        for (size_t i = 0; i < graph.neighbours[edge.v].size(); ++i)
        {
            WeightedEgde<C> newEdge = graph.neighbours[edge.v][i];
            if (newEdge.cost > minDistances[newEdge.v])
                continue;
            minDistances[newEdge.v] = newEdge.cost;
            workingEdges.push(newEdge);
        }

        acceptedEdges.push_back(edge);
        numberOfNodesInMST++;
    }

    return acceptedEdges;
}

template<typename C = int>
int mstPrimCost(const NeighbourListGraph<C>& graph)
{
    std::vector< WeightedEgde<C> > acceptedEdges = mstPrim(graph);

    int cost = 0;
    for (auto edge : acceptedEdges)
    {
        cost += edge.cost;
    }

    return cost;
}

#endif // MSTPrim_H
