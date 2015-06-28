/// @brief  Implementation of Topology Sort
/// @author z.skowron

#ifndef TopologySort_H
#define TopologySort_H

#include <vector>
#include <queue>
#include <functional>
#include <limits>

#include "NeighbourListGraph.h"
#include "ZAssert.h"

/// @brief Starts DFS from given node.
/// @note This algorithm doesn't use graph edge costs.
template<typename C = int>
bool topSortFromNode(int node, const NeighbourListGraph< WeightedEdge<C> >& graph, std::vector<int>& result, std::vector<bool>& visited, std::vector<bool>& processing)
{
    assert(processing.size() == graph.numberOfNodes);
    assert(visited.size() == graph.numberOfNodes);

    if (processing[node])
        return false; // We have found a cycle. It's impossible to sort the graph.

    if (visited[node])
        return true;

    processing[node] = true;
    visited[node] = true;

    for (size_t i = 0; i < graph.neighbours[node].size(); ++i)
    {
        WeightedEdge<C> edge = graph.neighbours[node][i];
        if (!topSortFromNode(edge.v, graph, result, visited, processing))
            return false;
    }

    result.push_back(node);

    processing[node] = false;

    return true;
}

/// @brief Sorts graph topologically.
///        Edge from p to k means that p must be done before k.
/// @note This algorithm doesn't use graph edge costs.
template<typename C = int>
bool topSort(const NeighbourListGraph< WeightedEdge<C> >& graph, std::vector<int>& result)
{
    result.clear();

    std::vector<bool> visited(graph.numberOfNodes);
    std::vector<bool> processing(graph.numberOfNodes);  // nodes that are currently being processed

    int nextToVisit = 0;
    while (nextToVisit < graph.numberOfNodes)
    {
        // if we are in visited node - skip it
        if (visited[nextToVisit])
        {
            nextToVisit++;
            continue;
        }

        if (!topSortFromNode(nextToVisit, graph, result, visited, processing))
            return false;
    }

    return true;
}

#endif // TopologySort_H
