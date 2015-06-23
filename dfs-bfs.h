/// @brief  Implementations of DFS and BFS helper functions
/// @author z.skowron

#ifndef BFS_DFS_H
#define BFS_DFS_H

#include <vector>
#include <queue>
#include <functional>
#include <limits>

#include "NeighbourListGraph.h"
#include "ZAssert.h"

/// @brief Returns nodes in order of dfs pre-traversal starting from given node.
/// @note This algorithm doesn't use graph edge costs.
template<typename C = int>
void dfsFromNode(int node, const NeighbourListGraph<C>& graph, std::vector<int>& result, std::vector<bool>& visited, bool preTraversal)
{
    assert(visited.size() == graph.numberOfNodes);
    if (visited[node])
        return;

    visited[node] = true;

    if (preTraversal)
        result.push_back(node);

    for (size_t i = 0; i < graph.neighbours[node].size(); ++i)
    {
        WeightedEgde<C> edge = graph.neighbours[node][i];
        dfsFromNode(edge.v, graph, result, visited, preTraversal);
    }

    if (!preTraversal)
        result.push_back(node);
}

/// @brief Returns nodes in order of dfs pre- or post-traversal starting from given node.
/// @note This algorithm doesn't use graph edge costs.
template<typename C = int>
std::vector<int> dfsFromNode(int startNode, const NeighbourListGraph<C>& graph, std::vector<bool>& visited, bool preTraversal)
{
    std::vector<int> result;

    dfsFromNode(startNode, graph, result, visited, preTraversal);

    return result;
}

/// @brief Returns nodes in order of dfs pre- or post-traversal.
/// @note This algorithm doesn't use graph edge costs.
template<typename C = int>
std::vector<int> dfs(const NeighbourListGraph<C>& graph, bool preTraversal)
{
    std::vector<int> result;
    std::vector<bool> visited(graph.numberOfNodes);

    int nextToVisit = 0;
    while (nextToVisit < graph.numberOfNodes)
    {
        // if we are in visited node - skip it
        if (visited[nextToVisit])
        {
            nextToVisit++;
            continue;
        }

        dfsFromNode(nextToVisit, graph, result, visited, preTraversal);
    }

    return result;
}

/// @brief Returns nodes in order of bfs traversal starting from given node.
/// @note This algorithm doesn't use graph edge costs.
template<typename C = int>
std::vector<int> bfsFromNode(int startNode, const NeighbourListGraph<C>& graph, std::vector<int>& result, std::vector<bool>& visited)
{
    std::queue<int> nodesToVisit;
    nodesToVisit.push(startNode);
    while (!nodesToVisit.empty())
    {
        int node = nodesToVisit.front();
        nodesToVisit.pop();
        if (visited[node])
        {
            continue;
        }

        visited[node] = true;
        result.push_back(node);

        for (unsigned int i = 0; i < graph.neighbours[node].size(); ++i)
        {
            WeightedEgde<C> edge = graph.neighbours[node][i];
            nodesToVisit.push(edge.v);
        }
    }

    return result;
}

/// @brief Returns nodes in order of bfs traversal starting from given node.
/// @note This algorithm doesn't use graph edge costs.
template<typename C = int>
std::vector<int> bfsFromNode(int startNode, const NeighbourListGraph<C>& graph, std::vector<bool>& visited)
{
    std::vector<int> result;

    bfsFromNode(startNode, graph, result, visited);

    return result;
}

/// @brief Returns nodes in order of bfs traversal.
/// @note This algorithm doesn't use graph edge costs.
template<typename C = int>
std::vector<int> bfs(int startNode, const NeighbourListGraph<C>& graph)
{
    std::vector<int> result;
    std::vector<bool> visited(graph.numberOfNodes);

    return dfsFromNode(startNode, graph, visited);
}

#endif // BFS_DFS_H
