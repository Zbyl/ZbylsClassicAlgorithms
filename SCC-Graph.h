/// @brief  Core for creating Strongly Connected Component graph
/// @author z.skowron

#ifndef SCCGraph_H
#define SCCGraph_H

// Call SCC to get strongly connected components of a graph.
// Then assign to all vertices index of the strongly connected component it belongs to.
// Then build a graph of all strongly connected components.
// Note: strongly connected component graph is always a DAG (Directed Acyclic Graph)

#include <vector>
#include <queue>
#include <functional>
#include <limits>

#include "NeighbourListGraph.h"
#include "IncidenceMatrixGraph.h"
#include "dfs-bfs.h"
#include "ZAssert.h"

/// @brief Computes graph of strongly connected components (in the form of IncidenceMatrix).
/// @todo  It has O(n^2 + m) time complexity, and O(n^2) memory complexity - isn't there a better algorithm?!
template<typename EdgeType>
IncidenceMatrix SCCGraph(const NeighbourListGraph<EdgeType>& graph, const std::vector< std::vector<int> >& stronglyConnectedComponents)
{
    std::vector<int> sccIndexes(graph.numberOfNodes);
    for (size_t i = 0; i < stronglyConnectedComponents.size(); ++i)
    {
        for (int node : stronglyConnectedComponents[i])
        {
            sccIndexes[node] = i;
        }
    }

    IncidenceMatrix sccGraph(stronglyConnectedComponents.size());
    for (const auto& neighbours : graph.neighbours)
    {
        for (const auto& edge : neighbours)
        {
            int scc0 = sccIndexes[edge.u];
            int scc1 = sccIndexes[edge.v];
            sccGraph.addDirectedEdge(Edge(scc0, scc1));
        }
    }

    return sccGraph;
}

/// @brief Checks if graph is semi-connected (for every u,v there is a path u-->v or v-->u).
/// @note  Graph is semi connected if all strongly connected components form a path.
///        http://himangi774.blogspot.com/2013/12/check-if-graph-is-strongl-connected.html
/// @todo  It has O(n^2 + m) time complexity, and O(n^2) memory complexity - isn't there a better algorithm?!
template<typename EdgeType>
bool isSemiConnectedGraph(const NeighbourListGraph<EdgeType>& graph, const IncidenceMatrix& sccGraph)
{
    // We convert sccGraph to neighbour list graph to run topology sort on it.
    // @note Improve: this is not a necessary step.
    NeighbourListGraph<Edge> sccListGraph(sccGraph.numberOfNodes);
    for (int i = 0; i < sccGraph.numberOfNodes; ++i)
    {
        for (int j = 0; j < sccGraph.numberOfNodes; ++j)
        {
            if (sccGraph.hasDirectedEdge(i, j))
            {
                sccListGraph.addDirectedEdge( Edge(i, j) );
            }
        }
    }

    // Component graph is a DAG, so dfs is enough to do a topological sort.
    std::vector<int> sortedNodes = dfs(sccListGraph, false);
    std::reverse(sortedNodes.begin(), sortedNodes.end());

    for (size_t i = 1; i < sortedNodes.size(); ++i)
    {
        if (!sccGraph.hasDirectedEdge(sortedNodes[i - 1], sortedNodes[i]))
            return false;
    }

    return true;
}

#endif // SCCGraph_H
