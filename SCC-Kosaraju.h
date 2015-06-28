/// @brief  Strongly Connected Component - Kosaraju's algorithm implementation
/// @author z.skowron

#ifndef SCCKosaraju_H
#define SCCKosaraju_H

// Pseudo kod algorytmu Kosaraju - skomplikowany...
// Two DFS-es...

#include <vector>
#include <queue>
#include <functional>
#include <limits>

#include "NeighbourListGraph.h"
#include "dfs-bfs.h"
#include "ZAssert.h"

/// @note Kosaraju's algorithm doesn't use graph edge costs.
template<typename C = int>
std::vector< std::vector<int> > SCCKosaraju(const NeighbourListGraph< WeightedEdge<C> >& graph)
{
    std::vector< std::vector<int> > sccGroups;  // result: strongly connected components

    NeighbourListGraph< WeightedEdge<C> > transposedGraph = graph.dumbTranspose();
        
    std::vector<int> postTraversal = dfs(graph, false);
    std::vector<bool> visited(graph.numberOfNodes);
    while (!postTraversal.empty())
    {
        int node = postTraversal.back();
        postTraversal.pop_back();
        // if we are in visited node - skip it
        if (visited[node])
        {
            continue;
        }

        std::vector<int> scc = dfsFromNode(node, transposedGraph, visited, true);
        sccGroups.push_back(scc);
    }

    return sccGroups;
}

#endif // SCCKosaraju_H
