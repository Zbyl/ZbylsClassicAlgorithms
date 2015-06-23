/// @brief  Strongly Connected Component - Tarjan's algorithm implementation
/// @author z.skowron

#ifndef SCCTarjan_H
#define SCCTarjan_H

// Pseudo kod algorytmu Tarjana - skomplikowany...
// Each node has:
// - rank - number (in DFS order) of lowest node we reached below this node.

#include <vector>
#include <queue>
#include <functional>
#include <limits>

#include "NeighbourListGraph.h"
#include "ZAssert.h"

/// @note Tarjan's algorithm doesn't use graph edge costs.
template<typename C = int>
class SCCTarjanHelper
{
private:
    const NeighbourListGraph<C>& graph;
    std::vector< std::vector<int> > sccGroups;  // result: strongly connected components
    std::vector<int> sccStack;                  // stack of nodes, from which we will extract strongly connected components
    std::vector<int> rank;                      // rank == 0 means, that the node was not yet visited, int max means - already part of scc
    int rankCounter;                            // used to number nodes in DFS order

public:
    SCCTarjanHelper(const NeighbourListGraph<C>& graph)
        : graph(graph)
        , rank(graph.numberOfNodes)
        , rankCounter(1)
    {}

    void dfsTarjan(int node)
    {
        sccStack.push_back(node);
        rank[node] = rankCounter++;

        bool rankCorrected = false;
        for (size_t i = 0; i < graph.neighbours[node].size(); ++i)
        {
            WeightedEgde<C> edge = graph.neighbours[node][i];
            if (rank[edge.v] == 0)
            {
                // node was not yet visited, so visit it
                dfsTarjan(edge.v);
            }

            int nextRank = rank[edge.v];
            if (nextRank < rank[node])
            {
                // node was visited, update rank
                rank[node] = nextRank;
                rankCorrected = true;
            }
            // if node has higher rank (our was corrected, or node is already part of another scc), we ignore it
        }

        if (!rankCorrected)
        {
            // We are in the first node, whose rank was not corrected.
            // So all elements on the stack up to this node is a strongly connected component.

            sccGroups.push_back( std::vector<int>() );
            while (true)
            {
                int sccNode = sccStack.back();
                sccStack.pop_back();
                sccGroups.back().push_back(sccNode);
                rank[sccNode] = std::numeric_limits<int>::max(); // mark as already part of an SCC
                if (sccNode == node)
                    break;
            }
        }
    }

public:
    std::vector< std::vector<int> > start()
    {
        int nextToVisit = 0;

        while (nextToVisit < graph.numberOfNodes)
        {
            // if we are in visited node - skip it
            if (rank[nextToVisit] != 0)
            {
                nextToVisit++;
                continue;
            }

            dfsTarjan(nextToVisit);
        }

        return sccGroups;
    }

};

/// @note Tarjan's algorithm doesn't use graph edge costs.
template<typename C = int>
std::vector< std::vector<int> > SCCTarjan(const NeighbourListGraph<C>& graph)
{
    SCCTarjanHelper<C> helper(graph);
    return helper.start();
}

#endif // SCCTarjan_H
