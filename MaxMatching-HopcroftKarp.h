/// @brief  Max Bipartitie Matching - Hopcroft Karp's algorithm implementation
/// @author z.skowron

#ifndef MaxMatching_HopcroftKarp_H
#define MaxMatching_HopcroftKarp_H

// @todo: Check if graph is bipartite, and if it is - split it

// Hopcroft-Karp algorithm for finding maximal bipartite matching:
//    https://youtu.be/n7r4Dp6cVg8
// Note: This is very similar to Dinic's MaxFlow algorithm!
//
//    Input: Bipartite graph B = V1, V2, E (from v1 to v2 only)
//    Create helper graph: G = (V1 + V2), (E + inv E)
//
//    M = 0       - found matching
//    while true
//      Do a BFS from unmatched vertices from V1, generating full levels of BFS (using unmatched-edge, matched-edge, unmatched-edge...), until we find unmatched vertices in the level. Then we stop.
//        If no unmatched vertices in V2 where reached - end.
//      We find maximal set of vertex disjoint paths from unmatched vertices on the last level of BFS.
//        (Use DFS where levels fount in BFS increase by one.)
//      We augment the M

#include <vector>
#include <queue>
#include <functional>
#include <limits>

#include "NeighbourListGraph.h"
#include "ZAssert.h"

struct MaxMatchingEdge : Edge
{
    MaxMatchingEdge(int u, int v)
        : Edge(u, v)
        , matched(false)
    {}

    bool matched;   // if true, then edge is part of maximum matching
};


/// @brief Run BFS from given set of vertices.
///        Even edges must be unmatched. Odd edges must be matched.
///        BFS will end after processing full level in which unmatched vertex from V2 was reached.
/// @note  Returns false if no unmatched vertex from V2 was reached.
///        distances will contain vertex distances from start node set, or -1 if vertex was not reached
bool maxMatchingHKBFS(const NeighbourListGraph<MaxMatchingEdge>& graph, const std::vector<int>& startNodes, const std::vector<bool>& nodesMatched, std::vector<int>& distances)
{
    assert(nodesMatched.size() <= graph.numberOfNodes);
    assert(startNodes.size() <= graph.numberOfNodes);
    assert(distances.size() == graph.numberOfNodes);
    std::fill(distances.begin(), distances.end(), -1);

    std::deque<int> queue; // BFS queue (note - improve performance by preallocating one array for all BFS runs)

    for (auto node : startNodes)
    {
        assert(!nodesMatched[node]);
        distances[node] = 0;
        queue.push_back(node);
    }

    int finishOnLevel = -1;
    while (!queue.empty())
    {
        int node = queue.front();
        queue.pop_front();

        int level = distances[node];
        bool isV1 = ((level & 1) == 0); // it could be done in a more explicit way; here we take advantage of the fact, that the graph is bipartite

        if (level == finishOnLevel)
            break;

        for (size_t i = 0; i < graph.neighbours[node].size(); ++i)
        {
            const MaxMatchingEdge& edge = graph.neighbours[node][i];

            // from V1 we must go through unmatched edges
            // from V2 we must go through matched edges
            if (isV1 && edge.matched)
                continue;
            if (!isV1 && !edge.matched)
                continue;

            // if we reached an unmatched node, we shouldn't go any lower
            if (!nodesMatched[node])
            {
                assert(!isV1);
                assert( (finishOnLevel == -1) || (finishOnLevel == level + 1) );
                finishOnLevel = level + 1;
            }

            if (distances[edge.v] == -1)
            {
                distances[edge.v] = level + 1;
                queue.push_back(edge.v);
            }
        }
    }

    return finishOnLevel != -1;
}

/// @brief This method will find maximal vertex-disjoint set of augmenting paths.
bool maxMatchingHKDFS(const NeighbourListGraph<MaxMatchingEdge>& graph, int startNode, const std::vector<bool>& nodesMatched, std::vector<int>& distances)
{
}

/// @brief This method will go throw all shortest paths and saturate the flow using them.
///        Returns total flow that was added (will be less than maxFlowAcceptable).
/// @param maxFlowAcceptable    Maximum flow that can flow up to given node from the path before it.
template<typename C = int>
int maxFlowDinicDFS(NeighbourListGraph< MaxFlowEdge<C> >& graph, int node, int endNode, std::vector<int>& distances, int maxFlowAcceptable)
{
    assert(distances.size() == graph.numberOfNodes);

    if (node == endNode)
        return maxFlowAcceptable;

    int maxFlowLeft = maxFlowAcceptable;
    for (size_t i = 0; i < graph.neighbours[node].size(); ++i)
    {
        MaxFlowEdge<C>& edge = graph.neighbours[node][i];
        if (distances[edge.v] != distances[node] + 1)
            continue; // shortest paths have increasing distances from start node

        int flowLeft = edge.capacity - edge.flow;
        // if edge is saturated - skip it
        if (flowLeft <= 0)
        {
            assert(flowLeft == 0); // we will never have negative flows
            continue;
        }

        int childrenFlow = maxFlowDinicDFS(graph, edge.v, endNode, distances, std::min(flowLeft, maxFlowLeft));
        edge.flow += childrenFlow;

        MaxFlowEdge<C>& twinEdge = graph.neighbours[edge.v][edge.twinEdge];
        twinEdge.flow -= childrenFlow;

        maxFlowLeft -= childrenFlow;
        if (maxFlowLeft == 0)
            break;
    }

    return maxFlowAcceptable - maxFlowLeft;
}

/// @brief Return max flow in graph (using Dinic's algorithm: O(N^2 * M)).
///        (Graph should have flow in all edges set to zero.)
/// @note  The resulting flow will be stored in graph edges.
template<typename C = int>
int maxFlowDinic(NeighbourListGraph< MaxFlowEdge<C> >& graph, int startNode, int endNode)
{
    int maxFlow = 0;

    std::vector<int> distances(graph.numberOfNodes);
    while (maxFlowDinicBFS(graph, startNode, endNode, distances))
    {
        int maxFlowAdd = maxFlowDinicDFS(graph, startNode, endNode, distances, std::numeric_limits<int>::max());
        assert(maxFlowAdd > 0);
        maxFlow += maxFlowAdd;
    }

    return maxFlow;
}

#endif // MaxMatching_HopcroftKarp_H
