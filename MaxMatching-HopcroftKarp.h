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

/// @brief This method will check if graph is bipartite.
/// @param graph        Input graph (V1 V2 E).
/// @param coloring     [out] Set to true for one set vertices (V1) false for the other (V2).
/// @returns true if graph is bipartite; false otherwise
template<typename EdgeType>
bool isBipartite(const NeighbourListGraph<EdgeType>& graph, std::vector<bool>& coloring)
{
    assert(coloring.size() == graph.numberOfNodes);
    std::fill(coloring.begin(), coloring.end(), false);

    std::vector<bool> nodesVisited(graph.numberOfNodes);

    for (int node = 0; node < graph.numberOfNodes; ++node)
    {
        if (nodesVisited[node])
            continue;

        if (!isBipartiteDFSFromNode(graph, node, true, coloring, nodesVisited))
            return false;
    }

    return true;
}

/// @brief DFS pass for checking if graph is bipartite.
/// @param graph            Input graph (V1 V2 E).
/// @param node             Node to start DFS from.
/// @param coloring         Set to true for one set vertices (V1) false for the other (V2).
/// @param nodesVisited     Set of already visited nodes.
/// @returns true if graph is bipartite; false otherwise
template<typename EdgeType>
bool isBipartiteDFSFromNode(const NeighbourListGraph<EdgeType>& graph, int node, bool color, std::vector<bool>& coloring, std::vector<bool>& nodesVisited)
{
    if (nodesVisited[node])
    {
        return color == coloring[node];
    }

    nodesVisited[node] = true;
    coloring[node] = color;

    for (auto& edge : graph.neighbours[node])
    {
        if (!isBipartiteDFSFromNode(graph, edge.v, !color, coloring, nodesVisited))
            return false;
    }

    return true;
}

/// @brief Edge type used while finding maximal matching in bipartite graph. 
struct MaxMatchingEdge : Edge
{
    MaxMatchingEdge(int u, int v)
        : Edge(u, v)
        , matched(false)
        , twinEdge(-1)
    {}

    bool matched;   // if true, then edge is part of maximum matching
    int twinEdge;   // index of this edge's twin edge (same edge, but in different direction) in v's edges; filled only in case of bidirectional edges
};

void setTwinEdge(MaxMatchingEdge& edge, int twinEdge)
{
    edge.twinEdge = twinEdge;
}


/// @brief Run BFS from given set of vertices, such that even edges will be unmatched, and odd edges will be matched.
///        BFS will end after processing full layer of nodes in which unmatched vertex from V2 was reached.
/// @param startNodes       Collection of unmatched nodes in V1 (for Hopcroft-Karp algorithm is should contain all such nodes).
/// @param nodesMatched     Collection of flags that say whether given node is matched in the matching we are trying to improve.
/// @param distances        [out] Distances from start node set, or -1 if vertex was not reached during BFS search (either not-reachable, or BFS stopped before reching it).
/// @return distance on which unmatched vertices from V2 were reached; -1 if none were reached.
int maxMatchingHKBFS(const NeighbourListGraph<MaxMatchingEdge>& graph, const std::vector<int>& startNodes, const std::vector<bool>& nodesMatched, std::vector<int>& distances)
{
    assert(startNodes.size() <= graph.numberOfNodes);
    assert(nodesMatched.size() == graph.numberOfNodes);
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
            int other = edge.v;

            // from V1 we must go through unmatched edges
            // from V2 we must go through matched edges
            if (isV1 && edge.matched)
                continue;
            if (!isV1 && !edge.matched)
                continue;

            // if we reached an unmatched node, we shouldn't go any lower
            if (!nodesMatched[other])
            {
                assert(isV1); // to unmatched nodes we should be able to get only from V1
                assert( (finishOnLevel == -1) || (finishOnLevel == level + 1) );
                finishOnLevel = level + 1;
            }

            if (distances[other] == -1)
            {
                distances[other] = level + 1;
                queue.push_back(other);
            }
        }
    }

    return finishOnLevel;
}

/// @brief This method will find maximal vertex-disjoint set of augmenting paths.
/// @param finishOnLevel    Length of shortest augmenting paths in the graph.
/// @param nodesVisited     Nodes that already belong to one augmenting path, and therefore cannot be used again.
/// @returns true if augmenting path was found; false otherwise
bool maxMatchingHKDFSFromNode(NeighbourListGraph<MaxMatchingEdge>& graph, int node, int finishOnLevel, std::vector<bool>& nodesVisited, std::vector<bool>& nodesMatched, const std::vector<int>& distances)
{
    if (nodesVisited[node])
        return false;
    nodesVisited[node] = true;

    int level = distances[node];
    assert(level >= 0);

    // if we have reached a node on last level, we have found an augmenting path
    if (level == finishOnLevel)
    {
        nodesMatched[node] = !nodesMatched[node];   // augment matching
        return true;
    }

    for (auto& edge : graph.neighbours[node])
    {
        int other = edge.v;
        if (distances[other] != level + 1)
            continue;

        bool pathFound = maxMatchingHKDFSFromNode(graph, other, finishOnLevel, nodesVisited, nodesMatched, distances);
        if (pathFound)
        {
            MaxMatchingEdge& twinEdge = graph.neighbours[other][edge.twinEdge];
            nodesMatched[node] = !nodesMatched[node];   // augment matching
            edge.matched = !edge.matched;   // augment matching
            twinEdge.matched = !twinEdge.matched;   // augment matching
            assert(edge.matched == twinEdge.matched);
            return true;
        }
    }

    return false;
}

/// @brief This method will find maximal matching in a bipartitie graph (V1, V2, E) using Hopcroft-Karp algorithm.
/// @param graph            Input graph. All edges must be unmatched. On return edges that were selected in the found maximal matching will have a flag set.
/// @param coloring         [in] Set to true for one set vertices (V1) false for the other (V2).
/// @param nodesMatched     [out] Nodes that were matched in the found maximal matching will have a flag set (from both V1 and V2).
/// @returns The value of maximal matching in bipartite graph.
int maxMatchingHK(NeighbourListGraph<MaxMatchingEdge>& graph, const std::vector<bool>& coloring, std::vector<bool>& nodesMatched)
{
    assert(coloring.size() == graph.numberOfNodes);
    assert(nodesMatched.size() == graph.numberOfNodes);
    std::fill(nodesMatched.begin(), nodesMatched.end(), false);

    std::vector<int> distances(graph.numberOfNodes);

    while (true)
    {
        // We have to start BFS from unmatched V1 vertices
        std::vector<int> startNodes;
        for (size_t node = 0; node < coloring.size(); ++node)
        {
            if (coloring[node] && !nodesMatched[node])
                startNodes.push_back(node);
        }

        int finishOnLevel = maxMatchingHKBFS(graph, startNodes, nodesMatched, distances);
        if (finishOnLevel == -1)
            break;

        std::vector<bool> nodesVisited(graph.numberOfNodes);
        for (auto startNode : startNodes)
        {
            assert(!nodesMatched[startNode]);
            maxMatchingHKDFSFromNode(graph, startNode, finishOnLevel, nodesVisited, nodesMatched, distances);
        }
    }

    int matchedCount = std::count(nodesMatched.begin(), nodesMatched.end(), true);
    assert( (matchedCount & 1) == 0 );
    return matchedCount / 2;
}

#endif // MaxMatching_HopcroftKarp_H
