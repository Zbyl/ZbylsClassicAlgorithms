/// @brief  Minimal Spanning Tree - Kruskal's algorithm implementation
/// @author z.skowron

#ifndef MSTKruskal_H
#define MSTKruskal_H

// Pseudo kod algorytmu Kruskala:
//
// R = 0    - result edges (MST)
// foreach v in G.V:
//    MAKE-SET(v)
// foreach edge (u, v) from edges G.E ordered by weight(u, v), increasing:
//    if FIND-ROOT(u) != FIND-ROOT(v):
//       R += (u, v)
//       UNION(u, v)
// return R
//

#include <vector>
#include <algorithm>

#include "FindUnion.h"
#include "NeighbourListGraph.h"
#include "ZAssert.h"

/// @brief Graph is represented as:
///        - number of nodes,
///        - list of undirected edges (begin, end, cost).
template<typename C = int>
struct EdgeListGraph
{
    EdgeListGraph(int numberOfNodes)
        : numberOfNodes(numberOfNodes)
    {
    }

    void addEdge(int u, int v, C cost)
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);
        edges.push_back(WeightedEgde<C>(u, v, cost));
    }

    int numberOfNodes;  // number of nodes
    std::vector< WeightedEgde<C> > edges;
};

template<typename C = int>
std::vector< WeightedEgde<C> > mstKruskal(const EdgeListGraph<C>& graph)
{
    FindUnion FindUnion(graph.numberOfNodes);
    std::vector< WeightedEgde<C> > sourceEdges(graph.edges.begin(), graph.edges.end());
    std::vector< WeightedEgde<C> > acceptedEdges;

    std::sort(sourceEdges.begin(), sourceEdges.end());

    int currentEdge = 0;
    int numberOfNodesInMST = 1;
    while (numberOfNodesInMST < graph.numberOfNodes)
    {
        WeightedEgde<C> edge = sourceEdges[currentEdge];
        currentEdge++;

        int root0 = FindUnion.findRoot(edge.u);
        int root1 = FindUnion.findRoot(edge.v);
        if (root0 == root1)
            continue;

        FindUnion.join(root0, root1);
        acceptedEdges.push_back(edge);
        numberOfNodesInMST++;
    }

    return acceptedEdges;
}

template<typename C = int>
int mstKruskalCost(const EdgeListGraph<C>& graph)
{
    std::vector< WeightedEgde<C> > acceptedEdges = mstKruskal(graph);

    int cost = 0;
    for (auto edge : acceptedEdges)
    {
        cost += edge.cost;
    }

    return cost;
}

#endif // MSTKruskal_H
