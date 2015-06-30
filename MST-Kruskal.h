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
#include "EdgeListGraph.h"
#include "ZAssert.h"

template<typename EdgeType>
std::vector<EdgeType> mstKruskal(const EdgeListGraph<EdgeType>& graph)
{
    FindUnion FindUnion(graph.numberOfNodes);
    std::vector<EdgeType> sourceEdges(graph.edges.begin(), graph.edges.end());
    std::vector<EdgeType> acceptedEdges;

    std::sort(sourceEdges.begin(), sourceEdges.end());

    int currentEdge = 0;
    int numberOfNodesInMST = 1;
    while (numberOfNodesInMST < graph.numberOfNodes)
    {
        EdgeType edge = sourceEdges[currentEdge];
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

template<typename EdgeType>
int mstKruskalCost(const EdgeListGraph<EdgeType>& graph)
{
    std::vector<EdgeType> acceptedEdges = mstKruskal(graph);

    int cost = 0;
    for (auto edge : acceptedEdges)
    {
        cost += edge.cost;
    }

    return cost;
}

#endif // MSTKruskal_H
