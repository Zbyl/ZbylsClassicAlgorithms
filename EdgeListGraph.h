
#ifndef EdgeListGraph_H
#define EdgeListGraph_H

#include "ZAssert.h"
#include "EdgeListGraph.h"

/// @brief Graph is represented as:
///        - number of nodes,
///        - list of undirected edges (begin, end, cost).
template<typename EdgeType>
struct EdgeListGraph
{
    EdgeListGraph(int numberOfNodes)
        : numberOfNodes(numberOfNodes)
    {
    }

    void addEdge(const EdgeType& edge)
    {
        assert(edge.u < numberOfNodes);
        assert(edge.v < numberOfNodes);
        edges.push_back(edge);
    }

    int numberOfNodes;  // number of nodes
    std::vector<EdgeType> edges;
};

#endif // EdgeListGraph_H
