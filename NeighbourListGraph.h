
#ifndef NeighbourListGraph_H
#define NeighbourListGraph_H

#include "ZAssert.h"

template<typename C = int>
struct WeightedEgde
{
    WeightedEgde(int u, int v, C cost)
        : u(u), v(v), cost(cost)
    {}

    // we could add edge id here to be able to match edges with some other data
    int u, v;
    C cost;

    bool operator<(const WeightedEgde& other) const
    {
        return this->cost < other.cost;
    }

    bool operator>(const WeightedEgde& other) const
    {
        return this->cost > other.cost;
    }
};

/// @brief Graph is represented as:
///        - array of nodes' neighbours,
///        - node A's neighbours is a list of directed edges (A, B, cost).
///        - each bidirectional edge appears in the graph twice: as (A, B, cost), and as (B, A, cost).
template<typename C = int>
struct NeighbourListGraph
{
    NeighbourListGraph(int numberOfNodes)
        : numberOfNodes(numberOfNodes)
        , neighbours(numberOfNodes)
    {
    }

    void addDirectedEdge(int u, int v, C cost)
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);
        neighbours[u].push_back(WeightedEgde<C>(u, v, cost));
    }

    void addBidirectionalEdge(int u, int v, C cost)
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);
        neighbours[u].push_back(WeightedEgde<C>(u, v, cost));
        neighbours[v].push_back(WeightedEgde<C>(v, u, cost));
    }

    int numberOfNodes;  // number of nodes
    std::vector< std::vector< WeightedEgde<C> > > neighbours;

    /// @brief Creates a transposed graph.
    NeighbourListGraph<C> transpose() const
    {
        NeighbourListGraph<C> transposedGraph(numberOfNodes);

        for (unsigned int n = 0; n < neighbours.size(); ++n)
        {
            for (unsigned int e = 0; e < neighbours[n].size(); ++e)
            {
                WeightedEgde<C> edge = neighbours[n][e];
                transposedGraph.addDirectedEdge(edge.v, edge.u, edge.cost);
            }
        }

        return transposedGraph;
    }
};

#endif // NeighbourListGraph_H
