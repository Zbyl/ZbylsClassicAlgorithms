
#ifndef NeighbourListGraph_H
#define NeighbourListGraph_H

#include "ZAssert.h"

/// @brief Set's edge "twin" edge (index of correlated edge in edge.v's neighbour list)
template<typename EdgeType>
void setTwinEdge(EdgeType& edge, int twinEdge)
{
}

struct Edge
{
    Edge(int u, int v)
        : u(u), v(v)
    {
    }

    // we could add edge id here to be able to match edges with some other data
    int u;      // start of edge
    int v;      // end of edge
};

template<typename C = int>
struct WeightedEdge : Edge
{
    WeightedEdge(int u, int v, C cost)
        : Edge(u, v)
        , cost(cost)
    {}

    // we could add edge id here to be able to match edges with some other data
    C cost;     // "cost" of edge

    bool operator<(const WeightedEdge& other) const
    {
        return this->cost < other.cost;
    }

    bool operator>(const WeightedEdge& other) const
    {
        return this->cost > other.cost;
    }
};

template<typename C = int>
struct MaxFlowEdge : Edge
{
    MaxFlowEdge(int u, int v, C capacity)
        : Edge(u, v)
        , capacity(capacity)
        , flow()
        , twinEdge(-1)
    {
    }

    C capacity;     // capacity of the edge
    C flow;         // current flow through the edge
    int twinEdge;   // index of this edge's twin edge (same edge, but in different direction) in v's edges; filled only in case of bidirectional edges
};

template<typename C>
void setTwinEdge(MaxFlowEdge<C>& edge, int twinEdge)
{
    edge.twinEdge = twinEdge;
}


/// @brief Graph is represented as:
///        - array of nodes' neighbours,
///        - node A's neighbours is a list of directed edges (A, B, cost).
///        - each bidirectional edge appears in the graph twice: as (A, B, cost), and as (B, A, cost).
template<typename EdgeType>
struct NeighbourListGraph
{
    NeighbourListGraph(int numberOfNodes)
        : numberOfNodes(numberOfNodes)
        , neighbours(numberOfNodes)
    {
    }

    NeighbourListGraph(NeighbourListGraph&& other) noexcept
        : numberOfNodes(other.numberOfNodes)
        , neighbours(std::move(other.neighbours))
    {
    }

    /// @brief Returns index of the edge in edge.u neighbour's list.
    int addDirectedEdge(const EdgeType& edge)
    {
        assert(edge.u < numberOfNodes);
        assert(edge.v < numberOfNodes);
        neighbours[edge.u].push_back(edge);
        return neighbours[edge.u].size() - 1;
    }

    /// @brief Returns index of the added edge in edge.u neighbour's list, and index of the added edge in edge.v neighbour's list.
    std::pair<int, int> addBidirectionalEdge(const EdgeType& edge)
    {
        EdgeType twinEdge = edge;
        twinEdge.u = edge.v;
        twinEdge.v = edge.u;
        int uIdx = addDirectedEdge(edge);
        int vIdx = addDirectedEdge(twinEdge);

        setTwinEdge(neighbours[edge.u][uIdx], vIdx);
        setTwinEdge(neighbours[edge.v][vIdx], uIdx);

        return std::make_pair(uIdx, vIdx);
    }

    int numberOfNodes;  // number of nodes
    std::vector< std::vector< EdgeType > > neighbours;

    /// @brief Creates a transposed graph.
    /// @note  Doesn't take care of twin edges.
    NeighbourListGraph dumbTranspose() const
    {
        NeighbourListGraph transposedGraph(numberOfNodes);

        for (unsigned int n = 0; n < neighbours.size(); ++n)
        {
            for (unsigned int e = 0; e < neighbours[n].size(); ++e)
            {
                EdgeType edge = neighbours[n][e];
                edge.u = neighbours[n][e].v;
                edge.v = neighbours[n][e].u;
                transposedGraph.addDirectedEdge(edge);
            }
        }

        return transposedGraph;
    }
};

#endif // NeighbourListGraph_H
