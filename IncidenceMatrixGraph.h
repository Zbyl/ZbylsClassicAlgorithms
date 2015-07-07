
#ifndef IncidenceMatrixGraph_H
#define IncidenceMatrixGraph_H

#include <limits>
#include <vector>
#include <unordered_set>
#include <unordered_map>

#include "ZAssert.h"
#include "NeighbourListGraph.h"

/// @brief Incidence matrix is an O(1) set of pairs.
///        Implemented as a two dimensinal array of bools.
struct IncidenceMatrix
{
    IncidenceMatrix(int numberOfNodes)
        : numberOfNodes(numberOfNodes)
        , incidenceMatrix(numberOfNodes * numberOfNodes)
    {
    }

    IncidenceMatrix(IncidenceMatrix&& other) noexcept
        : numberOfNodes(other.numberOfNodes)
        , incidenceMatrix(std::move(other.incidenceMatrix))
    {
    }

    bool hasDirectedEdge(int u, int v) const
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);
        return incidenceMatrix[u * numberOfNodes + v];
    }

    /// @brief Returns true if both edges u->v and v->u exist.
    bool hasBidirectionalEdge(int u, int v) const
    {
        return hasDirectedEdge(u, v) && hasDirectedEdge(v, u);
    }

    void addDirectedEdge(const Edge& edge)
    {
        assert(edge.u < numberOfNodes);
        assert(edge.v < numberOfNodes);
        incidenceMatrix[edge.u * numberOfNodes + edge.v] = true;
    }

    /// @brief Returns index of the added edge in edge.u neighbour's list, and index of the added edge in edge.v neighbour's list.
    void addBidirectionalEdge(const Edge& edge)
    {
        Edge twinEdge = edge;
        twinEdge.u = edge.v;
        twinEdge.v = edge.u;

        addDirectedEdge(edge);
        addDirectedEdge(twinEdge);
    }

    int numberOfNodes;  // number of nodes
    std::vector<bool> incidenceMatrix; // incidenceMatrix[from * N + to] - so rows are nodes reachable from node 'from'
};

/// @brief Weight matrix is a matrix of edge or path weights between all nodes.
///        std::numeric_limits<C>::max() means there is no path between nodes.
///        See this article for explanation:
///        http://wazniak.mimuw.edu.pl/index.php?title=Zaawansowane_algorytmy_i_struktury_danych/Wyk%C5%82ad_6
template<typename C>
struct WeightMatrix
{
    /// @brief Initializes weight matrix to identity with respect to distance multiplication if identity is true. Otherwise sets all fields to infinity.
    ///        Identity: (8 represents infinity)
    ///        0  8  8  8  8
    ///        8  0  8  8  8
    ///        8  8  0  8  8
    ///        8  8  8  0  8
    ///        8  8  8  8  0
    WeightMatrix(int numberOfNodes, bool identity = true)
        : numberOfNodes(numberOfNodes)
        , weightMatrix(numberOfNodes * numberOfNodes, std::numeric_limits<C>::max())
    {
        if (identity)
        {
            for (int i = 0; i < numberOfNodes; ++i)
            {
                setWeight(i, i, 0);
            }
        }
    }

    WeightMatrix(WeightMatrix&& other) noexcept
        : numberOfNodes(other.numberOfNodes)
        , weightMatrix(std::move(other.weightMatrix))
    {
    }

    /// @brief Returns std::numeric_limits<C>::max() if there is no path between nodes.
    C getWeight(int u, int v) const
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);
        return weightMatrix[u * numberOfNodes + v];
    }

    void setWeight(int u, int v, C cost)
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);
        weightMatrix[u * numberOfNodes + v] = cost;
    }

    void addDirectedEdge(const WeightedEdge<C>& edge)
    {
        setWeight(edge.u, edge.v, edge.cost);
    }

    /// @brief Returns index of the added edge in edge.u neighbour's list, and index of the added edge in edge.v neighbour's list.
    void addBidirectionalEdge(const WeightedEdge<C>& edge)
    {
        Edge twinEdge = edge;
        twinEdge.u = edge.v;
        twinEdge.v = edge.u;

        addDirectedEdge(edge);
        addDirectedEdge(twinEdge);
    }

    /// @brief Returns a matrix that represents shortest paths that are concatenation of paths represented by both operand matrixes.
    ///        Cost: O(|V|^3)
    ///        See: http://wazniak.mimuw.edu.pl/index.php?title=Zaawansowane_algorytmy_i_struktury_danych/Wyk%C5%82ad_6#Iloczyn_odleg.C5.82o.C5.9Bci_i_jego_w.C5.82a.C5.9Bciwo.C5.9Bci
    static void distanceMultiply(const WeightMatrix& lhs, const WeightMatrix& rhs, WeightMatrix& result)
    {
        assert(lhs.numberOfNodes == result.numberOfNodes);
        assert(rhs.numberOfNodes == result.numberOfNodes);

        for (int i = 0; i < numberOfNodes; ++i)
        {
            for (int j = 0; j < numberOfNodes; ++j)
            {
                C minVal = std::numeric_limits<C>::max();
                for (int k = 0; k < numberOfNodes; ++k)
                {
                    C lhsWeight = lhs.getWeight(i, k);
                    if (lhsWeight == std::numeric_limits<C>::max())
                        continue;
                    C rhsWeight = rhs.getWeight(k, j);
                    if (rhsWeight == std::numeric_limits<C>::max())
                        continue;
                    C curVal = lhsWeight + rhsWeight;
                    if (curVal < minVal)
                        minVal = curVal;
                }
                result.setWeight(i, j, minVal);
            }
        }

        return result;
    }

    int numberOfNodes;  // number of nodes
    std::vector<C> weightMatrix; // weightMatrix[from * N + to] = path length between nodes.
};

/// @brief Sparse incidence matrix is an O(1) set of pairs.
///        Implemented as a hash map of bools.
template<typename EdgeType, typename IndexType = int>
struct SparseIncidenceMatrix
{
    SparseIncidenceMatrix(int numberOfNodes)
        : numberOfNodes(numberOfNodes)
    {
        assert(static_cast<IndexType>(numberOfNodes * numberOfNodes) / numberOfNodes == numberOfNodes);
    }

    SparseIncidenceMatrix(SparseIncidenceMatrix&& other) noexcept
        : numberOfNodes(other.numberOfNodes)
        , incidenceSet(std::move(other.incidenceSet))
    {
    }

    bool hasDirectedEdge(int u, int v) const
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);
        return incidenceSet.count(u * numberOfNodes + v) > 0;
    }

    /// @brief Returns true if both edges u->v and v->u exist.
    bool hasBidirectionalEdge(int u, int v) const
    {
        return hasDirectedEdge(u, v) && hasDirectedEdge(v, u);
    }

    void addDirectedEdge(const Edge& edge)
    {
        assert(edge.u < numberOfNodes);
        assert(edge.v < numberOfNodes);
        incidenceSet.insert(edge.u * numberOfNodes + edge.v);
    }

    /// @brief Returns index of the added edge in edge.u neighbour's list, and index of the added edge in edge.v neighbour's list.
    void addBidirectionalEdge(const Edge& edge)
    {
        Edge twinEdge = edge;
        twinEdge.u = edge.v;
        twinEdge.v = edge.u;

        addDirectedEdge(edge);
        addDirectedEdge(twinEdge);
    }

    int numberOfNodes;  // number of nodes
    std::unordered_set<IndexType> incidenceSet;
};

/// @brief Sparse incidence matrix graph is a hash-map based graph implementation.
///        IndexType    type that must be able to contain numberOfNodes^2
template<typename EdgeType, typename IndexType = int>
struct SparseIncidenceMatrixGraph
{
    SparseIncidenceMatrixGraph(int numberOfNodes)
        : numberOfNodes(numberOfNodes)
    {
        assert( static_cast<IndexType>(numberOfNodes * numberOfNodes) / numberOfNodes == numberOfNodes );
    }

    SparseIncidenceMatrixGraph(SparseIncidenceMatrixGraph&& other) noexcept
        : numberOfNodes(other.numberOfNodes)
        , edgeSet(std::move(other.edgeSet))
    {
    }

    bool hasDirectedEdge(int u, int v) const
    {
        return getDirectedEdge(u, v) != nullptr;
    }

    /// @brief Returns true if both edges u->v and v->u exist.
    bool hasBidirectionalEdge(int u, int v) const
    {
        return getBidirectionalEdge(u, v) != nullptr;
    }

    EdgeType* getDirectedEdge(int u, int v)
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);

        auto it = edgeSet.find(u * numberOfNodes + v);
        if (it == edgeSet.end())
            return nullptr;

        return &(*it);
    }

    /// @brief Returns edge u->v if both edges u->v and v->u exist.
    EdgeType* getBidirectionalEdge(int u, int v)
    {
        EdgeType* edge0 = getDirectedEdge(u, v);
        EdgeType* edge1 = getDirectedEdge(v, u);
        if (edge0 && edge1)
            return edge0;
        return nullptr;
    }

    /// @brief Removes edge u->v from the graph.
    ///        Returns false if edge was not part of the graph.
    bool removeDirectedEdge(int u, int v)
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);

        return edgeSet.erase(u * numberOfNodes + v) > 0;
    }

    /// @brief Erases edges u->v and v->u.
    ///        Does nothing and returns false is any of those edges doesn't exist.
    bool getBidirectionalEdge(int u, int v)
    {
        auto it0 = edgeSet.find(u * numberOfNodes + v);
        auto it1 = edgeSet.find(v * numberOfNodes + u);

        if (it0 == edgeSet.end())
            return false;
        if (it1 == edgeSet.end())
            return false;

        edgeSet.erase(it0);
        if (u != v)
            edgeSet.erase(it1);

        return return;
    }

    void addDirectedEdge(const EdgeType& edge)
    {
        assert(edge.u < numberOfNodes);
        assert(edge.v < numberOfNodes);
        edgeSet.insert(std::make_pair(edge.u * numberOfNodes + edge.v, edge));
    }

    /// @brief Returns index of the added edge in edge.u neighbour's list, and index of the added edge in edge.v neighbour's list.
    void addBidirectionalEdge(const EdgeType& edge)
    {
        EdgeType twinEdge = edge;
        twinEdge.u = edge.v;
        twinEdge.v = edge.u;

        addDirectedEdge(edge);
        addDirectedEdge(twinEdge);
    }

    int numberOfNodes;  // number of nodes
    std::unordered_map<IndexType, EdgeType> edgeSet; // edgeCountSet[u->v] = edge from u to v
};

#if 0

/// @brief Sparse incidence matrix is an O(1) set of pairs.
///        Implemented as a hash map of bools.
struct SparseIncidenceMatrix
{
    SparseIncidenceMatrix(int numberOfNodes)
        : numberOfNodes(numberOfNodes)
    {
    }

    SparseIncidenceMatrix(SparseIncidenceMatrix&& other)
        : numberOfNodes(other.numberOfNodes)
        , incidenceSet(std::move(other.incidenceSet))
    {
    }

    bool hasDirectedEdge(int u, int v)
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);
        return incidenceSet.count(std::make_pair(u, v)) > 0;
    }

    /// @brief Returns true if both edges u->v and v->u exist.
    bool hasBidirectionalEdge(int u, int v)
    {
        return hasDirectedEdge(u, v) && hasDirectedEdge(v, u);
    }

    void addDirectedEdge(const Edge& edge)
    {
        assert(edge.u < numberOfNodes);
        assert(edge.v < numberOfNodes);
        incidenceSet.insert(std::make_pair(edge.u, edge.v));
    }

    /// @brief Returns index of the added edge in edge.u neighbour's list, and index of the added edge in edge.v neighbour's list.
    void addBidirectionalEdge(const Edge& edge)
    {
        Edge twinEdge = edge;
        twinEdge.u = edge.v;
        twinEdge.v = edge.u;

        addDirectedEdge(edge);
        addDirectedEdge(twinEdge);
    }

    int numberOfNodes;  // number of nodes
    std::unordered_set< std::pair<int, int> > incidenceSet;
};

/// @brief Sparse incidence matrix graph is a hash-map based graph implementation.
///        IndexType    type that must be able to contain numberOfNodes^2
template<typename EdgeType>
struct SparseIncidenceMatrixGraph
{
    SparseIncidenceMatrixGraph(int numberOfNodes)
        : numberOfNodes(numberOfNodes)
    {
    }

    SparseIncidenceMatrixGraph(SparseIncidenceMatrixGraph&& other)
        : numberOfNodes(other.numberOfNodes)
        , edgeSet(std::move(other.edgeSet))
    {
    }

    bool hasDirectedEdge(int u, int v)
    {
        return getDirectedEdge(u, v) != nullptr;
    }

    /// @brief Returns true if both edges u->v and v->u exist.
    bool hasBidirectionalEdge(int u, int v)
    {
        return getBidirectionalEdge(u, v) != nullptr;
    }

    EdgeType* getDirectedEdge(int u, int v)
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);

        auto it = edgeSet.find(std::make_pair(u, v));
        if (it == edgeSet.end())
            return nullptr;

        return &(*it);
    }

    /// @brief Returns edge u->v if both edges u->v and v->u exist.
    EdgeType* getBidirectionalEdge(int u, int v)
    {
        EdgeType* edge0 = getDirectedEdge(u, v);
        EdgeType* edge1 = getDirectedEdge(v, u);
        if (edge0 && edge1)
            return edge0;
        return nullptr;
    }

    /// @brief Removes edge u->v from the graph.
    ///        Returns false if edge was not part of the graph.
    bool removeDirectedEdge(int u, int v)
    {
        assert(u < numberOfNodes);
        assert(v < numberOfNodes);

        return edgeSet.erase(std::make_pair(u, v)) > 0;
    }

    /// @brief Erases edges u->v and v->u.
    ///        Does nothing and returns false is any of those edges doesn't exist.
    bool getBidirectionalEdge(int u, int v)
    {
        auto it0 = edgeSet.find(std::make_pair(u, v));
        auto it1 = edgeSet.find(std::make_pair(v, u));

        if (it0 == edgeSet.end())
            return false;
        if (it1 == edgeSet.end())
            return false;

        edgeSet.erase(it0);
        if (u != v)
            edgeSet.erase(it1);

        return return;
    }

    void addDirectedEdge(const EdgeType& edge)
    {
        assert(edge.u < numberOfNodes);
        assert(edge.v < numberOfNodes);
        edgeSet.insert(std::make_pair(std::make_pair(edge.u, edge.v), edge));
    }

    /// @brief Returns index of the added edge in edge.u neighbour's list, and index of the added edge in edge.v neighbour's list.
    void addBidirectionalEdge(const Edge& edge)
    {
        Edge twinEdge = edge;
        twinEdge.u = edge.v;
        twinEdge.v = edge.u;

        addDirectedEdge(edge);
        addDirectedEdge(twinEdge);
    }

    int numberOfNodes;  // number of nodes
    std::unordered_map< std::pair<int, int>, EdgeType > edgeSet; // edgeCountSet[u->v] = edge from u to v
};

#endif

#if 0
/// @brief Graph is represented as:
///        - number of nodes,
///        - incidence matrix of vertices (directed edges).
template<typename EdgeType>
struct IncidenceMatrixGraph
{
    IncidenceMatrixGraph(int numberOfNodes)
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
    std::vector< std::vector<EdgeType> > incidenceMatrix; // incidenceMatrix[from][to]
};
#endif

#endif // IncidenceMatrixGraph_H
