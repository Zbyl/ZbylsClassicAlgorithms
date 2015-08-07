/// @brief  Computes shortest paths using naive algorithm.
///         This algorithm will find shortest paths from all nodes to all other reachable nodes in the graph,
//          provided that there are no negative cycles in the graph (doesn't check it).
/// @author z.skowron

#ifndef BellmanFord_H
#define BellmanFord_H

// Pseudo kod algorytmu Bellmana-Forda:
// http://wazniak.mimuw.edu.pl/index.php?title=Zaawansowane_algorytmy_i_struktury_danych/Wyk%C5%82ad_6#Algorytm
//
// ODLEG£OŒCI(W)
//  D = W
//  m = 1
//  while n - 1 > m do
//  begin
//    D = MNO¯ENIE - ODLEG£OŒCI(D, D)
//    m = 2m
//  end
//  return D
//

#include <vector>
#include <queue>
#include <functional>
#include <algorithm>

#include "ZAssert.h"
#include "IncidenceMatrixGraph.h"
#include "NeighbourListGraph.h"
#include "EdgeListGraph.h"

template<typename C = int>
WeightMatrix<C> weightMatrixFromGraph(const NeighbourListGraph< WeightedEdge<C> >& graph)
{
    WeightMatrix<C> weightMatrix(graph.numberOfNodes);
    for (auto neighbours : graph.neighbours)
    {
        for (auto edge : neighbours)
        {
            assert(edge.u != edge.v); // if we have edge u->u, then we should put zero in the matrix anyway
            weightMatrix.addDirectedEdge(edge);
        }
    }
    return weightMatrix;
}

template<typename C = int>
WeightMatrix<C> weightMatrixFromGraph(const EdgeListGraph< WeightedEdge<C> >& graph)
{
    WeightMatrix<C> weightMatrix(graph.numberOfNodes);
    for (auto edge : graph.edges)
    {
        assert(edge.u != edge.v); // if we have edge u->u, then we should put zero in the matrix anyway
        weightMatrix.addDirectedEdge(edge);
    }
    return weightMatrix;
}

/// @brief Computes distances to all graph nodes from all other nodes.
///        Cost: O(|V|^3 * log |V|)
///        Use Floyd-Warshall for O(|V|^3) time.
template<typename C = int>
WeightMatrix<C> allDistancesNaive(const WeightMatrix<C>& weightMatrix)
{
    WeightMatrix<C> result = weightMatrix;
    WeightMatrix<C> tempResult(weightMatrix.numberOfNodes);
    int pathLengths = 1;

    while (pathLengths < weightMatrix.numberOfNodes - 1)
    {
        WeightMatrix<C>::distanceMultiply(result, result, tempResult);
        std::swap(result, tempResult);
        pathLengths = 2 * pathLengths;
    }

    return result;
}

/// @brief Computes distances to all graph nodes from all other nodes using Floyd-Warshall algorithm.
///        Cost: O(|V|^3)
template<typename C = int>
WeightMatrix<C> allDistancesFloydWarshall(const WeightMatrix<C>& weightMatrix)
{
    WeightMatrix<C> result(weightMatrix);  // distances through no nodes
    WeightMatrix<C> previousResult(weightMatrix.numberOfNodes);

    for (int k = 0; k < weightMatrix.numberOfNodes; ++k)
    {
        std::swap(result, previousResult);
        for (int i = 0; i < weightMatrix.numberOfNodes; ++i)
        {
            for (int j = 0; j < weightMatrix.numberOfNodes; ++j)
            {
                C minVal = previousResult.getWeight(i, j); // dist through nodes 0 to k-1

                C distUntilK = previousResult.getWeight(i, k);   // dist through nodes 0 to k
                C distAfterK = previousResult.getWeight(k, j);   // dist through nodes 0 to k
                if ( (distUntilK < std::numeric_limits<C>::max()) && (distAfterK < std::numeric_limits<C>::max()) )
                    minVal = std::min(minVal, distUntilK + distAfterK);

                result.setWeight(i, j, minVal);
            }
        }
    }

    return result;
}

#endif // BellmanFord_H
