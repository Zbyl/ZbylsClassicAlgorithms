/// @brief  Max Flow - Dinic's algorithm implementation
/// @author z.skowron

#ifndef MaxFlowDinic_H
#define MaxFlowDinic_H

// Pseudo kod algorytmu Dinica:
//
//    Graf reprezentuj¹cy sieæ ma krawêdzie dwukierunkowe (o tej samej przepustowoœci).
//    Przep³yw w krawêdzi u->v bêdzie zawsze równy przeciwieñstwu przep³ywu w krawêdzi v->u: f(u,v) = -f(v,u)
//
// 1. PrzejdŸ BFS'em sieæ (tylko po krawêdziach nie nasyconych) i wyznacz odleg³oœci od Ÿród³a.
// 2. IdŸ po wszystkich najkrótszych œcie¿kach (DFS taki, ¿e odleg³oœci od Ÿród³a w kolejnych wêz³ach rosn¹ o 1) i dodawaj ich przepustowoœæ do przep³ywu wêz³ów œcie¿ki.
//    (W ten sposób znajdziemy przep³yw blokuj¹cy z opisu algorytmu Dinica.)
//
//    while (find shortest path p from s to t)
//        for each node in p:
//            node.flow += maxFlow(p)
//
// 3. Powtórz wszystko, a¿ nie da siê znaleŸæ œcie¿ki od s do t.

#include <vector>
#include <queue>
#include <functional>
#include <limits>

#include "NeighbourListGraph.h"
#include "ZAssert.h"

/// @brief This method will find distances of all edges from the start node.
/// @note  Returns false if end node is not reachable.
template<typename C = int>
bool maxFlowDinicBFS(const NeighbourListGraph< MaxFlowEdge<C> >& graph, int startNode, int endNode, std::vector<int>& distances)
{
    assert(distances.size() == graph.numberOfNodes);
    std::fill(distances.begin(), distances.end(), -1);

    std::deque<int> queue; // BFS queue (note - improve performance by preallocating one array for all BFS runs)

    distances[startNode] = 0;
    queue.push_back(startNode);

    while (!queue.empty())
    {
        int node = queue.front();
        queue.pop_front();

        for (size_t i = 0; i < graph.neighbours[node].size(); ++i)
        {
            const MaxFlowEdge<C>& edge = graph.neighbours[node][i];

            // if edge is saturated - skip it
            if (edge.flow >= edge.capacity)
            {
                assert(edge.flow == edge.capacity); // we will never have flows bigger than capacity
                continue;
            }

            if (distances[edge.v] == -1)
            {
                distances[edge.v] = distances[node] + 1;
                queue.push_back(edge.v);
            }
        }
    }

    return distances[endNode] != -1;
}

/// @brief This method will go through all shortest paths and saturate the flow using them.
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

        int flowLeft = edge.capacity - edge.flow; // edge flow is in the range [-capacity, capacity]
        // if edge is saturated - skip it
        if (flowLeft <= 0)
        {
            assert(flowLeft == 0); // flow left will never be less than zero
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

#endif // MaxFlowDinic_H
