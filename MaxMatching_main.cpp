
#include "MaxMatching-HopcroftKarp.h"
#include "MaxFlow-Dinic.h"

#include <cstdio>
#include <iostream>

void MaxMatching_main()
{
    /*
    The freopen function below opens input.txt in read only mode and
    sets your standard input to work with the opened file.
    When you test your code with the sample data, you can use the function
    below to read in from the sample data file instead of the standard input.
    So. you can uncomment the following line for your local test. But you
    have to comment the following line when you submit for your scores.
    */

#ifdef ZBYL
    freopen("maxmatching.txt", "r", stdin);
#endif

    int V1; // V1 node count
    int V2; // V2 node count
    int M; // edge count

    std::cin >> V1;
    std::cin >> V2;
    std::cin >> M;

    NeighbourListGraph< MaxMatchingEdge > neighbourListGraph(V1 + V2);
    NeighbourListGraph< MaxMatchingEdge > neighbourListGraph2(V1 + V2);
    NeighbourListGraph< MaxFlowEdge<int> > neighbourListGraphFlow(V1 + V2 + 2);

    for (int i = 0; i < M; ++i)
    {
        int p, k;
        std::cin >> p >> k;
        p--;
        k--;

        neighbourListGraph.addBidirectionalEdge( MaxMatchingEdge(p, k) );
        neighbourListGraph2.addBidirectionalEdge( MaxMatchingEdge(p, k) );
        neighbourListGraphFlow.addBidirectionalEdge( MaxFlowEdge<int>(p, k, 1) );
    }

    int S = V1 + V2;
    int T = V1 + V2 + 1;
    for (int i = 0; i < V1; ++i)
    {
        neighbourListGraphFlow.addBidirectionalEdge( MaxFlowEdge<int>(S, i, 1) );
    }
    for (int i = V1; i < V1 + V2; ++i)
    {
        neighbourListGraphFlow.addBidirectionalEdge( MaxFlowEdge<int>(i, T, 1) );
    }

    std::vector<bool> coloring1(V1 + V2);
    for (int i = 0; i < V1; ++ i)
        coloring1[i] = true;

    std::vector<bool> coloring2(V1 + V2);
    bool bipartite = isBipartite(neighbourListGraph2, coloring2);
    assert(bipartite);

    std::vector<bool> coloring3(V1 + V2 + 2);
    bool bipartiteFlow = isBipartite(neighbourListGraphFlow, coloring3);
    assert(bipartiteFlow);

    std::vector<bool> nodesMatched1(V1 + V2);
    int maxMatching1 = maxMatchingHK(neighbourListGraph, coloring1, nodesMatched1);
    std::vector<bool> nodesMatched2(V1 + V2);
    int maxMatching2 = maxMatchingHK(neighbourListGraph2, coloring2, nodesMatched2);
    int maxFlow = maxFlowDinic(neighbourListGraphFlow, S, T);

    assert(maxMatching1 == maxMatching2);
    assert(maxMatching1 == maxFlow);

    std::cout << "Max matching using Hopcroft-Karps's algorithm: " << maxMatching1 << std::endl;
}

