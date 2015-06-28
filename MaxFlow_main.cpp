
#include "MaxFlow-Dinic.h"

#include <cstdio>
#include <iostream>

void MaxFlow_main()
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
    freopen("maxflow2.txt", "r", stdin);
#endif

    int N; // node count
    int M; // edge count
    int S; // source node
    int T; // sink node

    std::cin >> N;
    std::cin >> M;
    std::cin >> S;
    std::cin >> T;
    S--;
    T--;

    NeighbourListGraph< MaxFlowEdge<int> > neighbourListGraph(N);

    for (int i = 0; i < M; ++i)
    {
        int p, k, w;
        std::cin >> p >> k >> w;
        p--;
        k--;

        neighbourListGraph.addBidirectionalEdge( MaxFlowEdge<int>(p, k, w) );
    }

    int maxFlow = maxFlowDinic(neighbourListGraph, S, T);

    std::cout << "Max flow using Dinic's algorithm: " << maxFlow << std::endl;
}

