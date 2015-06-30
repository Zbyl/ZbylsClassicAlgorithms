
#include "MST-Kruskal.h"
#include "MST-Prim.h"

#include <cstdio>
#include <iostream>

void MST_main()
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
    freopen("mst.txt", "r", stdin);
#endif

    int N; // node count
    int M; // edge count

    std::cin >> N;
    std::cin >> M;

    EdgeListGraph< WeightedEdge<int> > edgeListGraph(N);
    NeighbourListGraph< WeightedEdge<int> > neighbourListGraph(N);

    for (int i = 0; i < M; ++i)
    {
        int p, k, w;
        std::cin >> p >> k >> w;
        p--;
        k--;

        edgeListGraph.addEdge( WeightedEdge<int>(p, k, w) );
        neighbourListGraph.addBidirectionalEdge( WeightedEdge<int>(p, k, w) );
    }

    int kruskalCost = mstKruskalCost(edgeListGraph);
    int primCost = mstPrimCost(neighbourListGraph);

    std::cout << "Cost of MST using Kruskal: " << kruskalCost << std::endl;
    std::cout << "Cost of MST using Prim: " << primCost << std::endl;
    assert(kruskalCost == primCost);
}

