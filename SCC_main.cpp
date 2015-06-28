
#include "SCC-Tarjan.h"
#include "SCC-Kosaraju.h"

#include <cstdio>
#include <iostream>

void SCC_main()
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
    freopen("scc.txt", "r", stdin);
#endif

    int N; // node count
    int M; // edge count

    std::cin >> N;
    std::cin >> M;

    NeighbourListGraph< WeightedEdge<int> > neighbourListGraph(N);

    for (int i = 0; i < M; ++i)
    {
        int p, k;
        std::cin >> p >> k;
        p--;
        k--;

        neighbourListGraph.addDirectedEdge( WeightedEdge<int>(p, k, 0) );
    }

    std::vector< std::vector<int> > sccGroupsT = SCCTarjan<int>(neighbourListGraph);
    std::vector< std::vector<int> > sccGroupsK = SCCKosaraju<int>(neighbourListGraph);

    std::cout << "Strongly connected components Tarjan:" << std::endl;
    for (unsigned int i = 0; i < sccGroupsT.size(); ++i)
    {
        for (auto element : sccGroupsT[i])
        {
            std::cout << element + 1 << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Strongly connected components Kosaraju:" << std::endl;
    for (unsigned int i = 0; i < sccGroupsK.size(); ++i)
    {
        for (auto element : sccGroupsK[i])
        {
            std::cout << element + 1 << " ";
        }
        std::cout << std::endl;
    }
}

