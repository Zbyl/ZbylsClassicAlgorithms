
#include "dfs-bfs.h"
#include "TopSort.h"

#include <cstdio>
#include <iostream>

void TopSort_main()
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
    //freopen("H:\\TopCoder\\topsorting\\resourcesIn\\topSort1.in", "r", stdin);
    freopen("topsort.txt", "r", stdin);
#endif

    int N; // node count
    int M; // edge count

    std::cin >> N;
    std::cin >> M;

    NeighbourListGraph<int> neighbourListGraph(N);

    for (int i = 0; i < M; ++i)
    {
        int p, k;
        std::cin >> p >> k;
        p--;
        k--;

        neighbourListGraph.addDirectedEdge(p, k, 0);
    }

    // edge from p to k means that p must be done before k

    std::vector<int> sorted;
    bool success = topSort(neighbourListGraph, sorted);
    std::vector<int> sorted2 = dfs(neighbourListGraph, false); // if graph cannot be sorted, it returns result anyway!

    std::cout << "Topologically sorted:" << std::endl;
    if (!success)
    {
        std::cout << " Can't sort!" << std::endl;
    }
    else
    {
        for (auto element : sorted)
        {
            std::cout << element + 1 << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Topologically sorted:" << std::endl;
    for (auto element : sorted2)
    {
        std::cout << element + 1 << " ";
    }
    std::cout << std::endl;
}
