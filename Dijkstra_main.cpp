
#include "Dijkstra.h"

#include <cstdio>
#include <iostream>

void Dijkstra_main()
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
    freopen("dijkstra.txt", "r", stdin);
#endif

    int S; // start node
    int E; // end node

    std::cin >> S;
    std::cin >> E;
    S--;
    E--;

    int N; // node count
    int M; // edge count

    std::cin >> N;
    std::cin >> M;

    NeighbourListGraph<int> neighbourListGraph(N);

    for (int i = 0; i < M; ++i)
    {
        int p, k, w;
        std::cin >> p >> k >> w;
        p--;
        k--;

        neighbourListGraph.addDirectedEdge(p, k, w);
    }

    std::vector<int> shortestPath;
    int distance = dijkstraPath(neighbourListGraph, S, E, shortestPath);
    int distance2 = dijkstraCost(neighbourListGraph, S, E);
    assert(distance == distance2);

    std::cout << "Distance: " << distance << std::endl;
    std::cout << "Shortest path: ";
    for (auto node : shortestPath)
    {
        std::cout << node + 1 << " ";
    }
    std::cout << std::endl;
}

