
#include "FindUnion.h"

#include <cstdio>
#include <iostream>

void FindUnion_main()
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
    freopen("FindUnion.txt", "r", stdin);
#endif

    int N; // number of elements
    int M; // number of join operations

    std::cin >> N;
    std::cin >> M;

    FindUnion FindUnion(N);

    for (int i = 0; i < M; ++i)
    {
        int element0, element1;
        std::cin >> element0 >> element1;

        FindUnion.join(element0 - 1, element1 - 1);
    }

    // split elements into separate groups
    std::vector< std::vector<int> > m_groups(N);
    for (int i = 0; i < N; ++i)
    {
        int root = FindUnion.findRoot(i);
        m_groups[root].push_back(i);
    }

    // print all groups
    for (int i = 0; i < N; ++i)
    {
        if (m_groups[i].empty())
            continue;

        for (auto element : m_groups[i])
        {
            std::cout << element + 1 << " ";
        }
        std::cout << std::endl;
    }
}

