
#include "Heap.h"

#include <cstdio>
#include <iostream>

void Heap_main()
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
    freopen("heapsort.txt", "r", stdin);
#endif

    int N; // number of elements to load first into heap
    int M; // number of elements to insert into a ready heap

    std::cin >> N;
    std::cin >> M;

	std::vector<int> data;
	std::vector<int> data2;

    for (int i = 0; i < N; ++i)
    {
        int v;
        std::cin >> v;
		
		data.push_back(v);
		data2.push_back(v);
    }

	make_heap(data);

    for (int i = 0; i < M; ++i)
    {
        int v;
        std::cin >> v;
		
		insert_heap(data, v);
		data2.push_back(v);
    }
	
	heap_sort(data2);

    std::cout << "Heap sorted in bulk:" << std::endl;
	for (auto element : data2)
	{
		std::cout << element << " ";
	}
	std::cout << std::endl;

    std::cout << "Heap sorted not in bulk:" << std::endl;
	while (!data.empty())
    {
		int smallest = pop_heap(data);
        std::cout << smallest << " ";
    }
    std::cout << std::endl;
}
