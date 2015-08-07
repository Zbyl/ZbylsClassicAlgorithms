/// @brief  Implementation of Topology Sort
/// @author z.skowron

#ifndef Heap_H
#define Heap_H

#include <vector>
#include <utility>

#include "ZAssert.h"

/// @brief Fixes heap invariant in a sub-heap whose root is fixupIndex, assuming that only this root violates the invariant.
/// @param fixupIndex	may be larger than data.size(); function is a no-op then
/// @see   https://en.wikipedia.org/wiki/Heapsort
///
///        *
///    x       *
///  o   o   *   *
/// o o o o * . . .
///
/// x - root of the sub-heap (fixupIndex)
/// o - nodes in the sub-heap; they might be modified when pushing x down to satisfy the heap invariant
///
template<typename T>
void fix_heap_down(std::vector<T>& data, int fixupIndex)
{
	int dataSize = static_cast<int>(data.size());

	int currentIdx = fixupIndex;
	while (true)
	{
		int idxOfSmallest = currentIdx;
		int idxOfChild0 = currentIdx * 2 + 1;
		int idxOfChild1 = currentIdx * 2 + 2;
		
		if ( (idxOfChild0 < dataSize) && (data[idxOfChild0] < data[idxOfSmallest]) )
		{
			idxOfSmallest = idxOfChild0;
		}
		if ( (idxOfChild1 < dataSize) && (data[idxOfChild1] < data[idxOfSmallest]) )
		{
			idxOfSmallest = idxOfChild1;
		}
		
		// if the smallest element is parent, then nothing more to do
		if (idxOfSmallest == currentIdx)
			break;
		
		// push parent down, exchanging it with smallest child
		std::swap(data[currentIdx], data[idxOfSmallest]);
		
		// now go another loop to push the new value even further down (if necessary)
		currentIdx = idxOfSmallest;
	}
}

/// @brief Fixes heap invariant in a heap where only  fixupIndex violates the invariant by possibly being smaller than it's parent.
/// @see   https://en.wikipedia.org/wiki/Heapsort
///
///        *
///    *       o
///  o   *   o   o
/// o o o x o . . .
///
/// x - violating node (fixupIndex)
/// * - nodes in the heap that might be modified when pushing x up to satisfy the heap invariant
///
template<typename T>
void fix_heap_up(std::vector<T>& data, int fixupIndex)
{
	int dataSize = static_cast<int>(data.size());

	int currentIdx = fixupIndex;
	while (currentIdx > 0)
	{
		int idxOfParent = (currentIdx - 1) / 2;
		
		// if heap invariant is ok, then end
		if (data[currentIdx] > data[idxOfParent])
		{
			break;
		}
		
		// fix the invariant by exchanging child with parent
		std::swap(data[currentIdx], data[idxOfParent]);
		
		// now go another loop to push the new value even further up (if necessary)
		currentIdx = idxOfParent;
	}
}

/// @brief Makes heap on a vector.
/// @see   https://en.wikipedia.org/wiki/Heapsort
template<typename T>
void make_heap(std::vector<T>& data)
{
	if (data.empty())
		return;
	
	int dataSize = static_cast<int>(data.size());
	
	/// We have to perform fixup from the parent of last element to first element.
	///        o
	///    o       o
	///  o   o   o   x
	/// x x x x x . . .
	///
	/// o - elements we need to fix-up
	///
	/// If we'll exchange parent with one of the children, we need to try to push this value down to leafs.
	///
	
	for (int fixupIndex = (dataSize - 1) / 2; fixupIndex >= 0; fixupIndex--)
	{
		fix_heap_down(data, fixupIndex);
	}
}

/// @brief Pops smallest element from the heap.
/// @note  Heap must not be empty.
/// @see   https://en.wikipedia.org/wiki/Heapsort
template<typename T>
T pop_heap(std::vector<T>& data)
{
	assert(!data.empty());

	T smallest = data.front();
	std::swap(data.front(), data.back());
	data.pop_back();
	
	fix_heap_down(data, 0);

    return smallest;
}

/// @brief Inserts a new element into the heap.
/// @see   https://en.wikipedia.org/wiki/Heapsort
template<typename T>
void insert_heap(std::vector<T>& data, T value)
{
	data.push_back(value);
	
	int dataSize = static_cast<int>(data.size());
	fix_heap_up(data, dataSize - 1);
}

/// @brief Implements heap sort (@todo: can be easily done in-place!).
/// @see   https://en.wikipedia.org/wiki/Heapsort
template<typename T>
void heap_sort(std::vector<T>& data)
{
	make_heap(data);
	
	std::vector<T> sorted;
	while (!data.empty())
	{
		T smallest = pop_heap(data);
		sorted.push_back(smallest);
	}
	
	std::swap(sorted, data);
}

#endif // Heap_H
