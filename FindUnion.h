/// @brief  Find-Union implementation
/// @author z.skowron
/// @see    http://wazniak.mimuw.edu.pl/index.php?title=Algorytmy_i_struktury_danych/Find-Union

#ifndef FindUnion_H
#define FindUnion_H

#include <vector>

class FindUnion
{
    struct FindUnionNode
    {
        /// @brief This constructs a root node.
        FindUnionNode(int id)
            : m_parent(id)
            , m_rank(0)
        {}

        int m_parent;   ///< Identifier of the set representant.
        int m_rank;     ///< Upper bound of the height of the tree below us (single node has rank 0). This would be the height of the tree below us, if we weren't doing path compression.
    };

private:
    std::vector<FindUnionNode> m_nodes;  ///< Collection of nodes.

public:
    FindUnion(int numberOfElements)
    {
        m_nodes.reserve(numberOfElements);
        for (int i = 0; i < numberOfElements; ++i)
        {
            m_nodes.push_back(FindUnionNode(i));
        }
    }

    int findRoot(int element)
    {
        FindUnionNode& node = m_nodes[element];
        if (node.m_parent == element)
            return element;   // we are the root

        // compress the path (we don't update rank)
        int root = findRoot(node.m_parent);
        node.m_parent = root;
        return root;
    }

    void join(int element0, int element1)
    {
        int root0 = findRoot(element0);
        int root1 = findRoot(element1);

        // if we already are in the same set - nothing to do.
        if (root0 == root1)
            return;

        // if the ranks are equal - bind one tree to the other, and bump the rank
        if (m_nodes[root0].m_rank == m_nodes[root1].m_rank)
        {
            m_nodes[root1].m_parent = root0;
            m_nodes[root0].m_rank++;
        }

        // if the ranks are not equal - bind tree with lower rank to one with higher rank
        if (m_nodes[root0].m_rank > m_nodes[root1].m_rank)
        {
            m_nodes[root1].m_parent = root0;
        }
        else
        {
            m_nodes[root0].m_parent = root1;
        }
    }
};

#endif // FindUnion_H
