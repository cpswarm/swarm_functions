#ifndef SPANNING_TREE_H
#define SPANNING_TREE_H

#include <queue>
#include <unordered_set>
#include <valarray>
#include "lib/edge.h"

using namespace std;

/**
 * @brief A class that generates a minimum-spanning-tree of a graph (MST).
 */
class spanning_tree
{
public:
    /**
     * @brief Constructor.
     */
    spanning_tree ();

    /**
     * @brief Get the edges of the MST.
     * @return A vector with all edges of the MST.
     */
    vector<edge> get_mst_edges();

    /**
     * @brief Initialize the internal tree structure from a given graph.
     * @param rows The number of rows in the array.
     * @param cols The number of columns in the array.
     * @param graph The graph array that defines vertices of the tree.
     * @param connect4 Whether only the von Neumann neighborhood is considered. Default true.
     */
    void initialize_graph(int rows, int cols, valarray<bool> graph, bool connect4 = true);

    /**
     * @brief Generate the MST using Kruskal's algorithm.
     */
    void construct();

private:
    /**
     * @brief Add an edge to the tree.
     * @param from The starting vertex.
     * @param to The ending vertex.
     * @param cost The cost of the edge.
     */
    void add_edge(int from, int to, int cost);

    /**
     * @brief Check whether two vertices are in different sets.
     * @param a The first vertex.
     * @param b The second vertex.
     * @return True if the vertices are in different connected components, i.e., the set for vertex a is different from that for vertex b.
     */
     bool different_sets(int a, int b);

    /**
     * @brief The sets of connected vertices.
     */
    vector<unordered_set<int>> nodes;

    /**
     * @brief Priority queue of edge objects sorted by cost.
     */
    priority_queue<edge, vector<edge>, compare_edge> edges;

    /**
     * @brief Edges in Minimal-Spanning Tree.
     */
    vector<edge> new_edges;

    /**
     * @brief Maximum possible number of vertices in the tree.
     */
    int max_nodes;
};

#endif // SPANNING_TREE_H
