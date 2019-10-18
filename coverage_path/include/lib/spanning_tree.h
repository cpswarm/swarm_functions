#ifndef SPANNING_TREE_H
#define SPANNING_TREE_H

#include <queue>
#include <unordered_set>
#include <valarray>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include "lib/edge.h"

using namespace std;
using namespace ros;

/**
 * @brief A class that generates a minimum-spanning-tree (MST) graph for a given grid map.
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
    vector<edge> get_mst_edges ();

    /**
     * @brief Get the generated MST for visualization.
     * @return An array of poses that represent the vertices of the tree.
     */
    geometry_msgs::PoseArray get_tree ();

    /**
     * @brief Initialize the internal tree structure from a given grid map.
     * @param gridmap The grid map that needs to be covered by the tree.
     * @param connect4 Whether only the von Neumann neighborhood is considered. Default true.
     */
    void initialize_graph (nav_msgs::OccupancyGrid gridmap, bool connect4 = true);

    /**
     * @brief Generate the MST using Kruskal's algorithm.
     */
    void construct ();

private:
    /**
     * @brief Add an edge to the tree.
     * @param from The starting vertex.
     * @param to The ending vertex.
     * @param cost The cost of the edge.
     */
    void add_edge (int from, int to, int cost);

    /**
     * @brief Check whether two vertices are in different sets.
     * @param a The first vertex.
     * @param b The second vertex.
     * @return True if the vertices are in different connected components, i.e., the set for vertex a is different from that for vertex b.
     */
     bool different_sets (int a, int b);

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
    vector<edge> mst_edges;

    /**
     * @brief The grid map that needs to be covered by the MST.
     */
    nav_msgs::OccupancyGrid map;
};

#endif // SPANNING_TREE_H
