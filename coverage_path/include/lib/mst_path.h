#ifndef MST_PATH_H
#define MST_PATH_H

#include <deque>
#include <valarray>
#include <unordered_set>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include "lib/edge.h"

using namespace std;
using namespace ros;

/**
 * @brief An class to compute the coverage path based on a minimum-spanning-tree (MST).
 */
class mst_path
{
public:
    /**
     * @brief Constructor.
     */
    mst_path ();

    /**
     * @brief Get the current way point of the path.
     * @return The current way point.
     */
    geometry_msgs::Point current_wp ();

    /**
     * @brief Generate all way points for the path.
     * @param start The current position of the CPS.
     */
    void generate_path (geometry_msgs::Point);

    /**
     * @brief Get the complete path.
     * @return The path as vector of poses.
     */
    nav_msgs::Path get_path ();

    /**
     * @brief Initialize the internal graph structure that represents the area division.
     * @param graph The graph array that defines the area division.
     * @param connect4 Whether only the von Neumann neighborhood is considered. Default true.
     */
    void initialize_graph (valarray<bool> graph, bool connect4 = true);

    /**
     * @brief Initialize private variables according to grid map.
     * @param gridmap The grid map for which the paths are generated
     */
    void initialize_map(nav_msgs::OccupancyGrid gridmap);

    /**
     * @brief Initialize the internal tree structure.
     * @param mst The graph array that defines vertices of the tree.
     */
    void initialize_tree (vector<edge> mst);

    /**
     * @brief Make the next way point the current one and return it.
     * @returns The next way point.
     */
    geometry_msgs::Point next_wp ();

    /**
     * @brief Get the previous way point.
     * @return The previous way point.
     */
    geometry_msgs::Point previous_wp ();

    /**
     * @brief Remove redundant edges in the internal graph.
     */
    void remove_edges ();

    /**
     * @brief Make the next way point the current one.
     */
    void step ();

private:
    /**
     * @brief Add an edge to the tree graph.
     * @param from The starting vertex.
     * @param to The ending vertex.
     * @param cost The cost of the edge.
     */
    void add_edge(int from, int to, int cost);

    /**
     * @brief Get the way point corresponding to a given index.
     * @param idx The index of the way point.
     * @return The way point, if a valid index was given. An empty point otherwise.
     */
    geometry_msgs::Point get_wp (int idx);

    /**
     * @brief Remove an edge from the tree graph.
     * @param e The edge to remove.
     */
    void remove_edge (edge e);

    /**
     * @brief The boustrophedon path.
     */
    deque<geometry_msgs::Point> path;

    /**
     * @brief The minimum spanning tree that defines the path.
     */
    vector<edge> mst;

    /**
     * @brief Priority queue of edge objects sorted by cost.
     */
    unordered_set<edge, hash_edge, compare_edge> edges;

    /**
     * @brief The sets of connected vertices.
     */
    vector<unordered_set<int>> nodes;

    /**
     * @brief Height of the grid map in cells.
     */
    int rows;

    /**
     * @brief Width of the grid map in cells.
     */
    int cols;

    /**
     * @brief Resolution of the grid map in meter / cell.
     */
    double res;

    /**
     * @brief Origin of cell (0,0) of the grid map.
     */
    geometry_msgs::Point origin;

    /**
     * @brief The current way point.
     */
    int wp;
};

#endif // MST_PATH_H
