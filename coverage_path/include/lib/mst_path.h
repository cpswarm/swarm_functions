#ifndef MST_PATH_H
#define MST_PATH_H

#include <deque>
#include <valarray>
#include <unordered_set>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
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
     * @brief Generate all way points for the path.
     * @param start The current position of the CPS.
     */
    void generate_path (geometry_msgs::Point);

    /**
     * @brief Get the graph generated from the grid map.
     * @return An array of poses representing the vertices of the grid.
     */
    geometry_msgs::PoseArray get_grid ();

    /**
     * @brief Get the complete path.
     * @return The path as vector of poses.
     */
    nav_msgs::Path get_path ();

    /**
     * @brief Get the current waypoint and possibly select next waypoint, if close enough.
     * @param position The current position of the CPS.
     * @param tolerance The distance to the current waypoint below which the next waypoint is selected.
     * @return A waypoint for the CPS to navigate to.
     */
    geometry_msgs::Point get_waypoint (geometry_msgs::Point position, double tolerance);

    /**
     * @brief Initialize the internal graph structure that represents the area division.
     * @param graph  gridmap The grid map for which the paths are generated.
     * @param connect4 Whether only the von Neumann neighborhood is considered. Default true.
     */
    void initialize_graph (nav_msgs::OccupancyGrid gridmap, bool connect4 = true);

    /**
     * @brief Remove edges of the graph that overlap with the tree.
     * @param mst The edges that define the tree.
     */
    void initialize_tree (vector<edge> mst);

    /**
     * @brief Check whether the current waypoint index is valid.
     * @return True, if the waypoint index is within the limits of the path, false otherwise.
     */
    bool valid ();

private:
    /**
     * @brief Add an edge to the tree graph.
     * @param from The starting vertex.
     * @param to The ending vertex.
     * @param cost The cost of the edge.
     */
    void add_edge(int from, int to, int cost);

    /**
     * @brief Calculate the distance between two points.
     * @param p1 The first point.
     * @param p2 The second point.
     * @return The straight line distance between both points.
     */
    double dist (geometry_msgs::Point p1, geometry_msgs::Point p2);

    /**
     * @brief Get the current way point.
     * @param offset The index offset to the current waypoint.
     * @return The way point, if a valid offset was given. An empty point otherwise.
     */
    geometry_msgs::Point get_wp (int offset=0);

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
     * @brief Priority queue of edge objects sorted by cost.
     */
    unordered_set<edge, hash_edge> edges;

    /**
     * @brief The sets of connected vertices.
     */
    vector<unordered_set<int>> nodes;

    /**
     * @brief The grid map that needs to be covered by the MST path.
     */
    nav_msgs::OccupancyGrid map;

    /**
     * @brief The current way point.
     */
    int wp;
};

#endif // MST_PATH_H
