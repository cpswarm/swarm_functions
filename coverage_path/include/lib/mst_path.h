#ifndef MST_PATH_H
#define MST_PATH_H

#include <deque>
#include <valarray>
#include <unordered_set>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
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
     * @return Whether the generation was successful.
     */
    bool generate_path (geometry_msgs::Point start);

    /**
     * @brief Get the complete path.
     * @return The path as vector of poses.
     */
    nav_msgs::Path get_path ();

    /**
     * @brief Get the current waypoint and possibly select next waypoint, if close enough.
     * @param position The current position of the CPS.
     * @param tolerance The distance to the current waypoint below which the next waypoint is selected.
     * @return A waypoint for the CPS to navigate to. An empty point if the waypoint is outside of the path.
     */
    geometry_msgs::Point get_waypoint (geometry_msgs::Point position, double tolerance);

    /**
     * @brief Initialize the internal graph structure that represents the area to cover.
     * @param graph  gridmap The grid map for which the paths are generated.
     * @param vertical Whether the sweeping pattern is vertical or horizontal. Default horizontal.
     * @param connect4 Whether only the von Neumann neighborhood is considered. Default true.
     */
    void initialize_graph (nav_msgs::OccupancyGrid gridmap, bool vertical = false, bool connect4 = true);

    /**
     * @brief Initialize properties of the area to be covered.
     * @param origin Coordinate of the bottom left point of the area.
     * @param rotation The angle by which the map has been rotated.
     * @param width The width of the area.
     * @param height The height of the area.
     */
    void initialize_map (geometry_msgs::Point origin, double rotation, double width, double height);

    /**
     * @brief Remove edges of the graph that overlap with the tree.
     * @param mst The edges that define the tree.
     */
    void initialize_tree (set<edge> mst);

    /**
     * @brief Remove waypoints that are within straight line segments of the path. Only keep turning points of the path.
     */
    void reduce ();

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
    void add_edge(int from, int to, double cost);

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
     * @brief Convert an grid map index to a real world position.
     * @param index The index of the grid cell in the map.
     * @return The coordinates of the grid cell in meters.
     */
    geometry_msgs::Point idx2wp (int index);

    /**
     * @brief Remove an edge from the tree graph.
     * @param e The edge to remove.
     */
    void remove_edge (edge e);

    /**
     * @brief Round a position on the map to the nearest grid cell.
     * @param pos A continuous position on the grid map, measured in grid cells from the bottom/left of the map.
     * @return The position of the nearest grid cell.
     */
    double round2idx (double pos);

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

    /**
     * @brief Coordinate of the bottom left bounding point of the area.
     */
    geometry_msgs::Point origin;

    /**
     * @brief The rotation of the map.
     */
    double rotation;

    /**
     * @brief Width of the area to cover.
     */
    double width;

    /**
     * @brief Height of the area to cover.
     */
    double height;

    /**
     * @brief Whether the sweeping pattern is vertical or horizontal.
     */
    bool vertical;
};

#endif // MST_PATH_H
