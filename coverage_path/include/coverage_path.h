#ifndef COVERAGE_PATH_H
#define COVERAGE_PATH_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <cpswarm_msgs/GetWaypoint.h>
#include <cpswarm_msgs/GetDouble.h>
#include <cpswarm_msgs/GetVector.h>
#include "lib/spanning_tree.h"
#include "lib/mst_path.h"

using namespace std;
using namespace ros;

/**
 * @brief Publisher to visualize the coverage path.
 */
Publisher path_publisher;

/**
 * @brief Publisher to visualize the current waypoint.
 */
Publisher wp_publisher;

/**
 * @brief Publisher to visualize the minimum spanning tree.
 */
Publisher mst_publisher;

/**
 * @brief Service client to get the assigned area.
 */
Subscriber map_subscriber;

/**
 * @brief Service client to get the angle which the area has to be rotated by.
 */
ServiceClient rotater;

/**
 * @brief Service client to get the offset which the area has to be translated by.
 */
ServiceClient translater;

/**
 * @brief The grid map representing the environment to be covered.
 */
nav_msgs::OccupancyGrid area;

/**
 * @brief Whether a valid grid map has been received.
 */
bool map_valid;

/**
 * @brief The minimum-spanning-tree (MST) that defines the coverage path.
 */
spanning_tree tree;

/**
 * @brief The coverage path.
 */
mst_path path;

/**
 * @brief The grid map underlying the path planning will be downsampled to this resolution in meter / cell.
 */
double resolution;

/**
 * @brief Whether to publish the coverage path on a topic for visualization.
 */
bool visualize;

/**
 * @brief Whether to divide the area among the CPSs before generating the path or to generate the path on the complete map.
 */
bool divide_area;

/**
 * @brief Whether the sweeping pattern is vertical or horizontal.
 */
bool vertical;

/**
 * @brief Whether there are only waypoints at turning points of the path or also waypoints regularly spaced on straight line segments of the path.
 */
bool turning_points;

/**
 * @brief Whether the swarm configuration has changed which requires a replanning of the path.
 */
bool reconfigure;

#endif // COVERAGE_PATH_H
