#ifndef COVERAGE_PATH_H
#define COVERAGE_PATH_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <cpswarm_msgs/GetWaypoint.h>
#include <cpswarm_msgs/ArrayOfStates.h>
#include <cpswarm_msgs/StateEvent.h>
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
 * @brief Service client to get the assigned area.
 */
ServiceClient map_getter;

/**
 * @brief Current position of the CPS.
 */
geometry_msgs::Pose pose;

/**
 * @brief Whether a valid position has been received.
 */
bool pose_valid;

/**
 * @brief Current state of the CPS.
 */
string state;

/**
 * @brief Whether a valid state has been received.
 */
bool state_valid;

/**
 * @brief The UUIDs of the other swarm members.
 */
map<string, Time> swarm;

/**
 * @brief Whether valid swarm information has been received.
 */
bool swarm_valid;

/**
 * @brief The grid map representing the environment to be covered.
 */
nav_msgs::OccupancyGrid gridmap;

/**
 * @brief The minimum-spanning-tree (MST) that defines the coverage path.
 */
spanning_tree tree;

/**
 * @brief The coverage path.
 */
mst_path path;

/**
 * @brief Time in seconds after which it is assumed that a swarm member has left the swarm if no position update has been received.
 */
double swarm_timeout;

/**
 * @brief Whether to publish the coverage path on a topic for visualization.
 */
bool visualize;

/**
 * @brief Whether the swarm configuration has changed which requires a replanning of the path.
 */
bool reconfigure;

#endif // COVERAGE_PATH_H
