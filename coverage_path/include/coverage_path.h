#ifndef COVERAGE_PATH_H
#define COVERAGE_PATH_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <cpswarm_msgs/GetWaypoint.h>
#include <cpswarm_msgs/GetMap.h>
#include <cpswarm_msgs/ArrayOfStates.h>
#include <cpswarm_msgs/PathGenerationAction.h>
#include "lib/spanning_tree.h"
#include "lib/mst_path.h"

using namespace std;
using namespace ros;

/**
 * @brief Action server for path generation.
 */
typedef actionlib::SimpleActionServer<cpswarm_msgs::PathGenerationAction> GenerationAction;

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
 * @brief Subscriber to get information about other CPSs in the swarm.
 */
Subscriber swarm_sub;

/**
 * @brief Service client to get the area to cover.
 */
ServiceClient area_getter;

/**
 * @brief The coverage path.
 */
mst_path path;

/**
 * @brief The UUIDs of the other swarm members.
 */
map<string, Time> swarm;

/**
 * @brief The grid map underlying the path planning will be downsampled to this resolution in meter / cell.
 */
double resolution;

/**
 * @brief Whether to publish the coverage path on a topic for visualization.
 */
bool visualize;

/**
 * @brief Whether to divide the area among the CPSs in the swarm before generating the path. Joining or leaving swarm members will trigger regeneration of the path.
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
 * @brief The time in seconds communication in the swarm can be delayed at most. Used to wait after an area division event before starting the area division or time after which it is assumed that a swarm member has left the swarm if no position update has been received.
 */
double swarm_timeout;

/**
 * @brief The behavior states in which CPSs are considered part of the swarm.
 */
vector<string> behaviors;

/**
 * @brief Whether the swarm configuration has changed which requires a replanning of the path.
 */
bool reconfigure;

#endif // COVERAGE_PATH_H
