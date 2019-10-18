#ifndef AREA_DIVISION_H
#define AREA_DIVISION_H

#include <ros/ros.h>
#include <map>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <cpswarm_msgs/ArrayOfPositions.h>
#include <swarmros/String.h>
#include "lib/area_division.h"

using namespace std;
using namespace ros;
/**
 * @brief UUID of this CPS.
 */
string uuid;

/**
 * @brief Publisher to stop the CPS.
 */
Publisher pos_publisher;

/**
 * @brief Publisher to visualize the assigned area grid map.
 */
Publisher area_publisher;

/**
 * @brief ROS rate object for controlling loop rates.
 */
Rate* rate;

/**
 * @brief The positions of the other swarm members.
 */
map<string, geometry_msgs::PoseStamped> swarm_pose;

/**
 * @brief Whether valid position information has been received from the other swarm members.
 */
bool swarm_valid;

/**
 * @brief Current position of the CPS.
 */
geometry_msgs::Pose pose;

/**
 * @brief Whether a valid position has been received.
 */
bool pose_valid;

/**
 * @brief The complete grid map.
 */
nav_msgs::OccupancyGrid gridmap;

/**
 * @brief Whether a valid grid map has been received.
 */
bool map_valid;

/**
 * @brief The object encapsulating the area division optimization algorithm.
 */
area_division division;

/**
 * @brief Time in seconds after which it is assumed that a swarm member has left the swarm if no position update has been received.
 */
double swarm_timeout;

/**
 * @brief The number of position updates from other swarm members to wait for before starting the area division.
 */
int swarm_updates;

/**
 * @brief Whether to publish the area division on a topic for visualization.
 */
bool visualize;

/**
 * @brief Whether the swarm configuration has changed and the area needs to be divided again.
 */
bool reconfigure;

/**
 * @brief Number of position updates received from the swarm.
 */
int updates;

#endif // AREA_DIVISION_H
