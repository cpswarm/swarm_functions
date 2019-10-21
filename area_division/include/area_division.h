#ifndef AREA_DIVISION_H
#define AREA_DIVISION_H

#include <ros/ros.h>
#include <map>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <cpswarm_msgs/AreaDivisionEvent.h>
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
Publisher pos_pub;

/**
 * @brief Publisher to syncronize with other CPSs.
 */
Publisher swarm_pub;

/**
 * @brief Publisher to visualize the assigned area grid map.
 */
Publisher area_pub;

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
area_division* division;

/**
 * @brief The time in seconds to wait after an area division event before starting the area division.
 */
double swarm_timeout;

/**
 * @brief Whether to publish the area division on a topic for visualization.
 */
bool visualize;

/**
 * @brief Whether the swarm configuration has changed and the area needs to be divided again.
 */
bool reconfigure;

/**
 * @brief The time at which the synchronization for the area division started.
 */
Time sync_start;

#endif // AREA_DIVISION_H
