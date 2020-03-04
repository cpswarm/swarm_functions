#ifndef AREA_DIVISION_H
#define AREA_DIVISION_H

#include <ros/ros.h>
#include <map>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <cpswarm_msgs/AreaDivisionEvent.h>
#include <cpswarm_msgs/GetDouble.h>
#include <cpswarm_msgs/ArrayOfStates.h>
#include <cpswarm_msgs/StateEvent.h>
#include <swarmros/String.h>
#include "lib/area_division.h"

using namespace std;
using namespace ros;

/**
 * @brief An enumeration for the state of the division.
 */
typedef enum {
    IDLE = 0, // this CPS is in a behavior state that does not require area division
    INIT,     // this CPS just changed to a behavior state that requires area division
    ACTIVE,   // this CPS is in a behavior state that requires area division
    SYNC,     // synchronization with the other CPSs is necessary
    DIVIDE,   // area division is required
    DEINIT    // this CPS just changed to a behavior state that does not require area division
} state_t;

/**
 * @brief The state of the area division node.
 */
state_t state;

/**
 * @brief Subscriber to get CPS UUID.
 */
Subscriber uuid_sub;

/**
 * @brief Subscriber to get CPS position.
 */
Subscriber pose_sub;

/**
 * @brief Subscriber to get information about other CPSs in the swarm.
 */
Subscriber swarm_sub;

/**
 * @brief Subscriber to get grid map.
 */
Subscriber map_sub;

/**
 * @brief Subscriber to get area division requests from other CPSs.
 */
Subscriber division_sub;

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
 * @brief Publisher to visualize the rotated map.
 */
Publisher map_rot_pub;

/**
 * @brief Publisher to visualize the downsampled map.
 */
Publisher map_ds_pub;

/**
 * @brief Service client to get the angle which the area has to be rotated by.
 */
ServiceClient rotater_cli;

/**
 * @brief ROS rate object for controlling loop rates.
 */
Rate* rate;

/**
 * @brief UUID of this CPS.
 */
string uuid;

/**
 * @brief The current behavior state of this CPS.
 */
string behavior;

/**
 * @brief Whether a valid state has been received.
 */
bool behavior_valid;

/**
 * @brief The behavior states in which area division is active
 */
vector<string> behaviors;

/**
 * @brief The positions of the other swarm members.
 */
map<string, geometry_msgs::PoseStamped> swarm_pose;

/**
 * @brief The UUIDs of the other swarm members.
 */
map<string, Time> swarm;

/**
 * @brief Whether valid state information has been received from the other swarm members.
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
nav_msgs::OccupancyGrid global_map;

/**
 * @brief Whether a valid grid map has been received.
 */
bool map_valid;

/**
 * @brief The object encapsulating the area division optimization algorithm.
 */
area_division* division;

/**
 * @brief The translation by which the area has been shifted.
 */
geometry_msgs::Vector3 translation;

/**
 * @brief The grid map underlying the area division will be downsampled to this resolution in meter / cell.
 */
double resolution;

/**
 * @brief The time in seconds communication in the swarm can be delayed at most. Used to wait after an area division event before starting the area division or time after which it is assumed that a swarm member has left the swarm if no position update has been received.
 */
double swarm_timeout;

/**
 * @brief Whether to publish the area division on a topic for visualization.
 */
bool visualize;

/**
 * @brief The time at which the synchronization for the area division started.
 */
Time sync_start;

#endif // AREA_DIVISION_H
