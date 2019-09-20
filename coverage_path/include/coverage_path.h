#ifndef COVERAGE_PATH_H
#define COVERAGE_PATH_H

#include <ros/ros.h>
#include <deque>
#include <map>
#include <vector>
#include <valarray>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cpswarm_msgs/ArrayOfPositions.h>
#include <cpswarm_msgs/Position.h>
#include <cpswarm_msgs/Velocity.h>
#include <cpswarm_msgs/ArrayOfVectors.h>
#include <cpswarm_msgs/VectorStamped.h>
#include <swarmros/String.h>

#include "lib/area_division.h"
#include "lib/spanning_tree.h"
#include "lib/mst_path.h"

using namespace std;
using namespace ros;

/**
 * @brief A vector type in Cartesian format containing UUID of the corresponding CPS together with last updated time stamp.
 */
typedef struct cartesian_vector_t {
    string uuid;
    vector<float> x;
    vector<float> y;
    Time stamp;
} cartesian_vector_t;

/**
 * @brief UUID of this CPS.
 */
string uuid;

/**
 * @brief Subscriber for the positions of the other CPSs.
 */
Subscriber swarm_pose_sub;

/**
 * @brief The positions of the other swarm members.
 */
map<string, geometry_msgs::PoseStamped> swarm_pose;

/**
 * @brief TODO
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
 * @brief The absolute positions of all known swarm members.
 */
// map<string, cartesian_vector_t> swarm_positions; TODO

/**
 * @brief TODO
 */
nav_msgs::OccupancyGrid gridmap;

/**
 * @brief TODO
 */
bool map_valid;

/**
 * @brief TODO
 */
area_division division;

/**
 * @brief TODO
 */
spanning_tree tree;

/**
 * @brief TODO
 */
mst_path path;

/**
 * @brief TODO
 */
double position_tolerance;

/**
 * @brief Time in seconds after which it is assumed that a swarm member has left the swarm if no position update has been received.
 */
double swarm_timeout;

/**
 * @brief TODO
 */
bool reconfigure;

#endif // COVERAGE_PATH_H
