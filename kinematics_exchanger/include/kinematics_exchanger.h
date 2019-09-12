#ifndef KINEMATICS_EXCHANGER_H
#define KINEMATICS_EXCHANGER_H

#include <map>
#include <vector>
#include <numeric>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <cpswarm_msgs/ArrayOfPositions.h>
#include <cpswarm_msgs/Position.h>
#include <cpswarm_msgs/Velocity.h>
#include <cpswarm_msgs/ArrayOfVectors.h>
#include <cpswarm_msgs/VectorStamped.h>

using namespace std;
using namespace ros;

/**
 * @brief A vector type in polar format containing UUID of the corresponding CPS together with last updated time stamp.
 */
typedef struct polar_vector_t {
    string uuid;
    vector<float> mag;
    vector<float> dir;
    Time stamp;
} polar_vector_t;

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
 * @brief Current position of the CPS.
 */
geometry_msgs::Pose pose;

/**
 * @brief Whether a valid position has been received.
 */
bool pose_valid;

/**
 * @brief Current velocity of the CPS.
 */
geometry_msgs::Twist velo;

/**
 * @brief Whether a valid velocity has been received.
 */
bool vel_valid;

/**
 * @brief The absolute positions of all known swarm members.
 */
map<string, cartesian_vector_t> swarm_positions;

/**
 * @brief The relative positions of all known swarm members.
 */
map<string, polar_vector_t> swarm_positions_rel;

/**
 * @brief The velocities of all known swarm members.
 */
map<string, polar_vector_t> swarm_velocities;

/**
 * @brief The number of data samples to average over for reliable results.
 */
int sample_size;

/**
 * @brief The number of position messages to ignore during initialization. This is because the first messages are inaccurate.
 */
int pos_init;

/**
 * @brief The number of velocity messages to ignore during initialization. This is because the first messages are inaccurate.
 */
int vel_init;

#endif // KINEMATICS_EXCHANGER_H
