#ifndef SWARM_KINEMATICS_EXCHANGER_H
#define SWARM_KINEMATICS_EXCHANGER_H

#include <map>
#include <vector>
#include <numeric>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <cpswarm_msgs/Position.h>
#include <cpswarm_msgs/Velocity.h>
#include <cpswarm_msgs/ArrayOfVectors.h>

using namespace std;
using namespace ros;

/**
 * @brief A vector type in polar format containing UUID of the corresponding CPS together with last updated time stamp.
 */
typedef struct vector_t {
    string uuid;
    vector<float> mag;
    vector<float> dir;
    Time stamp;
} vector_t;

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
 * @brief The positions of all known swarm members.
 */
map<string, vector_t> swarm_positions;

/**
 * @brief The velocities of all known swarm members.
 */
map<string, vector_t> swarm_velocities;

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

#endif // SWARM_KINEMATICS_EXCHANGER_H
