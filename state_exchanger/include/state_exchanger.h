#ifndef STATE_EXCHANGER_H
#define STATE_EXCHANGER_H

#include <map>
#include <ros/ros.h>
#include <smach_msgs/SmachContainerStatus.h>
#include <cpswarm_msgs/ArrayOfStates.h>
#include <cpswarm_msgs/StateEvent.h>

using namespace std;
using namespace ros;

/**
 * @brief A STATE type containing UUID and state of the corresponding CPS together with last updated time stamp.
 */
typedef struct state_t {
    string uuid;
    string state;
    Time stamp;
} state_t;

/**
 * @brief Current state of the CPS.
 */
string state;

/**
 * @brief Whether a valid state has been received.
 */
bool state_valid;

/**
 * @brief The state of all known swarm members.
 */
map<string, state_t> swarm_state;

/**
 * @brief The path of the smach state machine whose state shall be exchanged.
 */
string sm_path;

#endif // STATE_EXCHANGER_H
