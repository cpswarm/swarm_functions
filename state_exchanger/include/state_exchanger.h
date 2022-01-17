#ifndef STATE_EXCHANGER_H
#define STATE_EXCHANGER_H

#include <map>
#include <ros/ros.h>
#include <flexbe_msgs/BEStatus.h>
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
 * @brief Whether to only listen or also send data.
 */
bool read_only;

#endif // STATE_EXCHANGER_H
