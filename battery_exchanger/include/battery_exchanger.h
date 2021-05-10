#ifndef BATTERY_EXCHANGER_H
#define BATTERY_EXCHANGER_H

#include <map>
#include <ros/ros.h>
#include <cpswarm_msgs/ArrayOfBatteries.h>
#include <cpswarm_msgs/BatteryStateEvent.h>
#include <cpswarm_msgs/BatteryState.h>

using namespace std;
using namespace ros;

/**
 * @brief A battery state type containing UUID and battery state of the corresponding CPS together with last updated time stamp.
 */
typedef struct battery_t {
    string uuid;
    cpswarm_msgs::BatteryState state;
    Time stamp;
} battery_t;

/**
 * @brief Current battery state of the CPS.
 */
cpswarm_msgs::BatteryState battery;

/**
 * @brief Whether a valid battery state has been received.
 */
bool battery_valid;

/**
 * @brief The battery state of all known swarm members.
 */
map<string, battery_t> swarm_battery;

#endif // BATTERY_EXCHANGER_H
