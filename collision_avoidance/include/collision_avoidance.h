#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cpswarm_msgs/ArrayOfPositions.h>
#include "lib/repulsion.h"

using namespace std;
using namespace ros;

/**
 * @brief The object encapsulating the collision avoidance routines.
 */
repulsion ca;

#endif // COLLISION_AVOIDANCE_H
