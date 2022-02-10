#ifndef roi_assignment_H
#define roi_assignment_H

#include <string>
#include <set>
#include <map>
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <swarmros/String.h>
#include <cpswarm_msgs/RoiAssignmentAction.h>
#include <cpswarm_msgs/GetMultiPoints.h>
#include <cpswarm_msgs/GetDist.h>
#include <cpswarm_msgs/TaskAllocationEvent.h>
#include <cpswarm_msgs/TaskAllocatedEvent.h>
#include "lib/auctioning.h"
#include "lib/auction_rois.h"

using namespace std;
using namespace ros;

/**
 * @brief The random number generator used for waiting on other auctions.
 */
random_numbers::RandomNumberGenerator* rng;

/**
 * @brief Action server for ROI assignment.
 */
typedef actionlib::SimpleActionServer<cpswarm_msgs::RoiAssignmentAction> AssignmentAction;

/**
 * @brief Service client to get the ROIs from the area provider.
 */
ServiceClient get_rois_client;

/**
 * @brief Publisher to disseminate auctions.
 */
Publisher auction_pub;

/**
 * @brief Publisher to disseminate auction results.
 */
Publisher result_pub;

/**
 * @brief Publisher to disseminate auction bids.
 */
Publisher bid_pub;

/**
 * @brief The UUID of this CPS.
 */
string uuid;

/**
 * @brief The position of this CPS.
 */
geometry_msgs::PoseStamped position;

/**
 * @brief The object for managing the assignment auctions.
 */
auctioning* auct;

/**
 * @brief The object for managing the ROIs that need to be assigned.
 */
auction_rois rois;

#endif // roi_assignment_H
