#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/Pose.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include <cpswarm_msgs/TargetTracking.h>
#include "lib/targets.h"

using namespace std;
using namespace ros;

/**
 * @brief The targets being monitored.
 */
targets* monitor;

/**
 * @brief Current position of the CPS.
 */
geometry_msgs::Pose pose;

/**
 * @brief Whether a valid position has been received.
 */
bool pose_valid;

/**
 * @brief Compute the current yaw orientation of the CPS.
 * @return The current yaw angle of the CPS counterclockwise starting from x-axis/east.
 */
double yaw ()
{
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    return tf2::getYaw(orientation);
}

/**
 * @brief Callback function for target position.
 * @param msg ID of target and translation between CPS and target as received from the OpenMV camera.
 */
void tracking_callback (const cpswarm_msgs::TargetTracking::ConstPtr& msg)
{
    // compute distance and direction of target
    double distance = hypot(msg->tf.translation.x, msg->tf.translation.y);
    double direction = yaw() + atan2(msg->tf.translation.y, -msg->tf.translation.x) - M_PI / 2; // x is inverted in tracking camera tf

    // create position event message to update target internally
    cpswarm_msgs::TargetPositionEvent event;
    event.id = msg->id;
    event.pose.pose.position.x = pose.position.x + distance * cos(direction);
    event.pose.pose.position.y = pose.position.y + distance * sin(direction);
    event.header.stamp = msg->header.stamp;
    monitor->update(event, TARGET_TRACKED);
}

/**
 * @brief Callback function to receive information about targets found by other CPSs.
 * @param msg ID and position of target.
 */
void found_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    monitor->update(*msg, TARGET_KNOWN);
}

/**
 * @brief Callback function to receive updated information about targets from other CPSs.
 * @param msg ID and position of target.
 */
void update_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    monitor->update(*msg, TARGET_KNOWN);
}

/**
 * @brief Callback function to receive information about targets lost by other CPSs.
 * @param msg ID and last known position of target.
 */
void lost_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    monitor->update(*msg, TARGET_LOST);
}

/**
 * @brief Callback function for incoming target done events.
 * @param msg ID and last position of target.
 */
void done_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    monitor->update(*msg, TARGET_DONE);
}

/**
 * @brief Callback function for position updates.
 * @param msg Position received from the CPS.
 */
void pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // valid pose received
    if (msg->header.stamp.isValid())
        pose_valid = true;

    // store new position and orientation in class variables
    pose = msg->pose;

    ROS_DEBUG_THROTTLE(1, "Yaw %.2f", yaw());
    ROS_DEBUG_THROTTLE(1, "Pose [%.2f, %.2f, %.2f]", pose.position.x, pose.position.y, pose.position.z);
}

/**
 * @brief Main function to be executed by ROS.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char** argv)
{
    // init ros node
    init(argc, argv, "target_monitor");
    NodeHandle nh;

    // read parameters
    bool simulation;
    nh.param(this_node::getName() + "/simulation", simulation, false);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // rate of update loop
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);

    // create target monitor
    monitor = new targets();

    // read target positions from parameter file
    if (simulation) {
        monitor->simulate();
    }

    // no pose received yet
    pose_valid = false;

    // publishers and subscribers
    Subscriber pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    Subscriber tracking_sub = nh.subscribe("target_tracking", queue_size, tracking_callback);
    Subscriber local_done_sub = nh.subscribe("target_done", queue_size, done_callback);
    Subscriber found_sub = nh.subscribe("bridge/events/target_found", queue_size, found_callback);
    Subscriber update_sub = nh.subscribe("bridge/events/target_update", queue_size, update_callback);
    Subscriber lost_sub = nh.subscribe("bridge/events/target_lost", queue_size, lost_callback);
    Subscriber done_sub = nh.subscribe("bridge/events/target_done", queue_size, done_callback);

    // init position and yaw
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid pose...");
        rate.sleep();
        spinOnce();
    }

    // update information about targets
    while (ok()) {
        monitor->update(pose);
        rate.sleep();
        spinOnce();
    }

    // destroy target monitor
    delete monitor;

    return 0;
}
