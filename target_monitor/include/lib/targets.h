#ifndef TARGETS_H
#define TARGETS_H

#include <unordered_map>
#include <tf2/utils.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <swarmros/String.h>
#include <cpswarm_msgs/TargetTracking.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include <cpswarm_msgs/TargetHelp.h>
#include <cpswarm_msgs/TargetTrackedBy.h>
#include <cpswarm_msgs/ArrayOfBatteries.h>
#include "target.h"

/**
 * @brief A collection of target objects that are searched, tracked, and rescued by the CPSs.
 */
class targets
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    targets ();

    /**
     * @brief Publish a target position event for the given target.
     * @param id The ID of the target.
     */
    void publish (unsigned int id);

    /**
     * @brief Read simulated targets from parameter file.
     */
    void simulate ();

    /**
     * @brief Update the state of all targets. If no update has been received for a target within a fixed period, its state will change to TARGET_LOST. This needs to be called periodically.
     * @param pose The current position of the CPS.
     */
    void update (geometry_msgs::Pose pose);

    /**
     * @brief Update the information of a target.
     * @param msg The target position event message.
     * @param state The target state.
     *
     */
    void update (cpswarm_msgs::TargetPositionEvent msg, target_state_t state);

private:
    /**
     * @brief Publish a target position event.
     * @param event The name of the event.
     * @param id The ID of the target.
     */
    void publish_event (string event, string id);

    /**
     * @brief Compute the transformation between two poses.
     * @param p1 The first pose.
     * @param p2 The second pose.
     * @return The computed transformation.
     */
    geometry_msgs::Transform transform (geometry_msgs::Pose p1, geometry_msgs::Pose p2) const;

    /**
     * @brief Callback function to receive the UUID from the communication library.
     * @param msg Battery state of all known CPSs.
     */
    void battery_callback (const cpswarm_msgs::ArrayOfBatteries::ConstPtr& msg);

    /**
     * @brief Callback function to receive the UUID from the communication library.
     * @param msg UUID of this node.
     */
    void uuid_callback (const swarmros::String::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Publisher for transmitting target tracking information in simulation.
     */
    Publisher tracking_pub;

    /**
     * @brief Publisher for transmitting information about found targets locally to other nodes and remotely to other CPSs.
     */
    Publisher target_found_pub;

    /**
     * @brief Publisher for transmitting updated information about targets locally to other nodes and remotely to other CPSs.
     */
    Publisher target_update_pub;

    /**
     * @brief Publisher for transmitting information about lost targets locally to other nodes and remotely to other CPSs.
     */
    Publisher target_lost_pub;

    /**
     * @brief Publisher for transmitting information about completed targets locally to other nodes and remotely to other CPSs.
     */
    Publisher target_done_pub;

    /**
     * @brief Publisher for transmitting information about targets where help is needed for tracking locally to other nodes.
     */
    Publisher target_help_pub;

    /**
     * @brief Publisher for transmitting the number of CPSs tracking a target.
     */
    Publisher tracked_by_pub;

    /**
     * @brief Subscriber for receiving battery state information from other CPSs.
     */
    Subscriber battery_sub;

    /**
     * @brief A map holding ID and target object of all known targets.
     */
    unordered_map<string, shared_ptr<target>> target_map;

    /**
     * @brief A map holding ID and target object of simulated targets, including the ones not yet found.
     */
    unordered_map<string, shared_ptr<target>> simulated_targets;

    /**
     * @brief The UUID of the CPS that owns this class instance.
     */
    string cps;

    /**
     * The remaining times that tracking CPSs can still work.
     */
    map<string, int> batteries;

    /**
     * @brief Range of the target tracking camera in meter. It is used to simulate target detection. Targets within this distance are detected by the CPS.
     */
    double fov;
};

#endif // TARGETS_H
