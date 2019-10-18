#ifndef TARGETS_H
#define TARGETS_H

#include <unordered_map>
#include <tf2/utils.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <swarmros/String.h>
#include <cpswarm_msgs/TargetTracking.h>
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
     * @brief Compute the transformation between two poses.
     * @param p1 The first pose.
     * @param p2 The second pose.
     * @return The computed transformation.
     */
    geometry_msgs::Transform transform (geometry_msgs::Pose p1, geometry_msgs::Pose p2) const;

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
     * @brief Publisher for transmitting target tracking information.
     */
    Publisher tracking_pub;

    /**
     * @brief A map holding ID and target object of all known targets.
     */
    unordered_map<unsigned int, shared_ptr<target>> target_map;

    /**
     * @brief A map holding ID and target object of simulated targets, including the ones not yet found.
     */
    unordered_map<unsigned int, shared_ptr<target>> simulated_targets;

    /**
     * @brief The UUID of the CPS that owns this class instance.
     */
    string cps;

    /**
     * @brief Range of the target tracking camera in meter. It is used to simulate target detection. Targets within this distance are detected by the CPS.
     */
    double fov;
};

#endif // TARGETS_H
