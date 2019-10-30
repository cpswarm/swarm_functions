#ifndef TARGET_H
#define TARGET_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <cpswarm_msgs/TargetPositionEvent.h>

using namespace std;
using namespace ros;

/**
 * @brief An enumeration for the state of a target.
 */
typedef enum {
    TARGET_UNKNOWN = 0, // unknown target
    TARGET_KNOWN,       // target found by another CPS
    TARGET_TRACKED,     // target found by this CPS
    TARGET_ASSIGNED,    // target assigned to any CPS
    TARGET_LOST,        // target lost by any CPS
    TARGET_DONE         // target completed by any CPS
} target_state_t;

/**
 * @brief A target that is monitored by the CPSs.
 */
class target
{
public:
    /**
     * @brief Constructor.
     */
    target ();

    /**
     * @brief Copy constructor.
     */
    target (const target& t);

    /**
     * @brief Constructor that initializes some private member variables.
     * @param id The target ID.
     * @param state The target state.
     */
    target (unsigned int id, target_state_t state);

    /**
     * @brief Constructor that initializes some private member variables.
     * @param id The target ID.
     * @param state The target state.
     * @param pose The position of the target.
     */
    target (unsigned int id, target_state_t state, geometry_msgs::Pose pose);

    /**
     * @brief Constructor that initializes some private member variables.
     * @param id The target ID.
     * @param state The target state.
     * @param pose The position of the target.
     * @param stamp The time stamp of target.
     */
    target (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp);

    /**
     * @brief Destructor that destroys all objects.
     */
    ~target ();

    /**
     * @brief Get the position of the target.
     * @return A pose containing the target position.
     */
    geometry_msgs::Pose get_pose ();

    /**
     * @brief Check whether the target tracked by this CPS has been lost. Switch state from TARGET_TRACKED to TARGET_LOST if the tracking timeout has expired and publish event.
     */
    void lost ();

    /**
     * @brief Assignment operator.
     * @param t The object from where to take the assignment values.
     */
    void operator= (const target& t);

    /**
     * @brief Update the information about a target.
     * @param state The state of the target.
     * @param pose The position of the target.
     * @param stamp The time stamp of the update.
     */
    void update (target_state_t state, geometry_msgs::Pose pose, Time stamp);

private:
    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief The ID of this target. Negative IDs are invalid.
     */
    int id;

    /**
     * @brief State of the target.
     */
    target_state_t state;

    /**
     * @brief Position of the target.
     */
    geometry_msgs::Pose pose;

    /**
     * @brief The position of the target the last time it was published as target position event.
     */
    geometry_msgs::Pose last_pose;

    /**
     * @brief Time stamp of latest update of the target.
     */
    Time stamp;

    /**
     * @brief The time in seconds after which a target transitions into the state TARGET_LOST when no target update has been received.
     */
    Duration timeout;

    /**
     * @brief The loop rate object for running the behavior control loops at a specific frequency.
     */
    Rate* rate;

    /**
     * @brief Minimum distance between two consecutive target positions such that a target update event is published.
     */
    double target_tolerance;

    /**
     * @brief Publisher for transmitting information about found targets to other CPSs.
     */
    Publisher target_found_pub;

    /**
     * @brief Publisher for transmitting updated information about targets to other CPSs.
     */
    Publisher target_update_pub;

    /**
     * @brief Publisher for transmitting information about lost targets to other CPSs.
     */
    Publisher target_lost_pub;

    /**
     * @brief Publisher for transmitting information about completed targets to other CPSs.
     */
    Publisher target_done_pub;
};

#endif // TARGET_H
