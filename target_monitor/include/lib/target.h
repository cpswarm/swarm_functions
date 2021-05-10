#ifndef TARGET_H
#define TARGET_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

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
     * @brief Constructor that initializes some private member variables.
     * @param id The target ID.
     * @param state The target state.
     * @param pose The position of the target.
     * @param stamp The time stamp of target.
     * @param cps The CPS that is tracking the target.
     */
    target (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp, string cps);

    /**
     * @brief Destructor that destroys all objects.
     */
    ~target ();

    /**
     * @brief Get the number of CPSs currently tracking the target.
     * @return The number of CPSs.
     */
    int get_num_trackers ();

    /**
     * @brief Get the position of the target.
     * @return A pose containing the target position.
     */
    geometry_msgs::Pose get_pose ();

    /**
     * @brief Get the state of the target.
     * @return The current target state.
     */
    target_state_t get_state ();

    /**
     * @brief Get the time that the target still needs to be tracked.
     * @return The time in seconds.
     */
    double get_time_need ();

    /**
     * @brief Get the CPSs that are currently tracking the target.
     * @return The set of UUIDs of the tracking CPSs.
     */
    set<string> get_trackers ();

    /**
     * @brief Check whether the CPSs tracking the target need help.
     * @return True, if the minimum number of required tracking CPSs is not yet reached. False otherwise.
     */
    bool help ();

    /**
     * @brief Check whether the target tracked by this CPS has been lost. Switch state from TARGET_TRACKED to TARGET_LOST if the tracking timeout has expired.
     * @return Whether the target has been lost.
     */
    bool lost ();

    /**
     * @brief Assignment operator.
     * @param t The object from where to take the assignment values.
     */
    void operator= (const target& t);

    /**
     * @brief Check whether there are too many CPSs tracking the target.
     * @return True, if the maximum number of allowed tracking CPSs is exceeded. False otherwise.
     */
    bool overcrowded ();

    /**
     * @brief Update the information about a target.
     * @param state The state of the target.
     * @param pose The position of the target.
     * @param stamp The time stamp of the update.
     * @param cps The CPS that sends the update.
     */
    void update (target_state_t state, geometry_msgs::Pose pose, Time stamp, string cps);

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
     * @brief Time stamp of latest update of the target.
     */
    Time stamp;

    /**
     * @brief The time in seconds after which a target transitions into the state TARGET_LOST when no target update has been received.
     */
    Duration timeout;

    /**
     * @brief The time in seconds which a target has to be tracked until it switches to state done. Negative to disable, i.e., infinite time.
     */
    Duration time;

    /**
     * @brief A set of UUIDs of the CPSs tracking the target.
     */
    set<string> tracked_by;

    /**
     * @brief Minimum number of tracking CPSs that are needed for one target.
     */
    int min_trackers;

    /**
     * @brief Maximum number of tracking CPSs that are allowed for one target.
     */
    int max_trackers;

    /**
     * @brief The loop rate object for running the behavior control loops at a specific frequency.
     */
    Rate* rate;
};

#endif // TARGET_H
