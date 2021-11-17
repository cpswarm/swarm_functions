#include "collision_avoidance.h"

/**
 * @brief Callback function to receive the goal position of this CPS.
 * @param msg Current goal position sent to the position controller.
 */
void sp_pos_cb (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ca.set_sp_pos(msg);
}

/**
 * @brief Callback function to receive the target velocity of this CPS.
 * @param msg Current target velocity sent to the velocity controller.
 */
void sp_vel_cb (const geometry_msgs::Twist::ConstPtr& msg)
{
    ca.set_sp_vel(msg);
}

/**
 * @brief Callback function to receive the current position of this CPS.
 * @param msg Current position received from the FCU.
 */
void pos_cb (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ca.set_pos(msg);
}

/**
 * @brief Callback function to receive the current velocity of this CPS.
 * @param msg Current velocity received from the FCU.
 */
void vel_cb (const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    ca.set_vel(msg);
}

/**
 * @brief Callback function to receive the relative positions of the other CPSs in the swarm.
 * @param msg An array of distance and bearing of the other CPSs.
 */
void swarm_cb (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg)
{
    ca.set_swarm(msg);
}

/**
 * @brief Callback function for state updates.
 * @param msg State received from the CPS state machine.
 */
void state_callback (const smach_msgs::SmachContainerStatus::ConstPtr& msg)
{
    if (msg->path == "/SM_TOP/SarThreads/SarBehavior") {
        // enable collision avoidance by default
        active = true;

        // check if any behavior state is in the excluded list
        for (auto state : msg->active_states) {
            if (find(excluded.begin(), excluded.end(), state) != excluded.end()) {
                // disable collision avoidance
                active = false;
                ROS_DEBUG("In state %s, disable collision avoidance", state.c_str());
                break;
            }
        }
    }
}

/**
 * @brief A ROS node that avoids collisions with other CPSs in the swarm.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char **argv)
{
    // init ros node
    init(argc, argv, "collision_avoidance");
    NodeHandle nh;

    // don't perform collision avoidance initially
    active = false;

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    bool visualize;
    nh.param(this_node::getName() + "/visualize", visualize, false);
    nh.getParam(this_node::getName() + "/excluded", excluded);

    // initialize repulsion
    double dist_critical;
    nh.param(this_node::getName() + "/dist_critical", dist_critical, 1.0);
    double dist_avoid;
    nh.param(this_node::getName() + "/dist_avoid", dist_avoid, 3.0);
    string repulsion_shape = "linear";
    nh.param(this_node::getName() + "/repulsion_shape", repulsion_shape, repulsion_shape);

    ca.init(dist_critical, dist_avoid, repulsion_shape);

    // ros communication
    Subscriber sp_pos_sub = nh.subscribe("pos_controller/goal_position", queue_size, sp_pos_cb);
    Subscriber sp_vel_sub = nh.subscribe("vel_controller/target_velocity", queue_size, sp_vel_cb);
    Subscriber pos_sub = nh.subscribe("pos_provider/pose", queue_size, pos_cb);
    Subscriber vel_sub = nh.subscribe("vel_provider/velocity", queue_size, vel_cb);
    Subscriber swarm_sub = nh.subscribe("swarm_position_rel", queue_size, swarm_cb);
    Subscriber state_sub = nh.subscribe("smach_server/smach/container_status", queue_size, state_callback);
    Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/ca_goal_position", queue_size, true);
    Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("vel_controller/ca_target_velocity", queue_size, true);
    Publisher vis_pub;
    if (visualize)
        vis_pub = nh.advertise<geometry_msgs::PoseStamped>("collision_avoidance/direction", queue_size, true);

    // init loop rate
    Rate rate(loop_rate);

    ROS_DEBUG("Collision avoidance ready");

    // perform collision avoidance
    while (ok()) {
        // get updated information
        spinOnce();

        // only perform collision avoidance in desired behavior states
        if (active) {
            // calculate avoidance position / velocity
            bool avoid = ca.calc();

            // perform collision avoidance if necessary
            if (avoid) {
                // using position setpoint
                if (ca.sp_pos()) {
                    geometry_msgs::PoseStamped pos = ca.get_pos();
                    pos.header.stamp = Time::now();
                    pos_pub.publish(pos);
                }

                // using velocity setpoint
                else if (ca.sp_vel()) {
                    geometry_msgs::Twist vel = ca.get_vel();
                    // vel.header.stamp TODO
                    vel_pub.publish(vel);
                }

                else {
                    ROS_ERROR_THROTTLE(10, "Unknown setpoint, cannot perform collision avoidance!");
                }

                // visualization
                if (visualize) {
                    vis_pub.publish(ca.get_dir());
                }
            }
        }

        rate.sleep();
    }

    return 0;
}
