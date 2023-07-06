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
 * @brief Callback function to receive the relative positions of the other CPSs in the swarm.
 * @param msg An array of distance and bearing of the other CPSs.
 */
void swarm_cb (const cpswarm_msgs::ArrayOf3dVectors::ConstPtr& msg)
{
    ca.set_swarm(msg);
}

/**
 * @brief Callback function for state updates.
 * @param msg State received from the CPS state machine.
 */
void state_callback (const std_msgs::String::ConstPtr& msg)
{
    // enable collision avoidance by default
    active = true;

    // check each part of the behavior state path (for nested states)
    string path = msg->data;
    size_t pos = 0;
    while ((pos = path.find("/")) != string::npos) {
        if (pos > 0) {
            // disable collision avoidance if behavior state is in the excluded list
            if (find(excluded.begin(), excluded.end(), path.substr(0, pos)) != excluded.end()) {
                active = false;
                ROS_DEBUG("In state %s, disable collision avoidance", msg->data.c_str());
            }
        }
        path.erase(0, pos + 1);
    }
    // disable collision avoidance if behavior state is in the excluded list
    if (find(excluded.begin(), excluded.end(), path) != excluded.end()) {
        active = false;
        ROS_DEBUG("In state %s, disable collision avoidance", msg->data.c_str());
    }

    if (active)
        ROS_DEBUG("In state %s, enable collision avoidance", msg->data.c_str());
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
    nh.param(this_node::getName() + "/dist_critical", dist_critical, 3.0);
    double dist_attract;
    nh.param(this_node::getName() + "/dist_attract", dist_attract, 6.0);
    double dist_repulse;
    nh.param(this_node::getName() + "/dist_repulse", dist_repulse, 12.0);
    string attraction_shape;
    nh.param(this_node::getName() + "/attraction_shape", attraction_shape, attraction_shape);
    string repulsion_shape;
    nh.param(this_node::getName() + "/repulsion_shape", repulsion_shape, repulsion_shape);

    ca.init(dist_critical, dist_attract, dist_repulse, attraction_shape, repulsion_shape);

    // ros communication
    Subscriber sp_pos_sub = nh.subscribe("pos_controller/goal_position", queue_size, sp_pos_cb);
    Subscriber sp_vel_sub = nh.subscribe("vel_controller/target_velocity", queue_size, sp_vel_cb);
    Subscriber pos_sub = nh.subscribe("pos_provider/pose", queue_size, pos_cb);
    Subscriber swarm_sub = nh.subscribe("swarm_position_rel", queue_size, swarm_cb);
    Subscriber state_sub = nh.subscribe("flexbe/behavior_update", queue_size, state_callback);
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
            int neighbors = ca.calc();

            // perform collision avoidance if necessary
            if (neighbors > 0) {
                // debug output
                ROS_ERROR_THROTTLE(1, "Avoiding %d neighbors!", neighbors);

                // using position setpoint
                geometry_msgs::PoseStamped pos = ca.get_pos();
                pos.header.stamp = Time::now();
                if (ca.sp_pos()) {
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
                    geometry_msgs::PoseStamped dir;
                    dir = pos;
                    // tf2::Quaternion orientation;
                    // orientation.setRPY(0, atan2(ca.get_dir().z, ca.get_dir().x), atan2(ca.get_dir().y, ca.get_dir().x));
                    // dir.pose.orientation = tf2::toMsg(orientation);
                    vis_pub.publish(dir);
                }
            }
        }

        rate.sleep();
    }

    return 0;
}
