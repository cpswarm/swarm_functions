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

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    
    // initialize repulsion
    double equi_dist;
    nh.param(this_node::getName() + "/repulsion/equi_dist", equi_dist, 10.0);
    double repulse_spring;
    nh.param(this_node::getName() + "/repulsion/repulse_spring", repulse_spring, 1.0);
    double repulse_max;
    nh.param(this_node::getName() + "/repulsion/repulse_max", repulse_max, 1.0);
    double avoid_vel;
    nh.param(this_node::getName() + "/repulsion/avoid_vel", avoid_vel, 1.0);
    double accel_time;
    nh.param(this_node::getName() + "/repulsion/accel_time", accel_time, 1.0);

    ca.init(1.0/loop_rate, equi_dist, repulse_spring, repulse_max, avoid_vel, accel_time);

    // ros communication
    Subscriber sp_pos_sub = nh.subscribe("pos_controller/goal_position", queue_size, sp_pos_cb);
    Subscriber sp_vel_sub = nh.subscribe("vel_controller/target_velocity", queue_size, sp_vel_cb);
    Subscriber pos_sub = nh.subscribe("pos_provider/pose", queue_size, pos_cb);
    Subscriber vel_sub = nh.subscribe("vel_provider/velocity", queue_size, vel_cb);
    Subscriber swarm_sub = nh.subscribe("swarm_position_rel", queue_size, swarm_cb);
    Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/ca_goal_position", queue_size, true);
    Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("vel_controller/ca_target_velocity", queue_size, true);

    // init loop rate
    Rate rate(loop_rate);    

    ROS_DEBUG("Collision avoidance ready");

    // perform collision avoidance
    while (ok()) {
        // calculate avoidance position / velocity
        bool avoid = ca.calc();

        // perform collision avoidance if necessary
        if (avoid) {
            // using position setpoint
            if (ca.sp_pos()) 
                pos_pub.publish(ca.get_pos());

            // using velocity setpoint
            else if (ca.sp_vel())
                vel_pub.publish(ca.get_vel());

            else
                ROS_ERROR("Unknown setpoint, cannot perform collision avoidance!");
        }

        rate.sleep();
        spinOnce();
    }

    return 0;
}
