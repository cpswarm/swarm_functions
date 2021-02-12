#include "lib/repulsion.h"

repulsion::repulsion ()
{
}

void repulsion::init (double dist_critical, double dist_avoid, double vel_avoid, double accel_max, double time_vel, double time_accel)
{
    setpoint = CONTROL_UNDEFINED;
    this->dist_critical = dist_critical;
    this->dist_avoid = dist_avoid;
    this->vel_avoid = vel_avoid;
    this->accel_max = accel_max;
    this->time_vel = time_vel;
    this->time_accel = time_accel;
}

bool repulsion::calc ()
{
    // repulsion from other cpss
    geometry_msgs::Vector3 a_rep = repulse();

    // no avoidance necessary
    if (a_rep.x == 0 && a_rep.y == 0) {
        return false;
    }

    // calculate velocity to reach goal position
    geometry_msgs::Vector3 vel = target_velocity();

    // calculate avoidance velocity
    int_vel.linear.x = vel.x + a_rep.x * time_accel;
    int_vel.linear.y = vel.y + a_rep.y * time_accel;

    // calculate avoidance position
    if (setpoint == CONTROL_POSITION) {
        int_pos.pose.position.x = pos.pose.position.x + int_vel.linear.x * time_vel;
        int_pos.pose.position.y = pos.pose.position.y + int_vel.linear.y * time_vel;
        int_pos.pose.position.z = goal_pos.pose.position.z;
    }

    return true;
}

bool repulsion::sp_pos ()
{
    return setpoint == CONTROL_POSITION;
}

bool repulsion::sp_vel ()
{
    return setpoint == CONTROL_VELOCITY;
}

void repulsion::set_sp_pos (const geometry_msgs::PoseStamped::ConstPtr& pos)
{
    setpoint = CONTROL_POSITION;
    goal_pos = *pos;
}

void repulsion::set_sp_vel (const geometry_msgs::Twist::ConstPtr& vel)
{
    setpoint = CONTROL_VELOCITY;
    target_vel = *vel;
}

void repulsion::set_pos (const geometry_msgs::PoseStamped::ConstPtr& pos)
{
    this->pos = *pos;
}

void repulsion::set_vel (const geometry_msgs::TwistStamped::ConstPtr& vel)
{
    this->vel = vel->twist;
}

void repulsion::set_swarm (const cpswarm_msgs::ArrayOfVectors::ConstPtr& swarm)
{
    this->swarm = swarm->vectors;
}

geometry_msgs::PoseStamped repulsion::get_pos ()
{
    return int_pos;
}

geometry_msgs::Twist repulsion::get_vel ()
{
    return int_vel;
}

geometry_msgs::Vector3 repulsion::repulse ()
{
    // init repulsive force acceleration
    geometry_msgs::Vector3 a_repulsion;
    a_repulsion.x = 0;
    a_repulsion.y = 0;

    // yaw of this cps
    tf2::Quaternion orientation;
    tf2::fromMsg(pos.pose.orientation, orientation);
    double yaw = tf2::getYaw(orientation);

    // compute pair potentials for all neighbors
    for (auto pose : swarm) {
        // repulsion only from close neighbors
        if (pose.vector.magnitude < dist_avoid) {
            // pair potential
            double pot;

            // maximum repulsion
            if (pose.vector.magnitude < dist_critical)
                pot = 1;

            // repulsion following sine function
            else
                pot = 0.5 - 0.5 * sin(M_PI / (dist_avoid - dist_critical) * (pose.vector.magnitude - 0.5 * (dist_avoid + dist_critical)));

            // absolute bearing of neighbor
            double bear = yaw + pose.vector.direction;

            // sum up potentials as vector pointing away from neighbor
            a_repulsion.x += pot * -cos(bear);
            a_repulsion.y += pot * -sin(bear);
        }
    }

    // normalize to maximum acceleration
    double mag = hypot(a_repulsion.x, a_repulsion.y);
    if (mag > 0) {
        a_repulsion.x *= accel_max / mag;
        a_repulsion.y *= accel_max / mag;
    }

    return a_repulsion;
}

geometry_msgs::Vector3 repulsion::target_velocity ()
{
    geometry_msgs::Vector3 vel;

    // get direction towards original goal
    if (setpoint == CONTROL_POSITION) {
        // bearing of goal
        double bear = atan2(goal_pos.pose.position.y - pos.pose.position.y, goal_pos.pose.position.x - pos.pose.position.x);

        // velocity components in goal direction
        vel.x = cos(bear);
        vel.y = sin(bear);
    }
    else if (setpoint == CONTROL_VELOCITY) {
        // normalize
        double mag = hypot(target_vel.linear.x, target_vel.linear.y);
        vel.x = target_vel.linear.x / mag;
        vel.y = target_vel.linear.y / mag;
    }

    // apply desired velocity magnitude
    vel.x *= vel_avoid;
    vel.y *= vel_avoid;

    return vel;
}
