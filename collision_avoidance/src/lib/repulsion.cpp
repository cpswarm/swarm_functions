#include "lib/repulsion.h"

repulsion::repulsion ()
{
}

void repulsion::init (double cycle, double equi_dist, double repulse_spring, double repulse_max, double avoid_vel, double accel_time)
{
    setpoint = CONTROL_UNDEFINED;
    this->cycle = cycle;
    this->equi_dist = equi_dist;
    this->repulse_spring = repulse_spring;
    this->repulse_max = repulse_max;
    this->avoid_vel = avoid_vel;
    this->accel_time = accel_time;
}

bool repulsion::calc ()
{
    // repulsion from other cpss
    geometry_msgs::Vector3 a_rep = repulse();
    
    // no avoidance necessary
    if (a_rep.x == 0 && a_rep.y == 0)
        return false;

    // calculate velocity to reach goal position
    geometry_msgs::Vector3 vel = target_velocity();

    // calculate avoidance velocity
    int_vel.linear.x = vel.x + 1 / accel_time * (vel.x - this->vel.linear.x) * cycle + a_rep.x * cycle;
    int_vel.linear.y = vel.y + 1 / accel_time * (vel.y - this->vel.linear.y) * cycle + a_rep.y * cycle;

    // calculate avoidance position
    if (setpoint == CONTROL_POSITION) {
        int_pos.pose.position.x = pos.pose.position.x + int_vel.linear.x * cycle;
        int_pos.pose.position.y = pos.pose.position.y + int_vel.linear.y * cycle;
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

    // compute pair potentials for all neighbors
    for (auto pose : swarm) {
        // repulsion only from close neighbors
        if (pose.vector.magnitude < equi_dist) {
            // compute pair potential
            double pot = min(repulse_max, equi_dist - pose.vector.magnitude) / pose.vector.magnitude;

            // sum up potentials
            a_repulsion.x += pot * pose.vector.magnitude * cos(pose.vector.direction);
            a_repulsion.y += pot * pose.vector.magnitude * sin(pose.vector.direction);
        }
    }

    // apply spring constant
    a_repulsion.x *= -repulse_spring;
    a_repulsion.y *= -repulse_spring;

    return a_repulsion;
}

geometry_msgs::Vector3 repulsion::target_velocity ()
{
    geometry_msgs::Vector3 vel;

    // direction
    if (setpoint == CONTROL_POSITION) {
        // yaw of cps
        tf2::Quaternion orientation;
        tf2::fromMsg(pos.pose.orientation, orientation);
        double yaw = tf2::getYaw(orientation);

        // relative bearing of goal
        double bear = remainder(atan2(goal_pos.pose.position.y - pos.pose.position.y, goal_pos.pose.position.x - pos.pose.position.x) - yaw, 2*M_PI);
        if (bear < 0)
            bear += 2*M_PI;

        // velocity components in goal direction
        vel.x = -sin(bear); // bearing relative to cps heading
        vel.y = cos(bear);
    }
    else if (setpoint == CONTROL_VELOCITY) {
        // normalize
        double mag = hypot(target_vel.linear.x, target_vel.linear.y);
        vel.x = target_vel.linear.x / mag;
        vel.y = target_vel.linear.y / mag;
    }

    // magnitude
    double magnitude = avoid_vel;
    // limit velocity for small distances
    double dist = hypot(goal_pos.pose.position.x - pos.pose.position.x, goal_pos.pose.position.y - pos.pose.position.y);
    if (dist < M_PI / 2.0) {
        magnitude *= sin(dist);
    }
    // apply to velocity
    vel.x *= magnitude;
    vel.y *= magnitude;

    return vel;
}
