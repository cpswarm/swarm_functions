#include "lib/repulsion.h"

repulsion::repulsion ()
{
}

void repulsion::init (double equi_dist, double repulse_spring, double repulse_max, double accel_time)
{
    this->setpoint = CONTROL_UNDEFINED;
    this->equi_dist = equi_dist;
    this->repulse_spring = repulse_spring;
    this->repulse_max = repulse_max;
    this->accel_time = accel_time;
}

bool repulsion::calc ()
{
    // repulsion from other cpss
    geometry_msgs::Vector3 a_rep = this->repulse();
    
    // no avoidance necessary
    if (a_rep.x == 0 && a_rep.y == 0)
        return false;

    // get current velocity
    if (this->setpoint == CONTROL_POSITION) {
        // TODO: get from velocity provider, also for vel sp?
        geometry_msgs::Vector3 vel_cur = vel.get_velocity();
    }

    // calculate avoidance velocity
    // TODO
    int_vel.x = vel_cur.x + 1 / accel_time * (v_flock.x - vel_cur.x) * dt + (a_repulsion.x + a_alignment.x + a_wall.x) * dt;
    int_vel.y = vel_cur.y + 1 / accel_time * (v_flock.y - vel_cur.y) * dt + (a_repulsion.y + a_alignment.y + a_wall.y) * dt;

    // calculate avoidance position
    if (this->setpoint == CONTROL_POSITION) {
        // TODO
    }

    return true;
}

bool repulsion::sp_pos ()
{
    return this->setpoint == CONTROL_POSITION;
}

bool repulsion::sp_vel ()
{
    return this->setpoint == CONTROL_VELOCITY;
}

void repulsion::set_goal_pos (const geometry_msgs::PoseStamped::ConstPtr& pos)
{
    this->goal_pos = *pos;
    this->setpoint = CONTROL_POSITION;
}

void repulsion::set_target_vel (const geometry_msgs::PoseStamped::ConstPtr& vel)
{
    this->target_vel = *vel;
    this->setpoint = CONTROL_VELOCITY;
}

void repulsion::set_swarm (const cpswarm_msgs::ArrayOfVectors::ConstPtr& swarm)
{
    this->swarm = swarm->vectors;
}

void repulsion::get_pos ()
{
    return this->int_pos;
}

void repulsion::get_vel ()
{
    return this->int_vel;
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
