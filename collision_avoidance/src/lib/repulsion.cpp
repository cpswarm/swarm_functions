#include "lib/repulsion.h"

repulsion::repulsion ()
{
}

void repulsion::init (double dist_critical, double dist_avoid, double vel_avoid)
{
    setpoint = CONTROL_UNDEFINED;
    this->dist_critical = dist_critical;
    this->dist_avoid = dist_avoid;
}

bool repulsion::calc ()
{
    // repulsion from other cpss
    geometry_msgs::Vector3 repulsion;
    int neighbors;
    repulse(repulsion, neighbors);

    // no avoidance necessary
    if (neighbors <= 0) {
        return false;
    }

    // magnitude of repulsion
    double repulsion_mag = hypot(vector.x, vector.y);

    // direction towards goal position
    geometry_msgs::Vector3 direction = target_direction();

    // avoidance direction
    geometry_msgs::Vector3 avoidance;
    avoidance.x = direction.x + repulsion.x;
    avoidance.y = direction.y + repulsion.y;

    // avoidance magnitude
    double avoidance_mag = 1 - repulsion_mag / neighbors;

    // avoidance displacement
    if (setpoint == CONTROL_POSITION) {
        int_pos.pose.position.x = pos.pose.position.x + avoidance.x * avoidance_mag;
        int_pos.pose.position.y = pos.pose.position.y + avoidance.y * avoidance_mag;
        int_pos.pose.position.z = goal_pos.pose.position.z;
    }

    // avoidance velocity
    else if (setpoint == CONTROL_VELOCITY) {
        int_vel.linear.x = avoidance.x * avoidance_mag;
        int_vel.linear.y = avoidance.y * avoidance_mag;
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

void repulsion::repulse (geometry_msgs::Vector3& repulsion, int& neighbors)
{
    // init repulsion
    repulsion.x = 0;
    repulsion.y = 0;
    neighbors = 0;

    // yaw of this cps
    tf2::Quaternion orientation;
    tf2::fromMsg(pos.pose.orientation, orientation);
    double yaw = tf2::getYaw(orientation);

    // compute pair potentials for all neighbors
    for (auto pose : swarm) {
        // repulsion only from close neighbors
        if (pose.vector.magnitude < dist_avoid) {
            // count neighbors to repulse from
            ++neighbors;

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
            repulsion.x += pot * -cos(bear);
            repulsion.y += pot * -sin(bear);
        }
    }
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

    // normalize velocity
    else if (setpoint == CONTROL_VELOCITY) {
        double mag = hypot(target_vel.linear.x, target_vel.linear.y);
        vel.x = target_vel.linear.x / mag;
        vel.y = target_vel.linear.y / mag;
    }

    return vel;
}
