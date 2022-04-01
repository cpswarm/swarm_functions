#include "lib/repulsion.h"

repulsion::repulsion ()
{
    setpoint = CONTROL_UNDEFINED;
    pos_valid = false;
}

void repulsion::init (double dist_critical, double dist_avoid, string repulsion_shape)
{
    this->dist_critical = dist_critical;
    this->dist_avoid = dist_avoid;
    this->repulsion_shape = repulsion_shape;
}

bool repulsion::calc ()
{
    // no position given yet
    if (pos_valid == false)
        return false;

    // no setpoint defined
    if (setpoint == CONTROL_UNDEFINED)
        return false;

    // repulsion from other cpss, normalized to number of neighbors
    geometry_msgs::Vector3 repulsion;
    int neighbors;
    repulse(repulsion, neighbors);

    // no avoidance necessary
    if (neighbors <= 0) {
        return false;
    }

    // magnitude of repulsion
    double repulsion_mag = hypot(repulsion.x, repulsion.y);

    // direction towards goal position, normalized
    geometry_msgs::Vector3 direction = target_direction();

    // avoidance direction normalized (each in the range [0,1])
    this->direction.x = (direction.x + repulsion.x / double(neighbors)) / 2.0;
    this->direction.y = (direction.y + repulsion.y / double(neighbors)) / 2.0;

    // avoidance magnitude, inverse to repulsion to move slower when other cpss close by
    // linear function, maximum at repulsion zero: dist_avoid, minimum at repulsion one: dist_critical
    double avoidance_mag = dist_avoid - repulsion_mag / neighbors * (dist_avoid - dist_critical);

    // avoidance displacement
    if (setpoint == CONTROL_POSITION) {
        int_pos.pose.position.x = pos.pose.position.x + this->direction.x * avoidance_mag;
        int_pos.pose.position.y = pos.pose.position.y + this->direction.y * avoidance_mag;
        int_pos.pose.position.z = goal_pos.pose.position.z;
    }

    // avoidance velocity
    else if (setpoint == CONTROL_VELOCITY) {
        int_vel.linear.x = this->direction.x * avoidance_mag;
        int_vel.linear.y = this->direction.y * avoidance_mag;
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

    // reset previous results
    int_pos = geometry_msgs::PoseStamped();
    int_vel = geometry_msgs::Twist();
}

void repulsion::set_sp_vel (const geometry_msgs::Twist::ConstPtr& vel)
{
    setpoint = CONTROL_VELOCITY;
    target_vel = *vel;

    // reset previous results
    int_pos = geometry_msgs::PoseStamped();
    int_vel = geometry_msgs::Twist();
}

void repulsion::set_pos (const geometry_msgs::PoseStamped::ConstPtr& pos)
{
    this->pos = *pos;
    pos_valid = true;

    // reset previous results
    int_pos = geometry_msgs::PoseStamped();
    int_vel = geometry_msgs::Twist();
}

void repulsion::set_swarm (const cpswarm_msgs::ArrayOfVectors::ConstPtr& swarm)
{
    this->swarm = swarm->vectors;

    // reset previous results
    int_pos = geometry_msgs::PoseStamped();
    int_vel = geometry_msgs::Twist();
}

geometry_msgs::PoseStamped repulsion::get_dir ()
{
    // use position of cps
    geometry_msgs::PoseStamped dir;
    dir = pos;

    // calculate orientation
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, atan2(direction.y, direction.x));
    dir.pose.orientation = tf2::toMsg(orientation);

    return dir;
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
            else if (repulsion_shape == "sine")
                pot = 0.5 - 0.5 * sin(M_PI / (dist_avoid - dist_critical) * (pose.vector.magnitude - 0.5 * (dist_avoid + dist_critical)));

            // repulsion following exp function
            else if (repulsion_shape == "sine")
                pot = exp((pose.vector.magnitude - dist_critical) * 2.0*log(0.5) / (dist_avoid - dist_critical));

            // repulsion following linear function
            else
                pot = max(1 - (pose.vector.magnitude - dist_critical) / (dist_avoid - dist_critical), 0.0);

            // absolute bearing of neighbor
            double bear = yaw + pose.vector.direction;

            // sum up potentials as vector pointing away from neighbor
            repulsion.x += pot * -cos(bear);
            repulsion.y += pot * -sin(bear);
        }
    }
}

geometry_msgs::Vector3 repulsion::target_direction ()
{
    geometry_msgs::Vector3 dir;

    // get direction towards original goal
    if (setpoint == CONTROL_POSITION) {
        // bearing of goal
        double bear = atan2(goal_pos.pose.position.y - pos.pose.position.y, goal_pos.pose.position.x - pos.pose.position.x);

        // velocity components in goal direction
        dir.x = cos(bear);
        dir.y = sin(bear);
    }

    // normalize velocity
    else if (setpoint == CONTROL_VELOCITY) {
        double mag = hypot(target_vel.linear.x, target_vel.linear.y);
        if (mag > 0) {
            dir.x = target_vel.linear.x / mag;
            dir.y = target_vel.linear.y / mag;
        }
    }

    return dir;
}
