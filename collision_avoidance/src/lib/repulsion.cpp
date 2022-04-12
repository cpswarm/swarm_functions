#include "lib/repulsion.h"

repulsion::repulsion ()
{
    setpoint = CONTROL_UNDEFINED;
    pos_valid = false;
}

void repulsion::init (double dist_critical, double dist_avoid, string repulsion_shape, string attraction_shape)
{
    // initialize parameters
    this->dist_critical = dist_critical;
    this->dist_avoid = dist_avoid;
    this->repulsion_shape = repulsion_shape;
    this->attraction_shape = attraction_shape;

    // reset variables
    int_pos = geometry_msgs::PoseStamped();
    int_vel = geometry_msgs::Twist();
    direction = geometry_msgs::Vector3();
}

bool repulsion::calc ()
{
    // no position given yet
    if (pos_valid == false)
        return false;

    // no setpoint defined
    if (setpoint == CONTROL_UNDEFINED)
        return false;

    // invalid distances
    if (dist_avoid < dist_critical)
        return false;

    // repulsion from other cpss, normalized to [0,neighbors]
    geometry_msgs::Vector3 repulsion;
    int neighbors;
    double closest;
    repulse(repulsion, neighbors, closest);

    // no avoidance necessary
    if (neighbors <= 0) {
        return false;
    }

    // attraction towards goal position
    geometry_msgs::Vector3 attraction;
    attract(attraction, closest);

    // avoidance direction (class variable) as sum of all repulsions and attraction
    direction.x = attraction.x + repulsion.x;
    direction.y = attraction.y + repulsion.y;

    // limit magnitude of avoidance direction vector to 1
    if (hypot(direction.y, direction.x) > 1) {
        double avoidance_dir = atan2(direction.y, direction.x);
        direction.x = cos(avoidance_dir);
        direction.y = sin(avoidance_dir);
    }

    // avoidance magnitude, inverse to repulsion to move slower when other cpss close by
    // linear function of cps distance f(d)
    // minimum at d<=dist_critical: f(d)=dist_critical / 2
    // maximum at d>=dist_avoid:    f(d)=dist_avoid / 2
    double avoidance_mag = min(0.5*dist_avoid, max(0.5*dist_critical, 0.5*closest));

    // avoidance displacement
    if (setpoint == CONTROL_POSITION) {
        int_pos.header = pos.header;

        // avoidance position
        int_pos.pose.position.x = pos.pose.position.x + direction.x * avoidance_mag;
        int_pos.pose.position.y = pos.pose.position.y + direction.y * avoidance_mag;
        int_pos.pose.position.z = goal_pos.pose.position.z;

        // face original goal
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, atan2(attraction.y, attraction.x));
        int_pos.pose.orientation = tf2::toMsg(orientation);
    }

    // avoidance velocity
    else if (setpoint == CONTROL_VELOCITY) {
        int_vel.linear.x = direction.x * avoidance_mag;
        int_vel.linear.y = direction.y * avoidance_mag;
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
    pos_valid = true;
}

void repulsion::set_swarm (const cpswarm_msgs::ArrayOfVectors::ConstPtr& swarm)
{
    this->swarm = swarm->vectors;
}

geometry_msgs::Vector3 repulsion::get_dir ()
{
    return direction;
}

geometry_msgs::PoseStamped repulsion::get_pos ()
{
    return int_pos;
}

geometry_msgs::Twist repulsion::get_vel ()
{
    return int_vel;
}

void repulsion::repulse (geometry_msgs::Vector3& repulsion, int& neighbors, double& closest)
{
    // init repulsion
    repulsion.x = 0;
    repulsion.y = 0;
    neighbors = 0;
    closest = -1;

    // yaw of this cps
    tf2::Quaternion orientation;
    tf2::fromMsg(pos.pose.orientation, orientation);
    double yaw = tf2::getYaw(orientation);

    // compute pair potentials for all neighbors
    for (auto pose : swarm) {
        // measure closest neighbor
        if (pose.vector.magnitude < closest || closest < 0) {
            closest = pose.vector.magnitude;
        }

        // repulsion only from close neighbors
        if (pose.vector.magnitude < dist_avoid) {
            // count neighbors to repulse from
            ++neighbors;

            // pair potential
            double pot;

            // maximum repulsion
            if (pose.vector.magnitude < dist_critical)
                pot = 1.0;

            // linear function
            else if (repulsion_shape == "lin")
                pot = max(0.0, 1 - (pose.vector.magnitude - dist_critical) / (dist_avoid - dist_critical));

            // linear function with double slope
            else if (repulsion_shape == "li2")
                pot = min(1.0, max(0.0, -pose.vector.magnitude * 2.0 / (dist_avoid - dist_critical) + 2.0 * dist_avoid / (dist_avoid - dist_critical)));

            // sine function
            else if (repulsion_shape == "sine")
                pot = 0.5 - 0.5 * sin(M_PI / (dist_avoid - dist_critical) * (pose.vector.magnitude - 0.5 * (dist_avoid + dist_critical)));

            // logarithmic function
            else if (repulsion_shape == "log")
                pot = min(0.0, 1 - exp((pose.vector.magnitude - dist_avoid) * dist_critical/2.0));

            // exponential function
            else if (repulsion_shape == "exp")
                pot = exp((pose.vector.magnitude - dist_critical) * 2.0*log(0.5) / (dist_avoid - dist_critical));

            // constant repulsion
            else
                pot = 1;

            // absolute bearing of neighbor
            double bear = yaw + pose.vector.direction;

            // sum up potentials as vector pointing away from neighbor
            repulsion.x += pot * -cos(bear);
            repulsion.y += pot * -sin(bear);
        }
    }
}

void repulsion::attract (geometry_msgs::Vector3& attraction, double closest)
{
    geometry_msgs::Vector3 dir;
    double head;

    // heading towards original goal
    if (setpoint == CONTROL_POSITION && (goal_pos.pose.position.x != 0 || goal_pos.pose.position.y != 0))
        head = atan2(goal_pos.pose.position.y - pos.pose.position.y, goal_pos.pose.position.x - pos.pose.position.x);

    // velocity heading
    else if (setpoint == CONTROL_VELOCITY && (target_vel.linear.x != 0 || target_vel.linear.y != 0))
        head = atan2(target_vel.linear.y, target_vel.linear.x);

    // no valid setpoint
    else
        return;

    // magnitude, decrease with other cpss close by
    double magnitude;

    // no attraction
    if (closest < dist_critical)
        magnitude = 0.0;

    // linear function
    else if (attraction_shape == "lin")
        magnitude = min(1.0, max(0.0, 1.0 / (dist_avoid - dist_critical) * closest - (dist_critical / (dist_avoid - dist_critical))));

    // linear function with double slope
    else if (attraction_shape == "li2")
        magnitude = min(1.0, max(0.0, closest * 2.0/(dist_avoid - dist_critical) - (dist_avoid + dist_critical) / (dist_avoid - dist_critical)));

    // sine function
    else if (attraction_shape == "sin")
        magnitude = 0.5 * sin(2.0*M_PI / (2.0 * (dist_avoid - dist_critical)) * (closest - (dist_critical + dist_avoid) / 2.0)) + 0.5;

    // logarithmic function
    else if (attraction_shape == "log")
        magnitude = log(closest - dist_critical + 1) / exp(1);

    // exponential function
    else if (attraction_shape == "exp")
        magnitude = max(0.0, exp((closest - dist_avoid) * dist_critical/2.0));

    // constant
    else
        magnitude = 1.0;

    // calculate goal direction components
    attraction.x = magnitude * cos(head);
    attraction.y = magnitude * sin(head);
}
