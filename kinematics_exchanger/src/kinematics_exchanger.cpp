#include "kinematics_exchanger.h"

/**
 * @brief Accumulate sines.
 * @param x The sum to add to.
 * @param y The value to compute the sine for.
 * @return The sum including the sine.
 */
float acc_sin (float x, float y) {
    return x + sin(y);
}

/**
 * @brief Accumulate cosines.
 * @param x The sum to add to.
 * @param y The value to compute the cosine for.
 * @return The sum including the cosine.
 */
float acc_cos (float x, float y) {
    return x + cos(y);
}

/**
 * @brief Get the yaw orientation of the current pose.
 * @return The yaw angle of the given pose counterclockwise starting from x-axis/east.
 */
double get_yaw ()
{
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    return tf2::getYaw(orientation);
}

/**
 * @brief Compute the velocity difference of the CPS to a given velocity.
 * @param v The velocity to compare.
 * @return The velocity relative to the current velocity of the CPS as magnitude and direction.
 */
cpswarm_msgs::Vector rel_velocity (geometry_msgs::Vector3 v)
{
    // compute relative velocity
    double dx = v.x - velo.linear.x;
    double dy = v.y - velo.linear.y;
    double mag = hypot(dx, dy);
    double dir = atan2(dy, dx);

    // return relative velocity
    cpswarm_msgs::Vector rel_vel;
    rel_vel.magnitude = mag;
    rel_vel.direction = dir;
    return rel_vel;
}

/**
 * @brief Callback function for position updates.
 * @param msg Position received from the CPS.
 */
void pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // valid pose received
    if (msg->header.stamp.isValid())
        pose_valid = true;

    // store new position and orientation in class variables
    pose = msg->pose;
}

/**
 * @brief Callback function for velocity updates.
 * @param msg Velocity received from the CPS FCU.
 */
void vel_callback (const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // valid pose received
    if (msg->header.stamp.isValid())
        vel_valid = true;

    velo = msg->twist;
}

/**
 * @brief Callback function for position updates from other swarm members.
 * @param msg The position received from another CPS.
 */
void swarm_position_callback (cpswarm_msgs::Position msg) {
    // the first messages are inaccurate, drop them
    if (pos_init > 0) {
        pos_init -= 1;
        return;
    }

    // uuid of the sending swarm member
    string uuid = msg.swarmio.node;

    // add new swarm member
    if (swarm_positions.count(uuid) <= 0) {
        cartesian_vector_t data;
        data.uuid = uuid;
        swarm_positions.emplace(uuid, data);
    }
    if (swarm_positions_rel.count(uuid) <= 0) {
        polar_vector_t data;
        data.uuid = uuid;
        swarm_positions_rel.emplace(uuid, data);
    }

    // update swarm member
    swarm_positions[uuid].x.push_back(msg.pose.position.x);
    swarm_positions[uuid].y.push_back(msg.pose.position.y);
    swarm_positions[uuid].stamp = Time::now();
    swarm_positions_rel[uuid].mag.push_back(hypot(msg.pose.position.x - pose.position.x, msg.pose.position.y - pose.position.y));
    swarm_positions_rel[uuid].dir.push_back(remainder((atan2(msg.pose.position.y - pose.position.y, msg.pose.position.x - pose.position.x) - get_yaw()), 2*M_PI));
    swarm_positions_rel[uuid].stamp = Time::now();

    // remove old samples
    while (swarm_positions[uuid].x.size() > sample_size)
        swarm_positions[uuid].x.erase(swarm_positions[uuid].x.begin());
    while(swarm_positions[uuid].y.size() > sample_size)
        swarm_positions[uuid].y.erase(swarm_positions[uuid].y.begin());
    while (swarm_positions_rel[uuid].mag.size() > sample_size)
        swarm_positions_rel[uuid].mag.erase(swarm_positions_rel[uuid].mag.begin());
    while(swarm_positions_rel[uuid].dir.size() > sample_size)
        swarm_positions_rel[uuid].dir.erase(swarm_positions_rel[uuid].dir.begin());
}

/**
 * @brief Callback function for velocity updates from other swarm members.
 * @param msg The velocity received from another CPS.
 */
void swarm_velocity_callback (cpswarm_msgs::Velocity msg) {
    // the first messages are inaccurate, drop them
    if (vel_init > 0) {
        vel_init -= 1;
        return;
    }

    // uuid of the sending swarm member
    string uuid = msg.swarmio.node;

    // add new swarm member
    if (swarm_velocities.count(uuid) <= 0) {
        polar_vector_t data;
        data.uuid = uuid;
        swarm_velocities.emplace(uuid, data);
    }

    // update swarm member
    cpswarm_msgs::Vector velocity = rel_velocity(msg.velocity.linear);
    swarm_velocities[uuid].mag.push_back(velocity.magnitude);
    swarm_velocities[uuid].dir.push_back(velocity.direction);
    swarm_velocities[uuid].stamp = Time::now();

    // remove old samples
    while (swarm_velocities[uuid].mag.size() > sample_size)
        swarm_velocities[uuid].mag.erase(swarm_velocities[uuid].mag.begin());
    while(swarm_velocities[uuid].dir.size() > sample_size)
        swarm_velocities[uuid].dir.erase(swarm_velocities[uuid].dir.begin());
}

/**
 * @brief A ROS node that exchanges relative kinematics between CPSs in a swarm.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char **argv)
{
    // init ros node
    init(argc, argv, "kinematics_exchanger");
    NodeHandle nh;

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    double timeout;
    nh.param(this_node::getName() + "/timeout", timeout, 20.0);
    nh.param(this_node::getName() + "/sample_size", sample_size, 5);
    nh.param(this_node::getName() + "/init", pos_init, 30);
    nh.param(this_node::getName() + "/init", vel_init, 30);

    // publishers and subscribers
    Subscriber pose_subscriber = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    Subscriber vel_subscriber = nh.subscribe("vel_provider/velocity", queue_size, vel_callback);
    Subscriber incoming_position_subscriber = nh.subscribe("bridge/events/position", queue_size, swarm_position_callback);
    Subscriber incoming_velocity_subscriber = nh.subscribe("bridge/events/velocity", queue_size, swarm_velocity_callback);
    Publisher outgoing_position_publisher = nh.advertise<cpswarm_msgs::Position>("position", queue_size);
    Publisher outgoing_velocity_publisher = nh.advertise<cpswarm_msgs::Velocity>("velocity", queue_size);
    Publisher incoming_position_publisher = nh.advertise<cpswarm_msgs::ArrayOfPositions>("swarm_position", queue_size);
    Publisher incoming_rel_position_publisher = nh.advertise<cpswarm_msgs::ArrayOfVectors>("swarm_position_rel", queue_size);
    Publisher incoming_rel_velocity_publisher = nh.advertise<cpswarm_msgs::ArrayOfVectors>("swarm_velocity_rel", queue_size);

    // init loop rate
    Rate rate(loop_rate);

    // init position and velocity
    pose_valid = false;
    vel_valid = false;
    while (ok() && (pose_valid == false || vel_valid == false)) {
        ROS_DEBUG_ONCE("Waiting for valid pose and velocity ...");
        rate.sleep();
        spinOnce();
    }

    // init swarm kinematics messages
    cpswarm_msgs::ArrayOfPositions swarm_position;
    cpswarm_msgs::ArrayOfVectors swarm_position_rel;
    cpswarm_msgs::ArrayOfVectors swarm_velocity_rel;

    // continuously exchange kinematics between swarm members
    while (ok()) {
        // reset swarm kinematics messages
        swarm_position.positions.clear();
        swarm_position_rel.vectors.clear();
        swarm_velocity_rel.vectors.clear();

        // update absolute swarm position
        for (auto member=swarm_positions.begin(); member!=swarm_positions.end();) {
            // delete members that haven't updated their position lately
            if ((Time::now() - member->second.stamp) > Duration(timeout)) {
                member = swarm_positions.erase(member);
                continue;
            }

            // only consider members with enough samples
            if (member->second.x.size() >= sample_size) {
                // calculate average of swarm member position data
                cpswarm_msgs::Position position;
                position.header.stamp = Time::now();
                position.swarmio.node = member->first;

                // average coordinates
                position.pose.position.x = accumulate(member->second.x.begin(), member->second.x.end(), 0.0) / member->second.x.size();
                position.pose.position.y = accumulate(member->second.y.begin(), member->second.y.end(), 0.0) / member->second.y.size();

                // store averaged position of swarm member
                swarm_position.positions.push_back(position);
            }

            // next member
            ++member;
        }

        // update relative swarm position
        for (auto member=swarm_positions_rel.begin(); member!=swarm_positions_rel.end();) {
            // delete members that haven't updated their position lately
            if ((Time::now() - member->second.stamp) > Duration(timeout)) {
                member = swarm_positions_rel.erase(member);
                continue;
            }

            // only consider members with enough samples
            if (member->second.mag.size() >= sample_size) {
                // calculate average of swarm member position data
                cpswarm_msgs::VectorStamped position;
                position.header.stamp = Time::now();
                position.swarmio.node = member->first;

                // average magnitude
                position.vector.magnitude = accumulate(member->second.mag.begin(), member->second.mag.end(), 0.0) / member->second.mag.size();

                // average direction correctly using sines and cosines (avoid jump from 2π to 0)
                float sines = accumulate(member->second.dir.begin(), member->second.dir.end(), 0.0, acc_sin) / member->second.dir.size();
                float cosines = accumulate(member->second.dir.begin(), member->second.dir.end(), 0.0, acc_cos) / member->second.dir.size();
                position.vector.direction = atan2(sines, cosines);

                // store averaged position of swarm member
                swarm_position_rel.vectors.push_back(position);
            }

            // next member
            ++member;
        }

        // update swarm velocity
        for (auto member=swarm_velocities.begin(); member!=swarm_velocities.end();) {
            // delete members that haven't updated their velocity lately
            if ((Time::now() - member->second.stamp) > Duration(timeout)) {
                member = swarm_velocities.erase(member);
                continue;
            }

            // only consider members with enough samples
            if (member->second.mag.size() >= sample_size) {
                // calculate average of swarm member velocity data
                cpswarm_msgs::VectorStamped velocity;
                velocity.header.stamp = Time::now();
                velocity.swarmio.node = member->first;

                // average magnitude
                velocity.vector.magnitude = accumulate(member->second.mag.begin(), member->second.mag.end(), 0.0) / member->second.mag.size();

                // average direction correctly using sines and cosines (avoid jump from 2π to 0)
                float sines = accumulate(member->second.dir.begin(), member->second.dir.end(), 0.0, acc_sin) / member->second.dir.size();
                float cosines = accumulate(member->second.dir.begin(), member->second.dir.end(), 0.0, acc_cos) / member->second.dir.size();
                velocity.vector.direction = atan2(sines, cosines);

                // store averaged velocity of swarm member
                swarm_velocity_rel.vectors.push_back(velocity);
            }

            // next member
            ++member;
        }

        // publish swarm kinematics locally
        incoming_position_publisher.publish(swarm_position);
        incoming_rel_position_publisher.publish(swarm_position_rel);
        incoming_rel_velocity_publisher.publish(swarm_velocity_rel);

        // publish local kinematics to swarm
        cpswarm_msgs::Position position;
        position.header.stamp = Time::now();
        position.swarmio.name = "position";
        position.pose = pose;
        outgoing_position_publisher.publish(position);
        cpswarm_msgs::Velocity velocity;
        velocity.header.stamp = Time::now();
        velocity.swarmio.name = "velocity";
        velocity.velocity = velo;
        outgoing_velocity_publisher.publish(velocity);

        rate.sleep();
        spinOnce();
    }
}
