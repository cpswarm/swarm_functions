#include "lib/targets.h"

targets::targets ()
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/fov", fov, 0.64);

    // uuid of this cps
    cps = "";

    // read done targets from parameter server
    vector<int> done;
    nh.getParam(this_node::getNamespace() + "/targets_done", done);
    for (int t : done) {
        target_map.emplace(piecewise_construct, forward_as_tuple(t), forward_as_tuple(t, TARGET_DONE));
    }

    // publishers and subscribers
    tracking_pub = nh.advertise<cpswarm_msgs::TargetTracking>("target_tracking", queue_size, true);
    Subscriber uuid_sub = nh.subscribe("bridge/uuid", queue_size, &targets::uuid_callback, this);

    // init uuid
    while (ok() && cps == "") {
        rate.sleep();
        spinOnce();
    }
}

void targets::simulate ()
{
    // read all potential targets from parameter file
    vector<double> targets_x;
    vector<double> targets_y;
    nh.getParam(this_node::getName() + "/targets_x", targets_x);
    nh.getParam(this_node::getName() + "/targets_y", targets_y);
    if (targets_x.size() != targets_y.size()) {
        ROS_FATAL("Invalid targets specified! Exiting...");
        shutdown();
    }
    else if (targets_x.size() < 1)
        ROS_INFO("There are no targets!");
    for (int i = 0; i < targets_x.size(); ++i) {
        ROS_DEBUG("Target %d at [%.2f, %.2f]", i, targets_x[i], targets_y[i]);
        geometry_msgs::Pose new_target_pose;
        new_target_pose.position.x = targets_x[i];
        new_target_pose.position.y = targets_y[i];
        simulated_targets.emplace(piecewise_construct, forward_as_tuple(i), forward_as_tuple(i, TARGET_UNKNOWN, new_target_pose));
    }
}

void targets::update (geometry_msgs::Pose pose)
{
    // check if a target is lost
    for (auto t : target_map) {
        // update target and inform others in case target is lost
        t.second.lost();
    }

    // check if a new target is found in simulation
    for (auto t : simulated_targets) {
        // target pose
        geometry_msgs::Pose t_pose = t.second.get_pose();

        // visible distance from drone with fov at current altitude
        double dist = pose.position.z * tan(fov / 2.0);

        ROS_DEBUG("Target %d distance %.2f < %.2f", t.first, hypot(pose.position.x - t_pose.position.x, pose.position.y - t_pose.position.y), dist);

        // target is within camera fov
        if (hypot(pose.position.x - t_pose.position.x, pose.position.y - t_pose.position.y) <= dist) {
            // publish tracking information
            cpswarm_msgs::TargetTracking track;
            track.header.stamp = Time::now();
            track.id = t.first;
            track.tf = transform(pose, t_pose);
            tracking_pub.publish(track);
        }
    }
}

void targets::update (cpswarm_msgs::TargetPositionEvent msg, target_state_t state)
{
    // determine target pose
    geometry_msgs::Pose pose;
    pose = msg.pose.pose;

    ROS_DEBUG("Target %d at [%.2f, %.2f]", msg.id, pose.position.x, pose.position.y);

    // update existing target
    if (target_map.count(msg.id) > 0) {
        target_map[msg.id].update(state, pose, msg.header.stamp);
    }

    // add new target
    else {
        target_map.emplace(piecewise_construct, forward_as_tuple(msg.id), forward_as_tuple(msg.id, state, pose, msg.header.stamp));
    }
}

geometry_msgs::Transform targets::transform (geometry_msgs::Pose p1, geometry_msgs::Pose p2) const
{
    // orientation of first point
    tf2::Quaternion orientation;
    tf2::fromMsg(p1.orientation, orientation);

    // relative coordinates of second point
    double dx = p2.position.x - p1.position.x;
    double dy = p2.position.y - p1.position.y;
    double distance = hypot(dx, dy);
    double direction = (M_PI / 2.0) - tf2::getYaw(orientation) + atan2(dy, dx);

    // compute transform
    geometry_msgs::Transform tf;
    tf.translation.x = -distance * cos(direction); // x is inverted in tracking camera tf
    tf.translation.y = distance * sin(direction);

    return tf;
}

void targets::uuid_callback (const swarmros::String::ConstPtr& msg)
{
    cps = msg->value;
}
