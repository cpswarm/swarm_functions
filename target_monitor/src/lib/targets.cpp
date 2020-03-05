#include <cpswarm_msgs/TargetTracking.h>
#include "lib/targets.h"

targets::targets ()
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    nh.param(this_node::getName() + "/fov", fov, 0.5);

    // uuid of this cps
    cps = "";

    // read done targets from parameter server
    vector<int> done;
    nh.getParam(this_node::getNamespace() + "/targets_done", done);
    for (int t : done) {
        target_map.emplace(piecewise_construct, forward_as_tuple(t), forward_as_tuple(make_shared<target>(t, TARGET_DONE)));
    }

    // publishers and subscribers
    tracking_pub = nh.advertise<cpswarm_msgs::TargetTracking>("target_tracking", queue_size, true);
    target_found_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_found", queue_size);
    target_update_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_update", queue_size);
    target_lost_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_lost", queue_size);
    target_done_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_done", queue_size, true);

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
        new_target_pose.orientation.w = 1;
        simulated_targets.emplace(piecewise_construct, forward_as_tuple(i), forward_as_tuple(make_shared<target>(i, TARGET_UNKNOWN, new_target_pose)));
    }
}

void targets::update (geometry_msgs::Pose pose)
{
    // check if a target is lost
    for (auto t : target_map) {
        // update target and inform others in case target is lost
        if (t.second->lost()) {
            publish_event("target_lost", t.first);
        }
    }

    // check if a new target is found in simulation
    for (auto t : simulated_targets) {
        // target pose
        geometry_msgs::Pose t_pose = t.second->get_pose();

        ROS_DEBUG("Target %d distance %.2f < %.2f", t.first, hypot(pose.position.x - t_pose.position.x, pose.position.y - t_pose.position.y), fov);

        // target is within camera fov
        if (hypot(pose.position.x - t_pose.position.x, pose.position.y - t_pose.position.y) <= fov) {
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

    // existing target
    if (target_map.count(msg.id) > 0) {
        // get previous state of target
        target_state_t prev_state = target_map[msg.id]->get_state();

        // update target
        target_map[msg.id]->update(state, pose, msg.header.stamp);

        // publish event for certain state transitions
        if ((prev_state == TARGET_ASSIGNED || prev_state == TARGET_TRACKED) && state == TARGET_DONE && msg.swarmio.node != "") // only if incoming event
            publish_event("target_done", msg.id);
        else if (prev_state == TARGET_ASSIGNED && state == TARGET_TRACKED)
            publish_event("target_update", msg.id);
        else if (prev_state == TARGET_TRACKED && state == TARGET_TRACKED)
            publish_event("target_update", msg.id);
        else if (prev_state == TARGET_KNOWN && state == TARGET_TRACKED)
            publish_event("target_found", msg.id);
        else if (prev_state == TARGET_LOST && state == TARGET_TRACKED)
            publish_event("target_found", msg.id);
        else
            ROS_DEBUG("Not publishing event for target");
    }

    // new target
    else {
        // add to target map
        target_map.emplace(piecewise_construct, forward_as_tuple(msg.id), forward_as_tuple(make_shared<target>(msg.id, state, pose, msg.header.stamp)));

        // publish event if target has been found by this cps
        if (state == TARGET_TRACKED) {
            publish_event("target_found", msg.id);
        }
    }
}

void targets::publish_event (string event, int id)
{
    // create target position event
    cpswarm_msgs::TargetPositionEvent target;
    geometry_msgs::PoseStamped ps;
    ps.pose = target_map[id]->get_pose();
    ps.header.frame_id = "local_origin_ned";
    target.pose = ps;
    target.header.stamp = Time::now();
    target.swarmio.name = event;
    target.id = id;

    // publish target position event
    if (event == "target_found")
        target_found_pub.publish(target);

    else if (event == "target_update")
        target_update_pub.publish(target);

    else if (event == "target_lost")
        target_lost_pub.publish(target);

    else if (event == "target_done")
        target_done_pub.publish(target);

    else
        ROS_ERROR("Not publishing invalid event %s!", event.c_str());
}

geometry_msgs::Transform targets::transform (geometry_msgs::Pose p1, geometry_msgs::Pose p2) const
{
    // orientation of first point
    tf2::Quaternion orientation1;
    tf2::fromMsg(p1.orientation, orientation1);

    // relative coordinates of second point
    double dx = p2.position.x - p1.position.x;
    double dy = p2.position.y - p1.position.y;
    double distance = hypot(dx, dy);
    double direction = (M_PI / 2.0) - tf2::getYaw(orientation1) + atan2(dy, dx);

    // compute transform
    geometry_msgs::Transform tf;

    // translation
    tf.translation.x = -distance * cos(direction); // x is inverted in tracking camera tf
    tf.translation.y = distance * sin(direction);

    // rotation
    p1.orientation.w *= -1; // invert p1
    tf2::fromMsg(p1.orientation, orientation1);
    tf2::Quaternion orientation2;
    tf2::fromMsg(p2.orientation, orientation2);
    tf2::Quaternion rotation = orientation2 * orientation1;
    tf.rotation = tf2::toMsg(rotation);

    return tf;
}

void targets::uuid_callback (const swarmros::String::ConstPtr& msg)
{
    cps = msg->value;
}
