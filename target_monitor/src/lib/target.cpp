#include "lib/target.h"

target::target () : target(-1, TARGET_UNKNOWN)
{
}

target::target (const target& t) : target(t.id, t.state, t.pose, t.stamp)
{
}

target::target (unsigned int id, target_state_t state) : target(id, state, geometry_msgs::Pose())
{
}

target::target (unsigned int id, target_state_t state, geometry_msgs::Pose pose) : target(id, state, pose, Time::now())
{
}

target::target (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp) : id(id), state(state), pose(pose), stamp(stamp)
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    double timeout;
    nh.param(this_node::getName() + "/tracking_timeout", timeout, 5.0);
    this->timeout = Duration(timeout);
    nh.param(this_node::getName() + "/target_tolerance", target_tolerance, 0.1);

    // initialize publishers
    target_found_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_found", queue_size, true);
    target_update_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_update", queue_size, true);
    target_lost_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_lost", queue_size, true);
    target_done_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_done", queue_size, true);

    // init loop rate
    rate = new Rate(loop_rate);

    // inform others about newly found target
    if (state == TARGET_TRACKED) {
        // wait until subscriber is connected
        while (ok() && target_found_pub.getNumSubscribers() <= 0)
            rate->sleep();

        ROS_DEBUG("Found target %d at (%.2f,%.2f)", id, pose.position.x, pose.position.y);

        // publish event
        cpswarm_msgs::TargetPositionEvent target;
        geometry_msgs::PoseStamped ps;
        ps.pose = pose;
        ps.header.frame_id = "local_origin_ned";
        target.pose = ps;
        target.header.stamp = Time::now();
        target.swarmio.name = "target_found";
        target.id = id;
        target_found_pub.publish(target);
    }
}

target::~target ()
{
    delete rate;
}

geometry_msgs::Pose target::get_pose ()
{
    return pose;
}

void target::lost ()
{
    // target is being tracked
    if (state == TARGET_TRACKED) {
        // no updates received within timeout
        if (stamp + timeout < Time::now()) {
            // update target information
            state = TARGET_LOST;

            // wait until subscriber is connected
            while (ok() && target_lost_pub.getNumSubscribers() <= 0)
                rate->sleep();

            // publish event
            cpswarm_msgs::TargetPositionEvent target;
            geometry_msgs::PoseStamped ps;
            ps.pose = pose;
            ps.header.frame_id = "local_origin_ned";
            target.pose = ps;
            target.header.stamp = Time::now();
            target.swarmio.name = "target_lost";
            target.id = id;
            target_lost_pub.publish(target);
        }
    }
}

void target::operator= (const target& t)
{
    id = t.id;
    update(t.state, t.pose, t.stamp);
}

void target::update (target_state_t state, geometry_msgs::Pose pose, Time stamp)
{
    // this target has been completed already, nothing to do
    if (this->state == TARGET_DONE) {
        ROS_DEBUG("Target %d already done", id);
        return;
    }

    // target completed
    if (state == TARGET_DONE) {
        // store target id in parameter server
        vector<int> done;
        nh.getParam(this_node::getNamespace() + "/targets_done", done);
        done.push_back(id);
        nh.setParam(this_node::getNamespace() + "/targets_done", done);

        // publish event
        cpswarm_msgs::TargetPositionEvent target;
        geometry_msgs::PoseStamped ps;
        ps.pose = pose;
        ps.header.frame_id = "local_origin_ned";
        target.pose = ps;
        target.header.stamp = Time::now();
        target.swarmio.name = "target_done";
        target.id = id;
        target_done_pub.publish(target);

        // update target information
        this->state = state;
        this->pose = pose;
        this->stamp = stamp;

        ROS_DEBUG("Target %d done", id);

        return;
    }

    // this target has been assigned already, nothing to do except setting it to done
    if (this->state == TARGET_ASSIGNED) {
        ROS_DEBUG("Target %d already assigned", id);
        return;
    }

    // target has been assigned to a cps, stop tracking it
    if (state == TARGET_ASSIGNED) {
        // update target information
        this->state = state;
        this->stamp = stamp;

        // not necessary to publish event, since assignment is a swarm scope event already

        ROS_DEBUG("Target %d assigned", id);

        return;
    }

    // target is being tracked, update for already known target
    if (state == TARGET_TRACKED) {
        // compute distance that target moved
        double moved = hypot(last_pose.position.x - pose.position.x, last_pose.position.y - pose.position.y);

        // new target found
        if (this->state != TARGET_TRACKED) {
            // wait until subscriber is connected
            while (ok() && target_update_pub.getNumSubscribers() <= 0)
                rate->sleep();

            ROS_DEBUG("Found target %d at (%.2f,%.2f)", id, pose.position.x, pose.position.y);

            // publish event
            cpswarm_msgs::TargetPositionEvent target;
            geometry_msgs::PoseStamped ps;
            ps.pose = pose;
            ps.header.frame_id = "local_origin_ned";
            target.pose = ps;
            target.header.stamp = Time::now();
            target.swarmio.name = "target_found";
            target.id = id;
            target_found_pub.publish(target);
        }

        // target has moved enough for update
        if (moved > target_tolerance) {
            // wait until subscriber is connected
            while (ok() && target_update_pub.getNumSubscribers() <= 0)
                rate->sleep();

            // publish event
            cpswarm_msgs::TargetPositionEvent target;
            geometry_msgs::PoseStamped ps;
            ps.pose = pose;
            ps.header.frame_id = "local_origin_ned";
            target.pose = ps;
            target.header.stamp = Time::now();
            target.swarmio.name = "target_update";
            target.id = id;
            target_update_pub.publish(target);

            ROS_DEBUG("Target %d update", id);
        }

        // store current pose
        last_pose = pose;

        // update target information
        this->state = state;
        this->pose = pose;
        this->stamp = stamp;

        return;
    }
}
