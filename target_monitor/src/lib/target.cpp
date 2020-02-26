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
    target_found_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_found", queue_size);
    target_update_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_update", queue_size);
    target_lost_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_lost", queue_size);
    target_done_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_done", queue_size, true);

    // init loop rate
    rate = new Rate(loop_rate);

    ROS_DEBUG("Added target %d in state %d", id, state);

    // inform others about newly found target
    if (state == TARGET_TRACKED) {
        ROS_DEBUG("Target %d: unknown --> tracked, position (%.2f,%.2f)", id, pose.position.x, pose.position.y);

        // publish event
        publish_event("target_found");
    }

    else {
        ROS_DEBUG("Target %d: unknown --> known", id);
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
    if (state == TARGET_TRACKED || state == TARGET_ASSIGNED) {
        // no updates received within timeout
        if (stamp + timeout < Time::now()) {
            if (state == TARGET_TRACKED)
                ROS_INFO("Target %d: tracked --> lost", id);
            else if (state == TARGET_ASSIGNED)
                ROS_INFO("Target %d: assigned --> lost", id);
            else
                ROS_INFO("Target %d: lost", id);

            // update target information
            state = TARGET_LOST;

            // publish event
            publish_event("target_lost");
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
        ROS_DEBUG("Target %d: already done", id);
        return;
    }

    // this target has been assigned to another cps for completion but is still tracked
    else if (this->state == TARGET_ASSIGNED) {
        // update target information
        this->state = state;
        this->pose = pose;
        this->stamp = stamp;

        // target completed by the other cps
        if (state == TARGET_DONE) {
            ROS_DEBUG("Target %d: assigned --> done", id);

            // store target id in parameter server
            vector<int> done;
            nh.getParam(this_node::getNamespace() + "/targets_done", done);
            done.push_back(id);
            nh.setParam(this_node::getNamespace() + "/targets_done", done);

            // publish event
            publish_event("target_done");
        }

        // target still being tracked, send update
        else if (state == TARGET_TRACKED) {
            ROS_DEBUG("Target %d: assigned, update", id);

            // publish event
            publish_event("target_update");
        }

        return;
    }

    // target is being tracked
    else if (this->state == TARGET_TRACKED) {
        // send update
        if (state == TARGET_TRACKED) {
            // update target information
            this->state = state;
            this->pose = pose;
            this->stamp = stamp;

            // compute distance that target moved
            double moved = hypot(last_pose.position.x - pose.position.x, last_pose.position.y - pose.position.y);

            // target has moved enough for update
            if (moved > target_tolerance) {
                ROS_DEBUG("Target %d: tracked, update", id);

                // store current pose
                last_pose = pose;

                // publish event
                publish_event("target_update");
            }
        }

        // target has been assigned to another cps
        else if (state == TARGET_ASSIGNED) {
            ROS_DEBUG("Target %d: tracked --> assigned", id);

            // update target information
            this->state = state;
            this->stamp = stamp;

            // reaction to incoming swarm event, not necessary to publish event
        }

        return;
    }

    // target has been found by another cps
    else if (this->state == TARGET_KNOWN) {
        // target now found by this cps
        if (state == TARGET_TRACKED) {
            ROS_DEBUG("Target %d: known --> tracked, position (%.2f,%.2f)", id, pose.position.x, pose.position.y);

            // update target information
            this->state = state;
            this->pose = pose;
            this->stamp = stamp;

            // publish event
            publish_event("target_found");
        }

        // target has been assigned to another cps
        else if (state == TARGET_ASSIGNED) {
            ROS_DEBUG("Target %d: known --> assigned", id);

            // update target information
            this->state = state;
            this->stamp = stamp;

            // reaction to incoming swarm event, not necessary to publish event
        }

        // target has been lost by another cps
        else if (state == TARGET_LOST) {
            ROS_DEBUG("Target %d: known --> lost", id);

            // update target information
            this->state = state;
            this->pose = pose;
            this->stamp = stamp;

            // reaction to incoming swarm event, not necessary to publish event
        }

        return;
    }

    // target has been lost by another cps
    else if (this->state == TARGET_LOST) {
        // update target information
        this->state = state;
        this->pose = pose;
        this->stamp = stamp;

        // target has been found by another cps
        if (state == TARGET_KNOWN) {
            ROS_DEBUG("Target %d: lost --> known", id);

            // reaction to incoming swarm event, not necessary to publish event
        }

        // target has been found by this cps
        else if (state == TARGET_TRACKED) {
            ROS_DEBUG("Target %d: lost --> tracked", id);

            // publish event
            publish_event("target_found");
        }

        return;
    }

    else {
        ROS_ERROR("Target %d: invalid state transition %d --> %d!", id, this->state, state);
    }
}

void target::publish_event (string event)
{
    // create target position event
    cpswarm_msgs::TargetPositionEvent target;
    geometry_msgs::PoseStamped ps;
    ps.pose = pose;
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
