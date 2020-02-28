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

    // init loop rate
    rate = new Rate(loop_rate);

    ROS_INFO("Added target %d in state %d", id, state);
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
        }

        return;
    }

    // target is being tracked
    else if (this->state == TARGET_TRACKED) {
        // send update
        if (state == TARGET_TRACKED) {
            ROS_DEBUG("Target %d: tracked, update", id);

            // update target information
            this->state = state;
            this->pose = pose;
            this->stamp = stamp;
        }

        // target has been assigned to another cps
        else if (state == TARGET_ASSIGNED) {
            ROS_DEBUG("Target %d: tracked --> assigned", id);

            // update target information
            this->state = state;
            this->stamp = stamp;
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
        }

        // target has been assigned to another cps
        else if (state == TARGET_ASSIGNED) {
            ROS_DEBUG("Target %d: known --> assigned", id);

            // update target information
            this->state = state;
            this->stamp = stamp;
        }

        // target has been lost by another cps
        else if (state == TARGET_LOST) {
            ROS_DEBUG("Target %d: known --> lost", id);

            // update target information
            this->state = state;
            this->pose = pose;
            this->stamp = stamp;
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
        }

        // target has been found by this cps
        else if (state == TARGET_TRACKED) {
            ROS_DEBUG("Target %d: lost --> tracked", id);
        }

        return;
    }

    else {
        ROS_ERROR("Target %d: invalid state transition %d --> %d!", id, this->state, state);
    }
}
