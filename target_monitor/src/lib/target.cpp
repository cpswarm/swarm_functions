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

target::target (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp) : target(id, state, pose, stamp, "")
{
}

target::target (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp, string cps) : id(id), state(state), pose(pose), stamp(stamp)
{
    // set variables
    if (cps != "")
        this->tracked_by.insert(cps);

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    double timeout;
    nh.param(this_node::getName() + "/tracking_timeout", timeout, 5.0);
    this->timeout = Duration(timeout);
    double time;
    nh.param(this_node::getName() + "/tracking_time", time, -1.0);
    this->time = Duration(time);
    nh.param(this_node::getName() + "/min_trackers", min_trackers, 1);
    nh.param(this_node::getName() + "/max_trackers", max_trackers, 3);

    // init loop rate
    rate = new Rate(loop_rate);

    ROS_INFO("Added target %d in state %d", id, state);
}

target::~target ()
{
    delete rate;
}

int target::get_num_trackers ()
{
    return tracked_by.size();
}

geometry_msgs::Pose target::get_pose ()
{
    return pose;
}

target_state_t target::get_state ()
{
    return state;
}

double target::get_time_need ()
{
    return time.toSec();
}

set<string> target::get_trackers ()
{
    return tracked_by;
}

bool target::help ()
{
    return tracked_by.size() < min_trackers;
}

bool target::lost ()
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

            return true;
        }
    }

    return false;
}

void target::operator= (const target& t)
{
    id = t.id;
    state = t.state;
    pose = t.pose;
    stamp = t.stamp;
    time = t.time;
    tracked_by = t.tracked_by;
}

bool target::overcrowded ()
{
    return tracked_by.size() > max_trackers;
}

void target::update (target_state_t state, geometry_msgs::Pose pose, Time stamp, string cps)
{
    // this target has been completed already, nothing to do
    if (this->state == TARGET_DONE) {
        ROS_DEBUG("Target %d: already done", id);
        return;
    }

    // update tracking cpss
    if (state == TARGET_KNOWN || state == TARGET_TRACKED) {
        this->tracked_by.insert(cps);
    }
    if (state == TARGET_UNKNOWN || state == TARGET_LOST || state == TARGET_DONE) {
        this->tracked_by.erase(cps);
    }

    // update target information
    if (state == TARGET_KNOWN || state == TARGET_TRACKED) {
        this->pose = pose;
        this->stamp = stamp;
        if (this->time.toSec() > 0) {
            // update time the target has been tracked
            Duration tracked = stamp - this->stamp;
            if (tracked < this->time)
                this->time -= tracked;
            else
                this->time = Duration(0);
        }
    }

    // this target has been assigned to another cps for completion but is still tracked
    if (this->state == TARGET_ASSIGNED) {
        // target completed by the other cps
        if (state == TARGET_DONE) {
            ROS_DEBUG("Target %d: assigned --> done", id);

            this->state = state;
        }
    }

    // target is being tracked
    else if (this->state == TARGET_TRACKED) {
        // target has been assigned to another cps
        if (state == TARGET_ASSIGNED) {
            ROS_DEBUG("Target %d: tracked --> assigned", id);

            this->state = state;
        }
    }

    // target has been found by another cps
    else if (this->state == TARGET_KNOWN) {
        // target now found by this cps
        if (state == TARGET_TRACKED) {
            ROS_DEBUG("Target %d: known --> tracked, position (%.2f,%.2f)", id, pose.position.x, pose.position.y);

            this->state = state;
        }

        // target has been assigned to another cps
        else if (state == TARGET_ASSIGNED) {
            ROS_DEBUG("Target %d: known --> assigned", id);

            // update target information
            this->state = state;
        }

        // target has been lost by another cps
        else if (state == TARGET_LOST) {
            ROS_DEBUG("Target %d: known --> lost", id);

            // update target information
            this->state = state;
        }
    }

    // target has been lost by another cps
    else if (this->state == TARGET_LOST) {
        // target has been found by another cps
        if (state == TARGET_KNOWN) {
            ROS_DEBUG("Target %d: lost --> known", id);

            this->state = state;
        }

        // target has been found by this cps
        else if (state == TARGET_TRACKED) {
            ROS_DEBUG("Target %d: lost --> tracked", id);

            this->state = state;
        }
    }

    else {
        ROS_ERROR("Target %d: invalid state transition %d --> %d!", id, this->state, state);
    }

    // target tracked long enough
    if (this->time.isZero() || this->state == TARGET_DONE) {
        ROS_DEBUG("Target %d: tracked --> assigned", id);

        this->state = TARGET_DONE;

        // store target id in parameter server
        vector<int> done;
        nh.getParam(this_node::getNamespace() + "/targets_done", done);
        done.push_back(id);
        nh.setParam(this_node::getNamespace() + "/targets_done", done);
    }
}
