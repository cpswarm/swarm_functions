#include "area_division.h"

/**
 * @brief Switch to the synchronization state.
 */
void to_sync ()
{
    state = SYNC;
    ROS_DEBUG("Start synchronizing...");

    // reset information about swarm
    swarm_pose.clear();

    // stop moving
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = Time::now();
    goal_pose.pose = pose;
    pos_pub.publish(goal_pose);

    // start of synchronization time window
    sync_start = Time::now();
}

/**
 * @brief Callback function for behavior state updates.
 * @param msg State received from the CPS.
 */
void behavior_state_callback (const cpswarm_msgs::StateEvent::ConstPtr& msg)
{
    // store new state in class variables
    behavior = msg->state;

    // valid state received
    if (msg->header.stamp.isValid())
        behavior_valid = true;

    // cps switched to a behavior state that requires area division
    if (state == IDLE && find(behaviors.begin(), behaviors.end(), behavior) != behaviors.end())
        state = INIT;

    // cps switched to a behavior state that does not require area division
    if (state == ACTIVE && find(behaviors.begin(), behaviors.end(), behavior) == behaviors.end())
        state = DEINIT;
}

/**
 * @brief Callback function to receive area division requests from other CPSs.
 * @param msg UUIDs and position of the other CPS.
 */
void division_callback (const cpswarm_msgs::AreaDivisionEvent::ConstPtr& msg)
{
    // only divide area if active
    if (state == ACTIVE)
        to_sync();

    // only synchronize if ready
    if (state != SYNC)
        return;

    // check if cps is already known
    auto idx = swarm_pose.find(msg->swarmio.node);

    // add new cps
    if (idx == swarm_pose.end()) {
        ROS_DEBUG("Add CPS %s", msg->swarmio.node.c_str());

        // add cps
        swarm_pose.emplace(msg->swarmio.node, msg->pose);
    }

    // update existing cps
    else {
        ROS_DEBUG("Update CPS %s", msg->swarmio.node.c_str());

        // update cps
        idx->second.header.stamp = Time::now();
        idx->second = msg->pose;
    }
}

/**
 * @brief Callback function for position updates.
 * @param msg Position received from the CPS.
 */
void pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // store new position and orientation in class variables
    pose = msg->pose;

    // valid pose received
    if (msg->header.stamp.isValid())
        pose_valid = true;
}

/**
 * @brief Callback function to receive the states of the other CPSs.
 * @param msg UUIDs and states of the other CPSs.
 */
void swarm_state_callback (const cpswarm_msgs::ArrayOfStates::ConstPtr& msg)
{
    // update cps uuids
    for (auto cps : msg->states) {
        // only consider cpss in the same behavior states
        if (find(behaviors.begin(), behaviors.end(), cps.state) == behaviors.end())
            continue;

        // index of cps in map
        auto idx = swarm.find(cps.swarmio.node);

        // add new cps
        if (idx == swarm.end()) {
            swarm.emplace(cps.swarmio.node, Time::now());

            ROS_DEBUG("New CPS %s", cps.swarmio.node.c_str());
        }

        // update existing cps
        else {
            idx->second = Time::now();
        }
    }

    // remove old cps
    for (auto cps=swarm.cbegin(); cps!=swarm.cend();) {
        if (cps->second + Duration(swarm_timeout) < Time::now()) {
            ROS_DEBUG("Remove CPS %s", cps->first.c_str());
            swarm.erase(cps++);

            // divide area
            if (state == ACTIVE)
                to_sync();
        }
        else {
            ++cps;
        }
    }

    swarm_valid = true;
}

/**
 * @brief Callback function to receive the UUID from the communication library.
 * @param msg UUID of this node.
 */
void uuid_callback (const swarmros::String::ConstPtr& msg)
{
    uuid = msg->value;
}

/**
 * @brief Initialize this node.
 */
void init ()
{
    NodeHandle nh;

    // initialize flags
    uuid = "";
    pose_valid = false;
    swarm_valid = false;

    // publishers, subscribers, and service clients
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    uuid_sub = nh.subscribe("bridge/uuid", queue_size, uuid_callback);
    pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    swarm_sub = nh.subscribe("swarm_state", queue_size, swarm_state_callback);
    division_sub = nh.subscribe("bridge/events/area_division", queue_size, division_callback);
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", queue_size, true);
    swarm_pub = nh.advertise<cpswarm_msgs::AreaDivisionEvent>("area_division", queue_size, true);
    area_pub = nh.advertise<nav_msgs::OccupancyGrid>("area/assigned", queue_size, true);
    if (visualize) {
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("area/unassigned", queue_size, true);
    }

    // init uuid
    while (ok() && uuid == "") {
        ROS_DEBUG_ONCE("Waiting for UUID...");
        rate->sleep();
        spinOnce();
    }

    // init position
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid position information...");
        rate->sleep();
        spinOnce();
    }

    // init swarm
    while (ok() && swarm_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid swarm information...");
        rate->sleep();
        spinOnce();
    }

    // create area division object
    division = new area_division();

    // start area division
    to_sync();
}

/**
 * @brief Shutdown ROS communication.
 */
void deinit ()
{
    // shutdown ros communication
    uuid_sub.shutdown();
    pose_sub.shutdown();
    swarm_sub.shutdown();
    division_sub.shutdown();
    pos_pub.shutdown();
    swarm_pub.shutdown();
    area_pub.shutdown();
    map_pub.shutdown();

    // destroy optimizer
    delete division;

    state = IDLE;
}

/**
 * @brief Synchronize The CPSs by exchanging an event.
 */
void sync ()
{
    // publish synchronization event for some time
    if (sync_start + Duration(swarm_timeout) > Time::now()) {
        cpswarm_msgs::AreaDivisionEvent event;
        event.header.stamp = Time::now();
        event.swarmio.name = "area_division";
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "local_origin_ned";
        ps.pose = pose;
        event.pose = ps;
        swarm_pub.publish(event);
    }

    // divide area
    else
        state = DIVIDE;
}

/**
 * @brief Rotate a point by a given angle around the origin.
 * @param point The point to rotate.
 * @param angle The angle by which to rotate the point.
 * @return The rotated point.
 */
geometry_msgs::Point rotate (geometry_msgs::Point point, double angle)
{
    // no rotation required
    if (angle < 0.0001)
        return point;

    // return point around origin
    geometry_msgs::Point rotated;
    rotated.x = point.x*cos(angle) - point.y*sin(angle);
    rotated.y = point.x*sin(angle) + point.y*cos(angle);
    return rotated;
}

/**
 * @brief Divide the area of the grid map equally among multiple CPSs.
 */
void divide_area ()
{
    NodeHandle nh;

    // get map
    cpswarm_msgs::GetMap gm;
    gm.request.rotate = true; // align bottom edge of map horizontally
    gm.request.translate = true; // make map origin even
    gm.request.resolution = resolution; // downsample to given resolution
    ServiceClient gm_cli = nh.serviceClient<cpswarm_msgs::GetMap>("area/get_map");
    ROS_DEBUG("Wait for area/get_map service...");
    gm_cli.waitForExistence();
    ROS_DEBUG("area/get_map service available");
    gm_cli.call(gm); // call service by area provider
    // get result
    nav_msgs::OccupancyGrid gridmap = gm.response.map;
    double angle = gm.response.rotation;
    geometry_msgs::Vector3 translation = gm.response.translation;

    if (visualize)
        map_pub.publish(gridmap);

    // convert swarm pose to grid
    map<string, vector<int>> swarm_grid;
    for (auto cps : swarm_pose) {
        geometry_msgs::Point rotated = rotate(cps.second.pose.position, angle);
        rotated.x += translation.x;
        rotated.y += translation.y;
        vector<int> pos(2);
        pos[0] = int(round((rotated.x - gridmap.info.origin.position.x) / gridmap.info.resolution));
        pos[1] = int(round((rotated.y - gridmap.info.origin.position.y) / gridmap.info.resolution));
        swarm_grid.emplace(cps.first, pos);
        ROS_DEBUG("Other CPS %s at (%d,%d)", cps.first.c_str(), pos[0], pos[1]);
    }

    // add this robot to swarm grid
    geometry_msgs::Point rotated = rotate(pose.position, angle);
    rotated.x += translation.x;
    rotated.y += translation.y;
    vector<int> pos(2);
    pos[0] = int(round((rotated.x - gridmap.info.origin.position.x) / gridmap.info.resolution));
    pos[1] = int(round((rotated.y - gridmap.info.origin.position.y) / gridmap.info.resolution));
    swarm_grid.emplace(uuid, pos);
    ROS_DEBUG("Me %s at (%d,%d)", uuid.c_str(), pos[0], pos[1]);

    // divide area
    ROS_INFO("Dividing area...");
    vector<signed char, allocator<signed char>> map = gridmap.data;
    division->initialize_map((int)gridmap.info.height, (int)gridmap.info.width, map);
    division->initialize_cps(swarm_grid);
    division->divide();

    // publish result
    area_pub.publish(division->get_grid(gridmap, uuid));

    // wait a bit to avoid directly redividing again
    Time sleep_start = Time::now();
    while (ok() && sleep_start + Duration(swarm_timeout) > Time::now()) {
        spinOnce();
        rate->sleep();
    }

    // area division done
    state = ACTIVE;
}

/**
 * @brief A ROS node that divides the available area among a swarm of CPSs.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char **argv)
{
    // init ros node
    init(argc, argv, "area_division");
    NodeHandle nh;

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.5);
    rate = new Rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    nh.param(this_node::getName() + "/resolution", resolution, 1.0);
    nh.param(this_node::getName() + "/swarm_timeout", swarm_timeout, 5.0);
    nh.param(this_node::getName() + "/visualize", visualize, false);
    nh.getParam(this_node::getName() + "/states", behaviors);

    // initially, this cps does not perform area division
    state = IDLE;

    // init behavior state
    behavior_valid = false;
    Subscriber behavior_state_subscriber = nh.subscribe("state", queue_size, behavior_state_callback);
    while (ok() && behavior_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid behavior information...");
        rate->sleep();
        spinOnce();
    }

    // check if area division is necessary
    while (ok()) {
        spinOnce();

        switch (state) {
            // start up this node functionality
            case INIT:
                init();
                break;

            // synchronize with the other cpss
            case SYNC:
                sync();
                break;

            // divide area
            case DIVIDE:
                divide_area();
                break;

            // shutdown this node functionality
            case DEINIT:
                deinit();
                break;
        }

        rate->sleep();
    }

    // clean up
    delete rate;

    return 0;
}
