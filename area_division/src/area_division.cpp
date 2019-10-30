#include "area_division.h"

/**
 * @brief Divide the area of the grid map equally among multiple CPSs.
 */
void divide_area ()
{
    // convert swarm pose to grid
    map<string, vector<int>> swarm_grid;
    for (auto cps : swarm_pose) {
        vector<int> pos(2);
        pos[0] = int(round((cps.second.pose.position.x - gridmap.info.origin.position.x) / gridmap.info.resolution));
        pos[1] = int(round((cps.second.pose.position.y - gridmap.info.origin.position.y) / gridmap.info.resolution));
        swarm_grid.emplace(cps.first, pos);
        ROS_DEBUG("Other CPS %s at (%d,%d)", cps.first.c_str(), pos[0], pos[1]);
    }

    // add this robot to swarm grid
    vector<int> pos(2);
    pos[0] = int(round((pose.position.x - gridmap.info.origin.position.x) / gridmap.info.resolution));
    pos[1] = int(round((pose.position.y - gridmap.info.origin.position.y) / gridmap.info.resolution));
    swarm_grid.emplace(uuid, pos);
    ROS_DEBUG("Me %s at (%d,%d)", uuid.c_str(), pos[0], pos[1]);

    // divide area
    ROS_INFO("Dividing area...");
    vector<signed char, allocator<signed char>> map = gridmap.data;
    division->initialize_map((int)gridmap.info.width, (int)gridmap.info.height, map);
    division->initialize_cps(swarm_grid);
    division->divide();

    // visualize area
    if (visualize)
        area_pub.publish(division->get_grid(gridmap, uuid));

    reconfigure = false;
}

/**
 * @brief Synchronize The CPSs by exchanging an event.
 */
void sync ()
{
    // start synchronization sequence
    if (sync_start + Duration(swarm_timeout) < Time::now()) {
        // stop moving
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.stamp = Time::now();
        goal_pose.pose = pose;
        pos_pub.publish(goal_pose);

        // reset information about swarm
        swarm_pose.clear();

        // start of synchronization time window
        sync_start = Time::now();

        // send event to swarm
        cpswarm_msgs::AreaDivisionEvent event;
        event.header.stamp = Time::now();
        event.swarmio.name = "area_division";
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "local_origin_ned";
        ps.pose = pose;
        event.pose = ps;
        swarm_pub.publish(event);
    }
}

/**
 * @brief Callback function to get the area assignment of this CPS.
 * @param req Empty request.
 * @param res The grid map assigned to this CPS.
 * @return Whether the request succeeded.
 */
bool get_area (nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
    // synchronize with swarm
    sync();

    // wait for swarm updates
    while (Time::now() <= sync_start + Duration(swarm_timeout)) {
        rate->sleep();
        spinOnce();
    }

    // swarm configuration changed
    if (reconfigure) {
        // divide area
        divide_area();
    }

    // return assigned area
    res.map = division->get_grid(gridmap, uuid);

    return true;
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
 * @brief Callback function to receive area division requests from other CPSs.
 * @param msg UUIDs and position of the other CPS.
 */
void swarm_callback (const cpswarm_msgs::AreaDivisionEvent::ConstPtr& msg)
{
    // synchronize with swarm
    sync();

    // recalculate area division
    reconfigure = true;

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
 * @brief Callback function to receive the grid map.
 * @param msg Merged grid map from all CPSs.
 */
void map_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // TODO: react to map changes, i.e., reconfigure = true
    gridmap = *msg;
    map_valid = true;
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

    // init global variables
    reconfigure = true;
    sync_start = Time::now();

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    nh.param(this_node::getName() + "/swarm_timeout", swarm_timeout, 5.0);
    nh.param(this_node::getName() + "/visualize", visualize, false);

    // initialize flags
    uuid = "";
    pose_valid = false;
    swarm_valid = false;
    map_valid = false;

    // publishers and subscribers
    Subscriber uuid_sub = nh.subscribe("bridge/uuid", queue_size, uuid_callback);
    Subscriber pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    Subscriber swarm_sub = nh.subscribe("bridge/events/area_division", queue_size, swarm_callback);
    Subscriber map_sub = nh.subscribe("area_provider/map", queue_size, map_callback); // TODO: use explored/merged map
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", queue_size, true);
    swarm_pub = nh.advertise<cpswarm_msgs::AreaDivisionEvent>("area_division", queue_size, true);
    if (visualize)
        area_pub = nh.advertise<nav_msgs::OccupancyGrid>("assigned_map", queue_size, true);

    // init loop rate
    rate = new Rate(loop_rate);

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

    // init map
    while (ok() && map_valid == false) {
        ROS_DEBUG_ONCE("Waiting for grid map...");
        rate->sleep();
        spinOnce();
    }

    // create area division object
    division = new area_division();

    // provide area service
    ServiceServer area_service = nh.advertiseService("area/assigned", get_area);
    spin();

    // clean up
    delete rate;
    delete division;

    return 0;
}
