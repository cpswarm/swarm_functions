#include "coverage_path.h"

/**
 * @brief Generate an optimal coverage path for a given area.
 * @param start The starting position of the path.
 * @return Whether the path has been generated successfully.
 */
bool generate_path (geometry_msgs::Point start)
{
    // get area to cover
    nav_msgs::GetMap map;
    if (map_getter.call(map) == false){
        ROS_ERROR("Failed to get the assigned map, cannot compute coverage path!");
        return false;
    }

    ROS_INFO("Generate new coverage path...");

    // construct minimum spanning tree
    ROS_DEBUG("Construct minimum-spanning-tree...");
    tree.initialize_graph(map.response.map);
    tree.construct();

    // visualize path
    if (visualize)
        mst_publisher.publish(tree.get_tree());

    // generate path
    ROS_DEBUG("Generate coverage path...");
    path.initialize_graph(map.response.map);
    path.initialize_tree(tree.get_mst_edges());
    path.generate_path(start);

    // visualize path
    if (visualize)
        path_publisher.publish(path.get_path());

    reconfigure = false;

    return true;
}

/**
 * @brief Callback function to get the coverage path.
 * @param req Path planning request that is ignored.
 * @param res The coverage path.
 * @return Whether the request succeeded.
 */
bool get_path (nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
{
    // compute new path if swarm configuration changed
    if (reconfigure) {
        if (generate_path(req.start.pose.position) == false)
            return false;
    }

    // return coverage path
    res.plan = path.get_path();

    return true;
}

/**
 * @brief Callback function to get the current waypoint of the path.
 * @param req Empty get waypoint request.
 * @param res The current waypoint.
 * @return Whether the request succeeded.
 */
bool get_waypoint (cpswarm_msgs::GetWaypoint::Request &req, cpswarm_msgs::GetWaypoint::Response &res)
{
    // compute new path if swarm configuration changed
    if (reconfigure) {
        if (generate_path(req.position) == false)
            return false;
    }

    // return waypoint
    res.point = path.get_waypoint(req.position, req.tolerance);
    res.valid = path.valid();

    // visualize waypoint
    if (visualize) {
        geometry_msgs::PointStamped wp;
        wp.header.stamp = Time::now();
        wp.header.frame_id = "local_origin_ned";
        wp.point = res.point;
        wp_publisher.publish(wp);
    }

    return true;
}

/**
 * @brief Callback function for state updates.
 * @param msg State received from the CPS.
 */
void state_callback (const cpswarm_msgs::StateEvent::ConstPtr& msg)
{
    // store new position and orientation in class variables
    state = msg->state;

    // valid state received
    if (msg->header.stamp.isValid())
        state_valid = true;
}

/**
 * @brief Callback function to receive the states of the other CPSs.
 * @param msg UUIDs and states of the other CPSs.
 */
void swarm_callback (const cpswarm_msgs::ArrayOfStates::ConstPtr& msg)
{
    // update cps uuids
    for (auto cps : msg->states) {
        // only consider cpss in the same state, i.e., coverage
        if (cps.state != state)
            continue;

        // index of cps in map
        auto idx = swarm.find(cps.swarmio.node);

        // add new cps
        if (idx == swarm.end()) {
            swarm.emplace(cps.swarmio.node, Time::now());

            // recompute path
            reconfigure = true;
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

            // recompute path
            reconfigure = true;
        }
        else {
            ++cps;
        }
    }

    swarm_valid = true;
}

/**
 * @brief A ROS node that computes the optimal paths for area coverage with a swarm of CPSs.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char **argv)
{
    // init ros node
    init(argc, argv, "coverage_path");
    NodeHandle nh;

    // init global variables
    reconfigure = true;

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/swarm_timeout", swarm_timeout, 5.0);
    nh.param(this_node::getName() + "/visualize", visualize, false);
    bool divide_area;
    nh.param(this_node::getName() + "/divide_area", divide_area, false);

    // initialize flags
    state_valid = false;
    swarm_valid = false;

    // publishers, subscribers, and service clients
    Subscriber state_subscriber = nh.subscribe("state", queue_size, state_callback);
    Subscriber swarm_subscriber = nh.subscribe("swarm_state", queue_size, swarm_callback);
    if (visualize) {
        path_publisher = nh.advertise<nav_msgs::Path>("coverage_path/path", queue_size, true);
        wp_publisher = nh.advertise<geometry_msgs::PointStamped>("coverage_path/waypoint", queue_size, true);
        mst_publisher = nh.advertise<geometry_msgs::PoseArray>("coverage_path/mst", queue_size, true);
    }
    if (divide_area)
        map_getter = nh.serviceClient<nav_msgs::GetMap>("area/assigned");
    else
        map_getter = nh.serviceClient<nav_msgs::GetMap>("area/get_map");
    map_getter.waitForExistence();

    // init loop rate
    Rate rate(loop_rate);

    // init state
    while (ok() && state_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid state information...");
        rate.sleep();
        spinOnce();
    }

    // init swarm
    while (ok() && swarm_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid swarm information...");
        rate.sleep();
        spinOnce();
    }

    // provide coverage path services
    ServiceServer path_service = nh.advertiseService("coverage_path/path", get_path);
    ServiceServer wp_service = nh.advertiseService("coverage_path/waypoint", get_waypoint);
    spin();

    return 0;
}
