#include "coverage_path.h"

/**
 * @brief Generate an optimal coverage path for a given area.
 * @param start The starting position of the path.
 * @param roi Pointer to a ROI to cover. Optional, if not given mission area is covered.
 * @return Whether the path has been generated successfully.
 */
bool generate_path (geometry_msgs::Point start, const vector<geometry_msgs::Point>* roi = nullptr)
{
    NodeHandle nh;

    ROS_DEBUG("Starting at (%.2f,%.2f)", start.x, start.y);

    // get area to cover
    cpswarm_msgs::GetMap get_map;
    get_map.request.rotate = true;
    get_map.request.translate = true;
    get_map.request.resolution = resolution;
    ServiceClient area_getter;
    // get a roi
    if (roi != nullptr && roi->size() > 2) {
        get_map.request.coords = *roi;
        area_getter = nh.serviceClient<cpswarm_msgs::GetMap>("rois/get_map");
        swarm_sub.shutdown(); // rois are not divided, stop listening to swarm changes
    }
    // get area divided among swarm
    else if (divide_area) {
        area_getter = nh.serviceClient<cpswarm_msgs::GetMap>("area/get_divided_map");
        swarm_sub = nh.subscribe("swarm_state", queue_size, swarm_state_callback); // using divided map, start listening to swarm changes
    }
    // get complete area
    else
        area_getter = nh.serviceClient<cpswarm_msgs::GetMap>("area/get_map");
    area_getter.waitForExistence();
    if (area_getter.call(get_map) == false) {
        ROS_ERROR("Failed to get area coordinates!");
        return false;
    }
    nav_msgs::OccupancyGrid area = get_map.response.map;

    ROS_INFO("Generate new coverage path...");

    // construct minimum spanning tree
    ROS_DEBUG("Construct minimum-spanning-tree...");
    spanning_tree tree;
    tree.initialize_graph(area, vertical);
    tree.construct();

    // visualize path
    if (visualize)
        mst_publisher.publish(tree.get_tree());

    // generate path
    ROS_DEBUG("Generate coverage path...");
    path.initialize_graph(area);
    path.initialize_map(area.info.origin.position, get_map.response.rotation, area.info.width, area.info.height);
    path.initialize_tree(tree.get_mst_edges());
    if (path.generate_path(start) == false)
        return false;
    if (turning_points)
        path.reduce();

    // visualize path
    if (visualize)
        path_publisher.publish(path.get_path());

    reconfigure = false;

    return true;
}

/**
 * @brief Callback to execute the action server that generates the path.
 * @param goal An empty action server goal.
 * @param as The action server object.
 */
bool generate_path_callback (const cpswarm_msgs::PathGenerationGoal::ConstPtr& goal, GenerationAction* as)
{
    bool success = generate_path(goal->start, &(goal->area));

    if (as->isPreemptRequested()) {
        as->setPreempted();
    }
    else if (success) {
        cpswarm_msgs::PathGenerationResult result;
        result.path = path.get_path();
        as->setSucceeded(result);
    }
    else {
        as->setAborted();
    }

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
    // compute new path if it doesn't exist yet or swarm composition has changed
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
        wp.header.frame_id = "map";
        wp.point = res.point;
        wp_publisher.publish(wp);
    }

    return true;
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

            // regenerate coverage path
            reconfigure = true;
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

            // regenerate coverage path
            reconfigure = true;
        }
        else {
            ++cps;
        }
    }
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
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/resolution", resolution, 1.0);
    nh.param(this_node::getName() + "/visualize", visualize, false);
    nh.param(this_node::getName() + "/divide_area", divide_area, false);
    nh.param(this_node::getName() + "/vertical", vertical, false);
    nh.param(this_node::getName() + "/turning_points", turning_points, false);

    // publishers for introspection
    if (visualize) {
        path_publisher = nh.advertise<nav_msgs::Path>("coverage_path/path", queue_size, true);
        wp_publisher = nh.advertise<geometry_msgs::PointStamped>("coverage_path/waypoint", queue_size, true);
        mst_publisher = nh.advertise<geometry_msgs::PoseArray>("coverage_path/mst", queue_size, true);
    }

    // subscribers for swarm info
    if (divide_area) {
        nh.param(this_node::getName() + "/swarm_timeout", swarm_timeout, 5.0);
        nh.getParam(this_node::getName() + "/states", behaviors);
        swarm_sub = nh.subscribe("swarm_state", queue_size, swarm_state_callback);
    }

    // provide waypoint service
    ServiceServer wp_service = nh.advertiseService("coverage_path/waypoint", get_waypoint);

    // provide coverage path action
    GenerationAction generation_action(nh, "coverage_path/generate", boost::bind(&generate_path_callback, _1, &generation_action), false);
    generation_action.start();

    ROS_INFO("Path generation action server available");

    spin();

    return 0;
}
