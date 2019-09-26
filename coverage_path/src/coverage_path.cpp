#include "coverage_path.h"

/**
 * @brief Generate an optimal coverage path for a given area.
 * @return Whether the path has been generated successfully.
 */
bool generate_path ()
{
    // get area to cover
    nav_msgs::GetMap division;
    if (map_getter.call(division) == false){
        ROS_ERROR("Failed to get the assigned map, cannot compute coverage path!");
        return false;
    }

    // construct minimum spanning tree
    ROS_DEBUG("Construct minimum-spanning-tree...");
    tree.initialize_graph(division.response.map);
    tree.construct();

    // generate path
    ROS_DEBUG("Generate coverage path...");
    path.initialize_graph(division.response.map);
    path.initialize_tree(tree.get_mst_edges());
    path.generate_path(pose.position);

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
        if (generate_path() == false)
            return false;
    }

    // return coverage path
    res.plan = path.get_path();

    return true;
}

/**
 * @brief Callback function to get the coverage path.
 * @param req Path planning request that is ignored.
 * @param res The coverage path.
 * @return Whether the request succeeded.
 */
bool get_waypoint (cpswarm_msgs::GetWaypoint::Request &req, cpswarm_msgs::GetWaypoint::Response &res)
{
    // compute new path if swarm configuration changed
    if (reconfigure) {
        if (generate_path() == false)
            return false;
    }

    // return waypoint
    res.point = path.get_waypoint(pose.position, req.tolerance);

    // visualize waypoint
    if (visualize)
        wp_publisher.publish(res.point);

    return true;
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
    nh.param(this_node::getName() + "/visualize", visualize, false);

    // initialize flags
    pose_valid = false;
    map_valid = false;

    // publishers, subscribers, and service clients
    Subscriber pose_subscriber = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    if (visualize) {
        path_publisher = nh.advertise<nav_msgs::Path>("coverage_path/path", queue_size, true);
        wp_publisher = nh.advertise<nav_msgs::Path>("coverage_path/waypoint", queue_size, true);
    }
    map_getter = nh.serviceClient<nav_msgs::GetMap>("area/assigned");
    map_getter.waitForExistence();

    // init loop rate
    Rate rate(loop_rate);

    // init position
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid position information...");
        rate.sleep();
        spinOnce();
    }

    // init map
    while (ok() && map_valid == false) {
        ROS_DEBUG_ONCE("Waiting for grid map...");
        rate.sleep();
        spinOnce();
    }

    // provide coverage path services
    ServiceServer path_service = nh.advertiseService("coverage_path/path", get_path);
    ServiceServer wp_service = nh.advertiseService("coverage_path/waypoint", get_waypoint);
    spin();

    return 0;
}
