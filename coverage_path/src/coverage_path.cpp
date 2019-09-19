#include "coverage_path.h"

void generate_paths ()
{
    // convert swarm pose to grid
    map<string, vector<int>> swarm_grid;
    for (auto cps : swarm_pose) {
        vector<int> pos(2);
        pos[0] = int(round(cps.second.pose.position.x / gridmap.info.resolution));
        pos[1] = int(round(cps.second.pose.position.y / gridmap.info.resolution));
        swarm_grid.emplace(cps.first, pos);
    }

    // divide area
    division.initialize_cps(swarm_grid);
    vector<signed char, allocator<signed char>> map = gridmap.data;
    division.initialize_map((int)gridmap.info.width, (int)gridmap.info.height, map);
    division.divide();
    valarray<bool> assignment = division.get_region(uuid);

    // construct minimum spanning tree
    tree.initialize_graph((int)gridmap.info.width, (int)gridmap.info.height, assignment);
    tree.construct();
    vector<edge> mst = tree.get_mst_edges();

    // generate path
    path.initialize_map(gridmap);
    path.initialize_tree(mst);
    path.initialize_graph(assignment);
    path.remove_edges();
    path.generate_path(pose.position);
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
    // valid pose received
    if (msg->header.stamp.isValid())
        pose_valid = true;

    // store new position and orientation in class variables
    pose = msg->pose;
}

/**
 * @brief Callback function to receive the positions of the other CPSs.
 * @param msg UUIDs and positions of the other CPSs.
 */
void swarm_callback (const cpswarm_msgs::ArrayOfPositions::ConstPtr& msg)
{
    // update cps positions
    for (auto cps : msg->positions) {
        // index of cps in map
        auto idx = swarm_pose.find(cps.swarmio.node);

        // add new cps
        if (idx == swarm_pose.end()) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = Time::now();
            pose.pose = cps.pose;
            swarm_pose.emplace(cps.swarmio.node, pose);

            // regenerate paths
            reconfigure = true;
        }

        // update existing cps
        else {
            idx->second.header.stamp = Time::now();
            idx->second.pose = cps.pose;
        }
    }

    // remove old cps
    for (auto cps=swarm_pose.cbegin(); cps!=swarm_pose.cend();) {
        if (cps->second.header.stamp + Duration(swarm_timeout) < Time::now()) {
            swarm_pose.erase(cps++);

            // regenerate paths
            reconfigure = true;
        }
        else {
            ++cps;
        }
    }
}

/**
 * @brief Callback function to receive the grid map.
 * @param msg Merged grid map from all CPSs.
 */
void map_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_valid = true;
//     gridmap = *msg;
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
    nh.param(this_node::getName() + "/position_tolerance", position_tolerance, 0.5);
    nh.param(this_node::getName() + "/swarm_timeout", swarm_timeout, 5.0);

    // publishers and subscribers
    Subscriber uuid_sub = nh.subscribe("bridge/uuid", queue_size, uuid_callback);
    Subscriber pose_subscriber = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    Subscriber swarm_subscriber = nh.subscribe("swarm_position", queue_size, swarm_callback);
    Subscriber map_subscriber = nh.subscribe("map", queue_size, map_callback); // TODO: use merged map
    Publisher waypoint_publisher = nh.advertise<geometry_msgs::Point>("waypoint", queue_size, true);
    path_publisher = nh.advertise<nav_msgs::Path>("path", queue_size, true);

    // init loop rate
    Rate rate(loop_rate);

    // init uuid
    uuid = "";
    while (ok() && uuid == "") {
        rate.sleep();
        spinOnce();
    }

    // init position
    pose_valid = false;
    swarm_valid = true; // TODO
    while (ok() && (pose_valid == false || swarm_valid == false)) {
        ROS_DEBUG_ONCE("Waiting for valid position information...");
        rate.sleep();
        spinOnce();
    }

    // init map
    map_valid = false;
//     while (ok() && map_valid == false) {
//         ROS_ERROR_THROTTLE(1, "Waiting for grid map...");
//         rate.sleep();
//         spinOnce();
//     }

    // configure area division optimizer
    division.setup(80000, 0.01, 1e-4, 30, false); // TODO: define parameters

    // publish path waypoints
    while (ok()) {
        // compute new path if swarm configuration changed
        if (reconfigure) {
            generate_paths();
        }

        // get current waypoint
        geometry_msgs::Point waypoint;// = path.current_wp();

        // reached waypoint
        if (hypot(waypoint.x - pose.position.x, waypoint.y - pose.position.y) < position_tolerance) {
            // get next waypoint
            //waypoint = path.next_wp();

            // publish waypoint
            waypoint_publisher.publish(waypoint);
        }

        rate.sleep();
        spinOnce();
    }

    return 0;
}
