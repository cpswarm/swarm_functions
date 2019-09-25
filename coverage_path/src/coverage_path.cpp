#include "coverage_path.h"

void generate_paths ()
{
    // construct minimum spanning tree
    ROS_DEBUG("Construct minimum-spanning-tree...");
    tree.initialize_graph(gridmap);
    tree.construct();

    // generate path
    ROS_DEBUG("Generate coverage path...");
    path.initialize_graph(gridmap);
    path.initialize_tree(tree.get_mst_edges());
    path.generate_path(pose.position);

    reconfigure = false;
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
 * @brief Callback function to receive the grid map.
 * @param msg Merged grid map from all CPSs.
 */
void map_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    gridmap = *msg;
    map_valid = true;
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

    // initialize flags
    pose_valid = false;
    map_valid = false;

    // publishers and subscribers
    Subscriber pose_subscriber = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    Subscriber map_subscriber = nh.subscribe("assigned_map", queue_size, map_callback);
    Publisher path_publisher = nh.advertise<nav_msgs::Path>("path", queue_size, true);

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

    // publish path
    while (ok()) {
        // compute new path if swarm configuration changed
        if (reconfigure) {
            generate_paths();
            path_publisher.publish(path.get_path());
        }

        rate.sleep();
        spinOnce();
    }

    return 0;
}
