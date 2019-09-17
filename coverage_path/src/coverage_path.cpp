#include "coverage_path.h"

void generate_paths ()
{
    //division.divide(); // DARP.constructAssignmentM
    //tree.construct(); // KruskalMSTS.calculateMSTs(DARP.getBinrayRobotRegions(), nr);
    //path.generate();
    /*
    A r*rayList<Integer[]> InitRobots = p.getRobotsInit();
    ArrayList<ArrayList<Integer[]>> AllRealPaths = new ArrayList<>();
    for (int r=0;r<nr;r++) {
        CalculateTrajectories ct = new CalculateTrajectories(rows,cols, KruskalMSTS.get(r)); //Send MSTs
        ct.initializeGraph(CalcRealBinaryReg(p.getBinrayRobotRegions().get(r)), true); //Send [x2 x2] Binary Robot Region
        ct.RemoveTheAppropriateEdges();
        ct.CalculatePathsSequence(4*InitRobots.get(r)[0]*cols+2*InitRobots.get(r)[1]);
        AllRealPaths.add(ct.getPathSequence());
    }
    */
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
    // check if swarm members joined or left the swarm
    // TODO
//     reconfigure = true;

    // save positions
    swarm_pose = msg->positions;
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

    // publishers and subscribers
    Subscriber pose_subscriber = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    Subscriber swarm_subscriber = nh.subscribe("swarm_position", queue_size, swarm_callback);
    Subscriber map_subscriber = nh.subscribe("map", queue_size, map_callback); // TODO: use merged map
    Publisher waypoint_publisher = nh.advertise<geometry_msgs::Point>("waypoint", queue_size, true);
    path_publisher = nh.advertise<nav_msgs::Path>("path", queue_size, true);

    // init loop rate
    Rate rate(loop_rate);

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
