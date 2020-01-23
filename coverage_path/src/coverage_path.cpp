#include "coverage_path.h"

/**
 * @brief Rotate an occupancy grid map so the lower boundary is horizontal.
 * @param map A reference to the occupancy grid map to rotate.
 * @return The angle by which the map has been rotated.
 */
double rotate (nav_msgs::OccupancyGrid& map)
{
    // get angle
    cpswarm_msgs::GetDouble angle;
    if (rotater.call(angle) == false) {
        ROS_INFO("Not rotating map!");
        return 0.0;
    }
    double a = angle.response.value;

    ROS_DEBUG("Rotate map by %.2f...", a);

    // rotate origin
    geometry_msgs::Pose origin_new;
    origin_new.position.x = map.info.origin.position.x*cos(a) - map.info.origin.position.y*sin(a);
    origin_new.position.y = map.info.origin.position.x*sin(a) + map.info.origin.position.y*cos(a);

    // create empty rotated map extra large
    vector<vector<signed char>> rt;
    for (int i=0; i<2*map.info.height; ++i) {
        vector<signed char> row(2*map.info.width, 100);
        rt.push_back(row);
    }

    // rotate map
    int i_new, j_new, width_new=0, height_new=0;
    double x, y, x_new, y_new;
    for (int i=0; i<map.info.height; ++i) {
        for (int j=0; j<map.info.width; ++j) {
            // rotate coordinates
            x = double(j) * map.info.resolution + map.info.origin.position.x;
            y = double(i) * map.info.resolution + map.info.origin.position.y;
            x_new = x*cos(a) - y*sin(a);
            y_new = x*sin(a) + y*cos(a);
            j_new = int(round((x_new - origin_new.position.x) / map.info.resolution));
            i_new = int(round((y_new - origin_new.position.y) / map.info.resolution));

            // skip negative indexes
            if (i_new >= rt.size()) {
                continue;
            }
            if (j_new >= rt[i_new].size()) {
                continue;
            }

            // assign grid cell value
            rt[i_new][j_new] = map.data[i*map.info.width + j];

            // measure maximum required size
            if (rt[i_new][j_new] == 0) {
                if (i_new > height_new)
                    height_new = i_new;
                if (j_new > width_new)
                    width_new = j_new;
            }
        }
    }

    // truncate rotated map
    rt.resize(height_new);
    for (int i=0; i<rt.size(); ++i)
        rt[i].resize(width_new);

    // collapse map to one dimensional vector
    vector<signed char> rt_col;
    for (int i=0; i<rt.size(); ++i) {
        for (int j=0; j<rt[i].size(); ++j) {
            rt_col.push_back(rt[i][j]);
        }
    }

    // assign map data
    map.data = rt_col;

    // update meta data
    map.info.map_load_time = Time::now();
    map.info.width = width_new;
    map.info.height= height_new;
    map.info.origin = origin_new;

    return a;
}

/**
 * @brief Decrease the resolution of a occupancy grid map.
 * @param map A reference to the occupancy grid map to downsample.
 */
void downsample (nav_msgs::OccupancyGrid& map)
{
    // do not increase resolution
    if (map.info.resolution >= resolution)
        return;

    // reduction factor
    int f = int(round(resolution / map.info.resolution));

    ROS_DEBUG("Downsample map by %d...", f);

    // downsample map data
    vector<signed char> lr;
    for (int i=0; i+f<map.info.height; i+=f) {
        for (int j=0; j+f<map.info.width; j+=f) {
            // count frequency of map data values
            vector<unsigned int> values(256, 0);
            for (int m=i; m<i+f; ++m) {
                for (int n=j; n<j+f; ++n) {
                    values[map.data[m*map.info.width + n]]++;
                }
            }

            // choose value with highest frequency
            unsigned char value = 0;
            unsigned int freq = 0;
            for (int k=0; k<values.size(); ++k) {
                if (values[k] > freq) {
                    value = k;
                    freq = values[k];
                }
            }

            // push back most seen value
            lr.push_back(value);
        }
    }
    map.data = lr;

    // update meta data
    map.info.map_load_time = Time::now();
    map.info.resolution = resolution;
    map.info.width = int(floor(double(map.info.width) / double(f)));
    map.info.height = int(floor(double(map.info.height) / double(f)));
}

/**
 * @brief Generate an optimal coverage path for a given area.
 * @param start The starting position of the path.
 * @return Whether the path has been generated successfully.
 */
bool generate_path (geometry_msgs::Point start)
{
    // get area to cover
    nav_msgs::GetMap gm;
    if (map_getter.call(gm) == false){
        ROS_ERROR("Failed to get the assigned map, cannot compute coverage path!");
        return false;
    }
    nav_msgs::OccupancyGrid map = gm.response.map;

    // divided area is already rotated, translated, and downsampled
    double rotation = 0.0;
    geometry_msgs::Vector3 translation;
    if (divide_area) {
        // get angle of rotation
        cpswarm_msgs::GetDouble get_angle;
        if (rotater.call(get_angle))
            rotation = get_angle.response.value;

        // get translation vector
        cpswarm_msgs::GetVector get_translation;
        if (translater.call(get_translation))
            translation = get_translation.response.vector;
    }

    // original map still needs rotation and downsampling
    else {
        // rotate map
        rotation = rotate(map);

        // downsample resolution
        if (map.info.resolution < resolution) {
            downsample(map);
        }
    }

    ROS_INFO("Generate new coverage path...");

    // construct minimum spanning tree
    ROS_DEBUG("Construct minimum-spanning-tree...");
    tree.initialize_graph(map, translation, rotation, vertical);
    tree.construct();

    // visualize path
    if (visualize)
        mst_publisher.publish(tree.get_tree());

    // transform starting point
    geometry_msgs::Point cps;
    cps.x = (start.x + translation.x) * cos(rotation) - (start.y + translation.y) * sin(rotation);
    cps.y = (start.x + translation.x) * sin(rotation) + (start.y + translation.y) * cos(rotation);

    // generate path
    ROS_DEBUG("Generate coverage path...");
    path.initialize_graph(map, translation, rotation);
    path.initialize_tree(tree.get_mst_edges());
    path.generate_path(cps);

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
    nh.param(this_node::getName() + "/resolution", resolution, 1.0);
    nh.param(this_node::getName() + "/swarm_timeout", swarm_timeout, 5.0);
    nh.param(this_node::getName() + "/visualize", visualize, false);
    nh.param(this_node::getName() + "/divide_area", divide_area, false);
    nh.param(this_node::getName() + "/vertical", vertical, false);

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
    if (divide_area) {
        map_getter = nh.serviceClient<nav_msgs::GetMap>("area/assigned");
        translater = nh.serviceClient<cpswarm_msgs::GetVector>("area/get_translation");
        translater.waitForExistence();
    }
    else
        map_getter = nh.serviceClient<nav_msgs::GetMap>("area/get_map");
    map_getter.waitForExistence();
    rotater = nh.serviceClient<cpswarm_msgs::GetDouble>("area/get_rotation");
    rotater.waitForExistence();

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
