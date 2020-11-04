#include "coverage_path.h"

/**
 * @brief Determine properties of an area.
 * @param coords The coordinates defining the area polygon.
 * @param origin Returns the bottom left coordinate of the area.
 * @param rotation Returns the angle which the area has to be rotated so the lower boundary is horizontal.
 * @param width Returns the width of the area after rotation.
 * @param height Returns the height of the area after rotation.
 */
void analyze_area (vector<geometry_msgs::Point> coords, geometry_msgs::Point& origin, double& rotation, double& width, double& height)
{
    // get coordinates
    geometry_msgs::Point pl, pb, pr, pt;
    pl.x = numeric_limits<double>::max();
    pb.y = numeric_limits<double>::max();
    pr.x = numeric_limits<double>::min();
    pt.y = numeric_limits<double>::min();
    for (auto p : coords) {
        // left most point
        if (p.x < pl.x || (p.x == pl.x && p.y < pl.y))
            pl = p;

        // bottom most point
        if (p.y < pb.y)
            pb = p;

        // right most point
        if (p.x > pr.x || (p.x == pr.x && p.y < pr.y))
            pr = p;

        // top most point
        if (p.y > pt.y)
            pt = p;
    }

    // no rotation required
    if ((pl.x == pb.x && pl.y == pb.y) || (pr.x == pb.x && pr.y == pb.y)) {
        origin = pl;
        rotation = 0;
        width = pr.x - pl.x;
        height = pt.y - pb.y;
    }

    // rotate clockwise
    else if (pr.y < pl.y) {
        origin = pb;
        rotation = -atan2(pr.y - pb.y, pr.x - pb.x);
        width = hypot(pr.x - pb.x, pr.y - pb.y);
        height = hypot(pl.x - pb.x, pl.y - pb.y);
    }

    // rotate counter clockwise
    else {
        origin = pl;
        rotation = -atan2(pb.y - pl.y, pb.x - pl.x);
        width = hypot(pl.x - pb.x, pl.y - pb.y);
        height = hypot(pr.x - pb.x, pr.y - pb.y);
    }

    ROS_DEBUG("Origin (%.2f,%.2f)", origin.x, origin.y);
    ROS_DEBUG("Rotation %.2f", rotation);
    ROS_DEBUG("Size %.2fx%.2f", width, height);
}

/**
 * @brief Rotate an occupancy grid map so the lower boundary is horizontal.
 * @param map A reference to the occupancy grid map to rotate.
 * @param angle The angle to rotate the map by.
 */
void rotate (nav_msgs::OccupancyGrid& map, double angle)
{
    ROS_DEBUG("Rotate map by %.2f...", angle);

    // rotate origin
    geometry_msgs::Pose origin_new;
    origin_new.position.x = map.info.origin.position.x*cos(angle) - map.info.origin.position.y*sin(angle);
    origin_new.position.y = map.info.origin.position.x*sin(angle) + map.info.origin.position.y*cos(angle);

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
            x_new = x*cos(angle) - y*sin(angle);
            y_new = x*sin(angle) + y*cos(angle);
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

    // remove rows with obstacles only
    for (int i=0; i<map.info.height; ++i) {
        // count number of occupied cells in a row
        int obst = 0;
        for (int j=0; j<map.info.width; ++j) {
            if (map.data[i*map.info.width + j] == 100) {
                ++obst;
            }
        }

        // remove row
        if (obst == map.info.width) {
            // delete grid cells
            map.data.erase(map.data.begin() + i*map.info.width, map.data.begin() + (i+1)*map.info.width);

            // update meta data
            map.info.map_load_time = Time::now();
            --map.info.height;
            map.info.origin.position.y += map.info.resolution;

            // stay in current row
            --i;
        }
    }
}

/**
 * @brief Generate an optimal coverage path for a given area.
 * @param start The starting position of the path.
 * @return Whether the path has been generated successfully.
 */
bool generate_path (geometry_msgs::Point start)
{
    ROS_DEBUG("Starting at (%.2f,%.2f)", start.x, start.y);

    // get coordinates of area to cover
    vector<geometry_msgs::Point> coords;
    cpswarm_msgs::GetPoints get_coords;
    if (area_getter.call(get_coords))
        coords = get_coords.response.points;
    else {
        ROS_ERROR("Failed to get area coordinates!");
        return false;
    }

    // get properties of area to cover
    geometry_msgs::Point origin;
    double rotation, width, height;
    analyze_area(coords, origin, rotation, width, height);

    // original map still needs rotation and downsampling
    if (divide_area == false) {
        // rotate map
        rotate(area, rotation);

        // downsample resolution
        if (area.info.resolution < resolution) {
            downsample(area);
        }
    }

    ROS_INFO("Generate new coverage path...");

    // construct minimum spanning tree
    ROS_DEBUG("Construct minimum-spanning-tree...");
    tree.initialize_graph(area, vertical);
    tree.construct();

    // visualize path
    if (visualize)
        mst_publisher.publish(tree.get_tree());

    // generate path
    ROS_DEBUG("Generate coverage path...");
    path.initialize_graph(area);
    path.initialize_map(origin, rotation, width, height);
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
 * @brief Callback function to get the coverage path.
 * @param req Path planning request that is ignored.
 * @param res The coverage path.
 * @return Whether the request succeeded.
 */
bool get_path (nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
{
    // compute new path if area map has changed
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
    // compute new path if area map has changed
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
 * @brief Callback function to receive the grid map.
 * @param msg Grid map to be covered by this CPSs.
 */
void area_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    area = *msg;
    map_valid = true;
    reconfigure = true;
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
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/resolution", resolution, 1.0);
    nh.param(this_node::getName() + "/visualize", visualize, false);
    nh.param(this_node::getName() + "/divide_area", divide_area, false);
    nh.param(this_node::getName() + "/vertical", vertical, false);
    nh.param(this_node::getName() + "/turning_points", turning_points, false);
    nh.param(this_node::getName() + "/swarm_path", swarm_path, false);

    // publishers, subscribers, and service clients
    if (visualize) {
        path_publisher = nh.advertise<nav_msgs::Path>("coverage_path/path", queue_size, true);
        wp_publisher = nh.advertise<geometry_msgs::PointStamped>("coverage_path/waypoint", queue_size, true);
        mst_publisher = nh.advertise<geometry_msgs::PoseArray>("coverage_path/mst", queue_size, true);
    }
    if (divide_area)
        map_subscriber = nh.subscribe("area/assigned", queue_size, area_callback);
    else
        map_subscriber = nh.subscribe("area/map", queue_size, area_callback);
    if (swarm_path) {
        nh.param(this_node::getName() + "/swarm_timeout", swarm_timeout, 5.0);
        nh.getParam(this_node::getName() + "/states", behaviors);
        swarm_sub = nh.subscribe("swarm_state", queue_size, swarm_state_callback);
    }
    area_getter = nh.serviceClient<cpswarm_msgs::GetPoints>("area/get_area");
    area_getter.waitForExistence();

    // init loop rate
    Rate rate(loop_rate);

    // initialize flags
    map_valid = false;

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
