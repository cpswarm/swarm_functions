#include "area_division.h"

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

    ROS_ERROR("Rotate map by %.2f...", a);

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

    ROS_ERROR("Downsample map by %d...", f);

    // downsample map data
    vector<signed char> lr;
    for (int i=0; i+f<=map.info.height; i+=f) {
        for (int j=0; j+f<=map.info.width; j+=f) {
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
 * @brief Divide the area of the grid map equally among multiple CPSs.
 */
void divide_area ()
{
    // rotate map
    double angle = rotate(gridmap);

    map_rot_publisher.publish(gridmap);

    // downsample resolution
    if (gridmap.info.resolution < resolution) {
        downsample(gridmap);
    }

    map_ds_publisher.publish(gridmap);

    // convert swarm pose to grid
    map<string, vector<int>> swarm_grid;
    for (auto cps : swarm_pose) {
        geometry_msgs::Point rotated = rotate(cps.second.pose.position, angle);
        vector<int> pos(2);
        pos[0] = int(round((rotated.x - gridmap.info.origin.position.x) / gridmap.info.resolution));
        pos[1] = int(round((rotated.y - gridmap.info.origin.position.y) / gridmap.info.resolution));
        swarm_grid.emplace(cps.first, pos);
        ROS_DEBUG("Other CPS %s at (%d,%d)", cps.first.c_str(), pos[0], pos[1]);
    }

    // add this robot to swarm grid
    geometry_msgs::Point rotated = rotate(pose.position, angle);
    vector<int> pos(2);
    pos[0] = int(round((rotated.x - gridmap.info.origin.position.x) / gridmap.info.resolution));
    pos[1] = int(round((rotated.y - gridmap.info.origin.position.y) / gridmap.info.resolution));
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
    nh.param(this_node::getName() + "/resolution", resolution, 1.0);
    nh.param(this_node::getName() + "/swarm_timeout", swarm_timeout, 5.0);
    nh.param(this_node::getName() + "/visualize", visualize, false);

    // initialize flags
    uuid = "";
    pose_valid = false;
    swarm_valid = false;
    map_valid = false;

    // publishers, subscribers, and service clients
    Subscriber uuid_sub = nh.subscribe("bridge/uuid", queue_size, uuid_callback);
    Subscriber pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    Subscriber swarm_sub = nh.subscribe("bridge/events/area_division", queue_size, swarm_callback);
    Subscriber map_sub = nh.subscribe("area_provider/map", queue_size, map_callback); // TODO: use explored/merged map
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", queue_size, true);
    swarm_pub = nh.advertise<cpswarm_msgs::AreaDivisionEvent>("area_division", queue_size, true);
    if (visualize) {
        area_pub = nh.advertise<nav_msgs::OccupancyGrid>("assigned_map", queue_size, true);
        map_rot_publisher = nh.advertise<nav_msgs::OccupancyGrid>("area_division/rotated_map", queue_size, true);
        map_ds_publisher = nh.advertise<nav_msgs::OccupancyGrid>("area_division/downsampled_map", queue_size, true);
    }
    rotater = nh.serviceClient<cpswarm_msgs::GetDouble>("area/get_rotation");
    rotater.waitForExistence();

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
