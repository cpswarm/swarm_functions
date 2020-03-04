#include "area_division.h"

/**
 * @brief Switch to the synchronization state.
 */
void to_sync ()
{
    state = SYNC;
    ROS_DEBUG("Start synchronizing...");

    // reset information about swarm
    swarm_pose.clear();

    // stop moving
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = Time::now();
    goal_pose.pose = pose;
    pos_pub.publish(goal_pose);

    // start of synchronization time window
    sync_start = Time::now();
}

/**
 * @brief Callback function for behavior state updates.
 * @param msg State received from the CPS.
 */
void behavior_state_callback (const cpswarm_msgs::StateEvent::ConstPtr& msg)
{
    // store new state in class variables
    behavior = msg->state;

    // valid state received
    if (msg->header.stamp.isValid())
        behavior_valid = true;

    // cps switched to a behavior state that requires area division
    if (state == IDLE && find(behaviors.begin(), behaviors.end(), behavior) != behaviors.end())
        state = INIT;

    // cps switched to a behavior state that does not require area division
    if (state == ACTIVE && find(behaviors.begin(), behaviors.end(), behavior) == behaviors.end())
        state = DEINIT;
}

/**
 * @brief Callback function to receive area division requests from other CPSs.
 * @param msg UUIDs and position of the other CPS.
 */
void division_callback (const cpswarm_msgs::AreaDivisionEvent::ConstPtr& msg)
{
    // only divide area if active
    if (state == ACTIVE)
        to_sync();

    // only synchronize if ready
    if (state != SYNC)
        return;

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
 * @param msg Global grid map.
 */
void map_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // store map in class variable
    global_map = *msg;

    // valid map received
    map_valid = true;

    // divide area
    if (state == ACTIVE)
        to_sync();
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

            // divide area
            if (state == ACTIVE)
                to_sync();
        }
        else {
            ++cps;
        }
    }

    swarm_valid = true;
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
 * @brief Initialize this node.
 */
void init ()
{
    NodeHandle nh;

    // initialize flags
    uuid = "";
    pose_valid = false;
    swarm_valid = false;
    map_valid = false;

    // publishers, subscribers, and service clients
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    uuid_sub = nh.subscribe("bridge/uuid", queue_size, uuid_callback);
    pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    swarm_sub = nh.subscribe("swarm_state", queue_size, swarm_state_callback);
    map_sub = nh.subscribe("area/map", queue_size, map_callback); // TODO: use explored/merged map
    division_sub = nh.subscribe("bridge/events/area_division", queue_size, division_callback);
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", queue_size, true);
    swarm_pub = nh.advertise<cpswarm_msgs::AreaDivisionEvent>("area_division", queue_size, true);
    area_pub = nh.advertise<nav_msgs::OccupancyGrid>("area/assigned", queue_size, true);
    if (visualize) {
        map_rot_pub = nh.advertise<nav_msgs::OccupancyGrid>("area/rotated", queue_size, true);
        map_ds_pub = nh.advertise<nav_msgs::OccupancyGrid>("area/downsampled", queue_size, true);
    }
    rotater_cli = nh.serviceClient<cpswarm_msgs::GetDouble>("area/get_rotation");
    rotater_cli.waitForExistence();

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

    // init swarm
    while (ok() && swarm_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid swarm information...");
        rate->sleep();
        spinOnce();
    }

    // init map
    while (ok() && map_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid grid map...");
        rate->sleep();
        spinOnce();
    }

    // create area division object
    division = new area_division();

    // start area division
    to_sync();
}

/**
 * @brief Shutdown ROS communication.
 */
void deinit ()
{
    // shutdown ros communication
    uuid_sub.shutdown();
    pose_sub.shutdown();
    swarm_sub.shutdown();
    map_sub.shutdown();
    division_sub.shutdown();
    pos_pub.shutdown();
    swarm_pub.shutdown();
    area_pub.shutdown();
    map_rot_pub.shutdown();
    map_ds_pub.shutdown();
    rotater_cli.shutdown();

    // destroy optimizer
    delete division;

    state = IDLE;
}

/**
 * @brief Synchronize The CPSs by exchanging an event.
 */
void sync ()
{
    // publish synchronization event for some time
    if (sync_start + Duration(swarm_timeout) > Time::now()) {
        cpswarm_msgs::AreaDivisionEvent event;
        event.header.stamp = Time::now();
        event.swarmio.name = "area_division";
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "local_origin_ned";
        ps.pose = pose;
        event.pose = ps;
        swarm_pub.publish(event);
    }

    // divide area
    else
        state = DIVIDE;
}

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
    if (rotater_cli.call(angle) == false) {
        ROS_DEBUG("Not rotating map!");
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
 * @brief Shift a map to be aligned with the grid, i.e., the origin should be an even number.
 * @param map The map to shift.
 */
void translate (nav_msgs::OccupancyGrid& map)
{
    // compute required translation
    translation.x = round(map.info.origin.position.x) - map.info.origin.position.x;
    translation.y = round(map.info.origin.position.y) - map.info.origin.position.y;
    ROS_DEBUG("Translate map by (%.2f,%.2f)...", translation.x, translation.y);

    // translate origin
    map.info.origin.position.x += translation.x;
    map.info.origin.position.y += translation.y;

    // update meta data
    map.info.map_load_time = Time::now();
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
 * @brief Divide the area of the grid map equally among multiple CPSs.
 */
void divide_area ()
{
    // initialize map
    nav_msgs::OccupancyGrid gridmap = global_map;

    // rotate map
    double angle = rotate(gridmap);

    if (visualize)
        map_rot_pub.publish(gridmap);

    // downsample resolution
    if (gridmap.info.resolution < resolution) {
        downsample(gridmap);
    }

    // shift map
    translate(gridmap);

    if (visualize)
        map_ds_pub.publish(gridmap);

    // convert swarm pose to grid
    map<string, vector<int>> swarm_grid;
    for (auto cps : swarm_pose) {
        geometry_msgs::Point rotated = rotate(cps.second.pose.position, angle);
        rotated.x += translation.x;
        rotated.y += translation.y;
        vector<int> pos(2);
        pos[0] = int(round((rotated.x - gridmap.info.origin.position.x) / gridmap.info.resolution));
        pos[1] = int(round((rotated.y - gridmap.info.origin.position.y) / gridmap.info.resolution));
        swarm_grid.emplace(cps.first, pos);
        ROS_DEBUG("Other CPS %s at (%d,%d)", cps.first.c_str(), pos[0], pos[1]);
    }

    // add this robot to swarm grid
    geometry_msgs::Point rotated = rotate(pose.position, angle);
    rotated.x += translation.x;
    rotated.y += translation.y;
    vector<int> pos(2);
    pos[0] = int(round((rotated.x - gridmap.info.origin.position.x) / gridmap.info.resolution));
    pos[1] = int(round((rotated.y - gridmap.info.origin.position.y) / gridmap.info.resolution));
    swarm_grid.emplace(uuid, pos);
    ROS_DEBUG("Me %s at (%d,%d)", uuid.c_str(), pos[0], pos[1]);

    // divide area
    ROS_INFO("Dividing area...");
    vector<signed char, allocator<signed char>> map = gridmap.data;
    division->initialize_map((int)gridmap.info.height, (int)gridmap.info.width, map);
    division->initialize_cps(swarm_grid);
    division->divide();

    // publish result
    area_pub.publish(division->get_grid(gridmap, uuid));

    // wait a bit to avoid directly redividing again
    Time sleep_start = Time::now();
    while (ok() && sleep_start + Duration(swarm_timeout) > Time::now()) {
        spinOnce();
        rate->sleep();
    }

    // area division done
    state = ACTIVE;
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

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.5);
    rate = new Rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    nh.param(this_node::getName() + "/resolution", resolution, 1.0);
    nh.param(this_node::getName() + "/swarm_timeout", swarm_timeout, 5.0);
    nh.param(this_node::getName() + "/visualize", visualize, false);
    nh.getParam(this_node::getName() + "/states", behaviors);

    // initially, this cps does not perform area division
    state = IDLE;

    // init behavior state
    behavior_valid = false;
    Subscriber behavior_state_subscriber = nh.subscribe("state", queue_size, behavior_state_callback);
    while (ok() && behavior_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid behavior information...");
        rate->sleep();
        spinOnce();
    }

    // check if area division is necessary
    while (ok()) {
        spinOnce();

        switch (state) {
            // start up this node functionality
            case INIT:
                init();
                break;

            // synchronize with the other cpss
            case SYNC:
                sync();
                break;

            // divide area
            case DIVIDE:
                divide_area();
                break;

            // shutdown this node functionality
            case DEINIT:
                deinit();
                break;
        }

        rate->sleep();
    }

    // clean up
    delete rate;

    return 0;
}
