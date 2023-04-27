#include "lib/mst_path.h"

mst_path::mst_path ()
{
}

bool mst_path::generate_path (geometry_msgs::Point start)
{
    // reset counter of current waypoint
    this->wp = 0;

    // starting vertex
    // rotate and translate
    geometry_msgs::Point start_rt;
    start_rt.x = (start.x * cos(rotation) - start.y * sin(rotation) - map.info.origin.position.x) / map.info.resolution;
    start_rt.y = (start.x * sin(rotation) + start.y * cos(rotation) - map.info.origin.position.y) / map.info.resolution;
    // starting bound on map boundary, shift a bit inside to allow
    if (start_rt.x == map.info.width)
        start_rt.x -= 0.1;
    if (start_rt.y == map.info.height)
        start_rt.y -= 0.1;
    // starting point outside map
    if (start_rt.x < 0 || map.info.width < start_rt.x || start_rt.y < 0 || map.info.height < start_rt.y) {
        ROS_WARN("Start point (%.2f,%.2f) out of map!", start.x, start.y);
    }
    // convert start point to cell index
    int start_idx = round(2 * round2idx(start_rt.y) * 2*map.info.width + 2 * round2idx(start_rt.x));
    // make sure cell is not occupied
    unordered_set<int> checked;
    queue<int> to_check;
    int current;
    // check current cell
    checked.insert(start_idx);
    to_check.push(start_idx);
    // breadth-first search for empty cell
    while (to_check.empty() == false) {
        // check next cell
        current = to_check.front();
        to_check.pop();
        if (nodes[current].size() > 0)
            break;
        // add neighboring cells to queue
        if (current%(2*map.info.width) > 1 && checked.count(current-1) <= 0) { // left
            to_check.push(current-1);
            checked.insert(current-1);
        }
        if (current%(2*map.info.width) < 2*map.info.width-1 && checked.count(current+1) <= 0) { // right
            to_check.push(current+1);
            checked.insert(current+1);
        }
        if (current/(2*map.info.width) > 0 && checked.count(current-2*map.info.width) <= 0) { // down
            to_check.push(current-2*map.info.width);
            checked.insert(current-2*map.info.width);
        }
        if (current/(2*map.info.width) < 2*map.info.height-1 && checked.count(current+2*map.info.width) <= 0) { // up
            to_check.push(current+2*map.info.width);
            checked.insert(current+2*map.info.width);
        }
    }
    // convert vertex to point
    path.push_back(idx2wp(current));

    if (current != start_idx)
        ROS_WARN("Changed starting point of coverage path from (%.2f,%.2f) to (%.2f,%.2f)", idx2wp(start_idx).x, idx2wp(start_idx).y, path.back().x, path.back().y);
    ROS_DEBUG("First waypoint %d (%.2f,%.2f)", current, path.back().x, path.back().y);

    // visited vertices
    unordered_set<int> removed;

    // previous vertex
    int previous;

    // possible movements
    vector<int> movement;                      // priority:
    if (vertical) {
        movement.push_back(-2*map.info.width); // down
        movement.push_back(-1);                // left
        movement.push_back(2*map.info.width);  // up
        movement.push_back(1);                 // right
    }
    else {
        movement.push_back(-1);                // left
        movement.push_back(-2*map.info.width); // down
        movement.push_back(1);                 // right
        movement.push_back(2*map.info.width);  // up
    }

    // valid movement
    bool found = false;

    // look for valid first step
    for (int idx=0; idx<movement.size(); idx++) {
        if (nodes[current].count(current + movement[idx]) > 0) {
            previous = current + movement[idx];
            found = true;
            break;
        }
    }

    // no valid first step found
    if (!found) {
        ROS_ERROR("Could not generate coverage path, invalid starting point!");
        return false;
    }

    // generate path
    do {
        // remember visited vertices
        removed.insert(current);

        // look for valid step
        found = false;
        previous = current;
        for (int idx=0; idx<movement.size(); idx++){
            if (nodes[previous].count(previous + movement[idx]) > 0 &&
                removed.count(previous + movement[idx]) <= 0) {
                current = previous + movement[idx];
                found = true;
                break;
            }
        }
        if (!found) {
            break;
        }

        // remove vertices from sets
        nodes[current].erase(previous);
        nodes[previous].erase(current);

        // add vertex to path
        path.push_back(idx2wp(current));
    } while (found);

    // final waypoint
    path.push_back(path.front());

    return true;
}

nav_msgs::Path mst_path::get_path ()
{
    nav_msgs::Path nav_path;
    vector<geometry_msgs::PoseStamped> poses;
    geometry_msgs::PoseStamped pose;
    for (auto p : path) {
        // relative to original map
        p.x += map.info.origin.position.x;
        p.y += map.info.origin.position.y;

        // rotate
        pose.pose.position.x = p.x * cos(-rotation) - p.y * sin(-rotation);
        pose.pose.position.y = p.x * sin(-rotation) + p.y * cos(-rotation);

        poses.push_back(pose);
    }
    nav_path.poses = poses;
    nav_path.header.stamp = Time::now();
    nav_path.header.frame_id = "map";
    return nav_path;
}

geometry_msgs::Point mst_path::get_waypoint (geometry_msgs::Point position, double tolerance)
{
    ROS_DEBUG("Check if distance between (%.2f,%.2f) and (%.2f,%.2f) < %.2f", position.x, position.y, get_wp().x, get_wp().y, tolerance);

    // close enough to or past current waypoint
    if (dist(position, get_wp()) < tolerance || dist(position, get_wp(1)) < dist(position, get_wp())) {
        // select next waypoint
        ++wp;
        ROS_DEBUG("Select waypoint %d", wp);
    }

    ROS_DEBUG("Return waypoint %d at (%.2f,%.2f)", wp, get_wp().x, get_wp().y);

    return get_wp();
}

void mst_path::initialize_map (nav_msgs::OccupancyGrid gridmap, double rotation, bool vertical, bool connect4)
{
    ROS_DEBUG("Gridmap size %dx%d origin (%.2f,%.2f)", gridmap.info.width, gridmap.info.height, gridmap.info.origin.position.x, gridmap.info.origin.position.y);

    // rotation of map
    this->rotation = rotation;

    // orientation of path edges
    this->vertical = vertical;

    // initialize path
    map = gridmap;
    int rows = gridmap.info.height;
    int cols = gridmap.info.width;
    nodes.clear();
    nodes.resize(2*rows*2*cols);
    edges.clear();
    path.clear();

    // double graph resolution
    valarray<bool> inflated(2*rows*2*cols);
    for (int i=0; i<2*rows; i++) {
        for (int j=0; j<2*cols; j++) {
            inflated[i*2*cols + j] = (gridmap.data[(i/2)*cols + j/2] == 0);
        }
    }

    // iterate all rows and columns
    for (int i=0; i<2*rows; i++) {
        for (int j=0; j<2*cols; j++) {
            // found a vertex
            if (inflated[2*i*cols+j]) {
                // check von neumann neighborhood for connected vertices
                if (i>0 && inflated[(i-1)*2*cols+j]) {
                    add_edge(i*2*cols+j, (i-1)*2*cols+j, 1);
                }
                if (i<2*rows-1 && inflated[(i+1)*2*cols+j]) {
                    add_edge(i*2*cols+j, (i+1)*2*cols+j, 1);
                }
                if (j>0 && inflated[i*2*cols+j-1]) {
                    add_edge(i*2*cols+j, i*2*cols+j-1, 1);
                }
                if (j<2*cols-1 && inflated[i*2*cols+j+1]) {
                    add_edge(i*2*cols+j, i*2*cols+j+1, 1);
                }

                // check moore neighborhood for connected vertices
                if (!connect4) {
                    if (i>0 && j>0 && inflated[(i-1)*2*cols+j-1]) {
                        add_edge(i*2*cols+j, (i-1)*2*cols+j-1, sqrt(2));
                    }
                    if (i<2*rows-1 && j<2*cols-1 && inflated[(i+1)*2*cols+j+1]) {
                        add_edge(i*2*cols+j, (i+1)*2*cols+j+1, sqrt(2));
                    }
                    if (i<2*rows-1 && j>0 && inflated[(i+1)*2*cols+j-1]) {
                        add_edge(i*2*cols+j, (i+1)*2*cols+j-1, sqrt(2));
                    }
                    if (i>0 && j<2*cols-1 && inflated[(i-1)*2*cols+j+1]) {
                        add_edge(i*2*cols+j, (i-1)*2*cols+j+1, sqrt(2));
                    }
                }
            }
        }
    }
}

void mst_path::initialize_tree (set<edge> mst)
{
    // remove edges not required by mst
    int cols = map.info.width;
    int alpha;
    for (auto e : mst) {
        if (e.vhigh - e.vlow == 1) {
            alpha = (4*e.vlow + 3) - 2 * (e.vhigh % cols);
            remove_edge(edge(alpha, alpha + 2*cols, 1));
            remove_edge(edge(alpha + 2*cols, alpha, 1));
            remove_edge(edge(alpha+1, alpha+1 + 2*cols, 1));
            remove_edge(edge(alpha+1 + 2*cols, alpha+1, 1));

        }
        else{
            alpha = (4*e.vlow + 2*cols) - 2 * (e.vhigh % cols);
            remove_edge(edge(alpha, alpha+1, 1));
            remove_edge(edge(alpha+1, alpha, 1));
            remove_edge(edge(alpha + 2*cols, alpha+1 + 2*cols, 1));
            remove_edge(edge(alpha+1 + 2*cols, alpha + 2*cols, 1));
        }
    }
}

void mst_path::reduce ()
{
    // iterate over path segments
    if (path.size() > 2) {
        for (auto it = path.begin()+1; it != path.end()-1; ) {
            // direction of path segments
            double dx1 = it->x - (it-1)->x;
            double dy1 = it->y - (it-1)->y;
            double dx2 = (it+1)->x - it->x;
            double dy2 = (it+1)->y - it->y;

            // two consecutive path segments are parallel, remove intermediate point
            if (atan2(dy1,dx1) == atan2(dy2,dx2))
                it = path.erase(it);

            else
                ++it;
        }
    }
}

bool mst_path::valid ()
{
    return 0 <= wp && wp < path.size();
}

void mst_path::add_edge (int from, int to, double cost)
{
    // add edge to priority queue
    edge e(from,to,cost,vertical);
    edges.insert(e);

    // add vertices to sets
    nodes[from].insert(to);
    nodes[to].insert(from);
}

double mst_path::dist (geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return hypot(p1.x - p2.x, p1.y - p2.y);
}

geometry_msgs::Point mst_path::get_wp (int offset)
{
    geometry_msgs::Point waypoint;

    if (0 <= wp+offset && wp+offset < path.size()) {
        // current waypoint
        geometry_msgs::Point p = path[wp+offset];

        // relative to original map
        p.x += map.info.origin.position.x;
        p.y += map.info.origin.position.y;

        // rotate
        waypoint.x = p.x * cos(-rotation) - p.y * sin(-rotation);
        waypoint.y = p.x * sin(-rotation) + p.y * cos(-rotation);
    }

    return waypoint;
}

geometry_msgs::Point mst_path::idx2wp (int index)
{
    geometry_msgs::Point waypoint;

    // convert index to relative waypoint position on map
    waypoint.x = (index % (2*map.info.width)) / 2.0 * map.info.resolution;
    waypoint.y = (index / (2*map.info.width)) / 2.0 * map.info.resolution;

    // shift waypoint to center path on map
    waypoint.x += map.info.resolution / 4;
    waypoint.y += map.info.resolution / 4;

    return waypoint;
}

void mst_path::remove_edge (edge e)
{
    if (edges.count(e) > 0) {
        edges.erase(e);
        nodes[e.vlow].erase(e.vhigh);
        nodes[e.vhigh].erase(e.vlow);
    }
}

double mst_path::round2idx (double pos)
{
    pos *= 2;
    pos = floor(pos);
    pos /= 2;

    return pos;
}
