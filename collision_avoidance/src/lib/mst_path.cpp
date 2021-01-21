#include "lib/mst_path.h"

mst_path::mst_path ()
{
}

bool mst_path::generate_path (geometry_msgs::Point start)
{
    // reset counter of current waypoint
    this->wp = 0;

    // starting vertex
    geometry_msgs::Point wp;
    wp.x = (start.x - origin.x) * cos(rotation) - (start.y - origin.y) * sin(rotation);
    wp.y = (start.x - origin.x) * sin(rotation) + (start.y - origin.y) * cos(rotation);
    path.push_back(wp);
    int current = 2 * round(wp.y / map.info.resolution) * 2*map.info.width + 2 * round(wp.x / map.info.resolution);

    // visited vertices
    unordered_set<int> removed;

    // previous vertex
    int previous;

    // last movement
    int offset;

    // possible movements
    vector<int> movement;
    movement.push_back(2*map.info.width);
    movement.push_back(-1);
    movement.push_back(-2*map.info.width);
    movement.push_back(1);

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
        ROS_ERROR("No path found!");
        return false;
    }

    // generate path
    do {
        // remember visited vertices
        removed.insert(current);

        // last movement
        offset = distance(movement.begin(), find(movement.begin(), movement.end(), previous-current));

        // look for valid step
        found = false;
        previous = current;
        for (int idx=0; idx<movement.size(); idx++){
            if (nodes[previous].count(previous + movement[(idx+offset) % movement.size()]) > 0 &&
                removed.count(previous + movement[(idx+offset) % movement.size()]) <= 0) {
                current = previous + movement[(idx+offset) % movement.size()];
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

        // convert index to relative waypoint position on map
        wp.x = (current % (2*map.info.width)) / 2.0 * map.info.resolution;
        wp.y = (current / (2*map.info.width)) / 2.0 * map.info.resolution;

        // shift waypoint to center path on map
        double width_path = (round(width / map.info.resolution) * 2 - 1) / 2.0 * map.info.resolution;
        double height_path = (round(height / map.info.resolution) * 2 - 1) / 2.0 * map.info.resolution;
        wp.x += (width - width_path) / 2.0;
        wp.y += (height - height_path) / 2.0;

        // add vertex to path
        path.push_back(wp);
    } while (found);

    // final waypoint
    wp = path.front();
    path.push_back(wp);

    return true;
}

nav_msgs::Path mst_path::get_path ()
{
    nav_msgs::Path nav_path;
    vector<geometry_msgs::PoseStamped> poses;
    geometry_msgs::PoseStamped pose;
    for (auto p : path) {
        // rotate
        pose.pose.position.x = p.x * cos(-rotation) - p.y * sin(-rotation);
        pose.pose.position.y = p.x * sin(-rotation) + p.y * cos(-rotation);

        // relative to original map
        pose.pose.position.x += origin.x;
        pose.pose.position.y += origin.y;

        poses.push_back(pose);
    }
    nav_path.poses = poses;
    nav_path.header.stamp = Time::now();
    nav_path.header.frame_id = "local_origin_ned";
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

void mst_path::initialize_graph (nav_msgs::OccupancyGrid gridmap, bool vertical, bool connect4)
{
    ROS_DEBUG("Gridmap size %dx%d origin (%.2f,%.2f)", gridmap.info.width, gridmap.info.height, gridmap.info.origin.position.x, gridmap.info.origin.position.y);
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
                        add_edge(i*2*cols+j, (i-1)*2*cols+j-1, 1);
                    }
                    if (i<2*rows-1 && j<2*cols-1 && inflated[(i+1)*2*cols+j+1]) {
                        add_edge(i*2*cols+j, (i+1)*2*cols+j+1, 1);
                    }
                    if (i<2*rows-1 && j>0 && inflated[(i+1)*2*cols+j-1]) {
                        add_edge(i*2*cols+j, (i+1)*2*cols+j-1, 1);
                    }
                    if (i>0 && j<2*cols-1 && inflated[(i-1)*2*cols+j+1]) {
                        add_edge(i*2*cols+j, (i-1)*2*cols+j+1, 1);
                    }
                }
            }
        }
    }
}

void mst_path::initialize_map (geometry_msgs::Point origin, double rotation, double width, double height)
{
    // bottom left of area
    this->origin = origin;

    // rotation of map
    this->rotation = rotation;

    // dimensions of area
    this->width = width;
    this->height = height;

    ROS_DEBUG("Area size %.2fx%.2f origin (%.2f,%.2f)", width, height, origin.x, origin.y);
}

void mst_path::initialize_tree (vector<edge> mst)
{
    // remove edges not required by mst
    int cols = map.info.width;
    int alpha, vmax, vmin;
    for (int i=0; i<mst.size(); i++) {
        vmax = max(mst[i].from, mst[i].to);
        vmin = min(mst[i].from, mst[i].to);

        if (vmax - vmin == 1) {
            alpha = (4*vmin + 3) - 2 * (vmax % cols);
            remove_edge(edge(alpha, alpha + 2*cols, 1));
            remove_edge(edge(alpha + 2*cols, alpha, 1));
            remove_edge(edge(alpha+1, alpha+1 + 2*cols, 1));
            remove_edge(edge(alpha+1 + 2*cols, alpha+1, 1));

        }
        else{
            alpha = (4*vmin + 2*cols) - 2 * (vmax % cols);
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

void mst_path::add_edge (int from, int to, int cost)
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
        // rotate
        waypoint.x = path[wp+offset].x * cos(-rotation) - path[wp+offset].y * sin(-rotation);
        waypoint.y = path[wp+offset].x * sin(-rotation) + path[wp+offset].y * cos(-rotation);

        // relative to original map
        waypoint.x += origin.x;
        waypoint.y += origin.y;
    }

    return waypoint;
}

void mst_path::remove_edge (edge e)
{
    if (edges.count(e) > 0) {
        edges.erase(e);
        nodes[e.from].erase(e.to);
        nodes[e.to].erase(e.from);
    }
}
