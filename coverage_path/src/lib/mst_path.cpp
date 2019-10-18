#include "lib/mst_path.h"

mst_path::mst_path ()
{
}

void mst_path::generate_path (geometry_msgs::Point start)
{
    // starting vertex
    geometry_msgs::Point wp = start;
    path.push_back(wp);
    int(round((wp.x - map.info.origin.position.x) / map.info.resolution));
    int(round((wp.y - map.info.origin.position.y) / map.info.resolution));
    int current = 2 * round((wp.y - map.info.origin.position.y) / map.info.resolution) * 2*map.info.width + 2 * round((wp.x - map.info.origin.position.x) / map.info.resolution);

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
        return;
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
            ROS_DEBUG("Path terminated at %d (%.2f,%.2f)", current, (current % (2*map.info.width)) / 2.0 * map.info.resolution + map.info.origin.position.x, (current / (2*map.info.width)) / 2.0 * map.info.resolution + map.info.origin.position.y);
            return;
        }

        ROS_DEBUG("Path at %d (%.2f,%.2f)", current, (current % (2*map.info.width)) / 2.0 * map.info.resolution + map.info.origin.position.x, (current / (2*map.info.width)) / 2.0 * map.info.resolution + map.info.origin.position.y);

        // remove vertices from sets
        nodes[current].erase(previous);
        nodes[previous].erase(current);

        // convert index to waypoint position on map
        wp.x = (current % (2*map.info.width)) / 2.0 * map.info.resolution + map.info.origin.position.x;
        wp.y = (current / (2*map.info.width)) / 2.0 * map.info.resolution + map.info.origin.position.y;

        // shift waypoint to center path on cell
        wp.x += 0.25 * map.info.resolution;
        wp.y += 0.25 * map.info.resolution;

        // add vertex to path
        path.push_back(wp);
    } while (found);

    // final waypoint
    wp = path.front();
    path.push_back(wp);
}

geometry_msgs::PoseArray mst_path::get_grid ()
{
    geometry_msgs::PoseArray grid;
    vector<geometry_msgs::Pose> poses;
    geometry_msgs::Pose pose;

    for (int i=0; i<nodes.size(); ++i) {
        // from
        pose.position.x = (i % (2*map.info.width)) / 2.0 * map.info.resolution + map.info.origin.position.x;
        pose.position.y = (i / (2*map.info.width)) / 2.0 * map.info.resolution + map.info.origin.position.y;

        // orientation
        for (auto it=nodes[i].begin(); it!=nodes[i].end(); ++it) {
            double dx = (*it % (2*map.info.width)) / 2.0 * map.info.resolution + map.info.origin.position.x - pose.position.x;
            double dy = (*it / (2*map.info.width)) / 2.0 * map.info.resolution + map.info.origin.position.y - pose.position.y;
            tf2::Quaternion direction;
            direction.setRPY(0, 0, atan2(dy, dx));
            pose.orientation = tf2::toMsg(direction);

            poses.push_back(pose);
        }
    }

    grid.poses = poses;
    grid.header.stamp = Time::now();
    grid.header.frame_id = "local_origin_ned";
    return grid;
}

nav_msgs::Path mst_path::get_path ()
{
    nav_msgs::Path nav_path;
    vector<geometry_msgs::PoseStamped> poses;
    geometry_msgs::PoseStamped pose;
    for (auto p : path) {
        pose.pose.position = p;
        poses.push_back(pose);
    }
    nav_path.poses = poses;
    nav_path.header.stamp = Time::now();
    nav_path.header.frame_id = "local_origin_ned";
    return nav_path;
}

geometry_msgs::Point mst_path::get_waypoint (geometry_msgs::Point position, double tolerance)
{
    // close enough to or past current waypoint
    if (dist(position, get_wp()) < tolerance || dist(position, get_wp(1)) < dist(position, get_wp())) {
        // select next waypoint
        ++wp;
    }

    return get_wp();
}

void mst_path::initialize_graph (nav_msgs::OccupancyGrid gridmap, bool connect4)
{
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

void mst_path::initialize_tree (vector<edge> mst)
{
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

bool mst_path::valid ()
{
    return 0 <= wp && wp < path.size();
}

void mst_path::add_edge (int from, int to, int cost)
{
    // add edge to priority queue
    edge e(from,to,cost);
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
        waypoint = path[wp+offset];
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
