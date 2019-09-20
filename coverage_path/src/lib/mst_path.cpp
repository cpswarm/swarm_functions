#include "lib/mst_path.h"

mst_path::mst_path ()
{
}

geometry_msgs::Point mst_path::current_wp ()
{
    return get_wp(wp);
}

void mst_path::generate_path (geometry_msgs::Point start)
{
    // starting vertex
    geometry_msgs::Point wp = start;
    path.push_back(wp);
    int(round((wp.x - origin.x) / res));
    int(round((wp.y - origin.y) / res));
    int current = 2 * round((wp.y - origin.y) / res) * 2*cols + 2 * round((wp.x - origin.x) / res);

//     for (int i=0; i<500; ++i) {
//         if (nodes[i].size() > 0) {
//             cout << i << ": ";
//             for (auto n : nodes[i])
//                 cout << n << ",";
//             cout << endl;
//         }
//     }

    // visited vertices
    unordered_set<int> removed;

    // previous vertex
    int previous;

    // last movement
    int offset;

    // possible movements
    vector<int> movement;
    movement.push_back(2*cols);
    movement.push_back(-1);
    movement.push_back(-2*cols);
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
            return;
        }

        // remove vertices from sets
        nodes[current].erase(previous);
        nodes[previous].erase(current);

        // add vertex to path
        wp.x = (current % (2*cols)) / 2.0 * res + origin.x;
        wp.y = (current / (2*cols)) / 2.0 * res + origin.y;

        path.push_back(wp);
    } while (found);

    // final waypoint
    wp = path.front();
    path.push_back(wp);
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

void mst_path::initialize_graph (valarray<bool> graph, bool connect4)
{
    nodes.clear();
    nodes.resize(2*rows*2*cols);
    edges.clear();
    path.clear();

    // inflate graph
    valarray<bool> inflated(2*rows*2*cols);
    for (int i=0; i<2*rows; i++) {
        for (int j=0; j<2*cols; j++) {
            inflated[i*2*cols + j] = graph[(i/2)*cols + j/2];
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

void mst_path::initialize_map(nav_msgs::OccupancyGrid gridmap)
{
//     rows = gridmap.info.width;
//     cols = gridmap.info.height;
    rows = gridmap.info.height;
    cols = gridmap.info.width;
    res = gridmap.info.resolution;
    origin = gridmap.info.origin.position;
}

void mst_path::initialize_tree (vector<edge> mst)
{
    this->mst = mst;
}

geometry_msgs::Point mst_path::next_wp ()
{
    return get_wp(++wp);
}

geometry_msgs::Point mst_path::previous_wp ()
{
    return get_wp(wp-1);
}

void mst_path::remove_edges ()
{
    int alpha, vmax, vmin;
    for (int i=0; i<mst.size(); i++) {
        vmax = max(mst[i].from, mst[i].to);
        vmin = min(mst[i].from, mst[i].to);

        if (abs(mst[i].from - mst[i].to) == 1) {
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

void mst_path::step ()
{
    ++wp;
}

void mst_path::add_edge (int from, int to, int cost)
{
    // add edge to priority queue
    edges.insert(edge(from, to, cost));

    // add vertices to sets
    nodes[from].insert(to);
    nodes[to].insert(from);
}

geometry_msgs::Point mst_path::get_wp (int idx)
{
    geometry_msgs::Point waypoint;

    if (0 <= idx && idx < path.size()) {
        waypoint = path[idx];
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
