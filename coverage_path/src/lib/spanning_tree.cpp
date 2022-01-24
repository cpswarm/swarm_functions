#include "lib/spanning_tree.h"

spanning_tree::spanning_tree()
{
}

set<edge> spanning_tree::get_mst_edges ()
{
    return mst_edges;
}

geometry_msgs::PoseArray spanning_tree::get_tree ()
{
    geometry_msgs::PoseArray path;
    vector<geometry_msgs::Pose> poses;
    geometry_msgs::Pose pose;

    for (auto e : mst_edges) {
        // from
        pose.position.x = e.vlow % map.info.width * map.info.resolution + map.info.origin.position.x;
        pose.position.y = e.vlow / map.info.width * map.info.resolution + map.info.origin.position.y;

        // orientation
        double dx = e.vhigh % map.info.width * map.info.resolution + map.info.origin.position.x - pose.position.x;
        double dy = e.vhigh / map.info.width * map.info.resolution + map.info.origin.position.y - pose.position.y;
        tf2::Quaternion direction;
        direction.setRPY(0, 0, atan2(dy, dx));
        pose.orientation = tf2::toMsg(direction);

        // offset for visualization
        pose.position.x += 0.5 * map.info.resolution;
        pose.position.y += 0.5 * map.info.resolution;

        poses.push_back(pose);
    }

    path.poses = poses;
    path.header.stamp = Time::now();
    path.header.frame_id = "map";
    return path;
}

void spanning_tree::initialize_graph (nav_msgs::OccupancyGrid gridmap, bool vertical, bool connect4)
{
    // orientation of tree edges
    this->vertical = vertical;

    // initialize map
    map = gridmap;
    int rows = gridmap.info.height;
    int cols = gridmap.info.width;

    // empty containers
    nodes.clear();
    nodes.resize(rows*cols);
    edges = priority_queue<edge, vector<edge>, compare_edge>();

    // iterate all rows and columns
    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols; j++) {
            // found a vertex
            if (map.data[i*cols+j] == 0) {
                // check von neumann neighborhood for connected vertices
                if (i>0 && map.data[(i-1)*cols+j] == 0) {
                    add_edge(i*cols+j, (i-1)*cols+j, 1);
                }
                if (i<rows-1 && map.data[(i+1)*cols+j] == 0) {
                    add_edge(i*cols+j, (i+1)*cols+j, 1);
                }
                if (j>0 && map.data[i*cols+j-1] == 0) {
                    add_edge(i*cols+j, i*cols+j-1, 1);
                }
                if (j<cols-1 && map.data[i*cols+j+1] == 0) {
                    add_edge(i*cols+j, i*cols+j+1, 1);
                }

                // check moore neighborhood for connected vertices
                if (!connect4) {
                    if (i>0 && j>0 && map.data[(i-1)*cols+j-1] == 0) {
                        add_edge(i*cols+j, (i-1)*cols+j-1, sqrt(2));
                    }
                    if (i<rows-1 && j<cols-1 && map.data[(i+1)*cols+j+1] == 0) {
                        add_edge(i*cols+j, (i+1)*cols+j+1, sqrt(2));
                    }
                    if (i<rows-1 && j>0 && map.data[(i+1)*cols+j-1] == 0) {
                        add_edge(i*cols+j, (i+1)*cols+j-1, sqrt(2));
                    }
                    if (i>0 && j<cols-1 && map.data[(i-1)*cols+j+1] == 0) {
                        add_edge(i*cols+j, (i-1)*cols+j+1, sqrt(2));
                    }
                }
            }
        }
    }
}

void spanning_tree::construct ()
{
    mst_edges.clear();

    while (!edges.empty()) {
        // select shortest edge
        edge edge = edges.top();

        // edge connects different sets
        if (nodes[edge.vlow] != nodes[edge.vhigh]) {
            // combine the two sets
            unordered_set<int> s(nodes[edge.vlow].begin(), nodes[edge.vlow].end());
            for (auto v : nodes[edge.vhigh])
                s.insert(v);
            for (auto v : s)
                nodes[v] = s;

            // add edge to mst
            mst_edges.insert(edge);
        }

        // remove edge from source tree
        edges.pop();
    }
}

void spanning_tree::add_edge (int from, int to, double cost)
{
    // add edge to priority queue
    edges.push(edge(from, to, cost, vertical));

    // create singleton set for both vertices
    if (nodes[from].size() == 0) {
        nodes[from].insert(from);
    }
    if (nodes[to].size() == 0) {
        nodes[to].insert(to);
    }
}

bool spanning_tree::different_sets (int a, int b)
{
    return nodes[a] != nodes[b];
}
