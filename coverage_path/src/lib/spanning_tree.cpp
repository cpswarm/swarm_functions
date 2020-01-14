#include "lib/spanning_tree.h"

spanning_tree::spanning_tree()
{
}

vector<edge> spanning_tree::get_mst_edges ()
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
        double x = e.from % map.info.width * map.info.resolution + map.info.origin.position.x;
        double y = e.from / map.info.width * map.info.resolution + map.info.origin.position.y;

        // orientation
        double dx = e.to % map.info.width * map.info.resolution + map.info.origin.position.x - x;
        double dy = e.to / map.info.width * map.info.resolution + map.info.origin.position.y - y;
        tf2::Quaternion direction;
        direction.setRPY(0, 0, atan2(dy, dx) + rotation);
        pose.orientation = tf2::toMsg(direction);

        // translate
        x += translation.x;
        y += translation.y;

        // rotate
        pose.position.x = x*cos(rotation) - y*sin(rotation);
        pose.position.y = x*sin(rotation) + y*cos(rotation);

        poses.push_back(pose);
    }

    path.poses = poses;
    path.header.stamp = Time::now();
    path.header.frame_id = "local_origin_ned";
    return path;
}

void spanning_tree::initialize_graph (nav_msgs::OccupancyGrid gridmap, geometry_msgs::Vector3 vec, double angle, bool connect4)
{
    // transformation of output tree
    rotation = -angle;
    translation.x = -vec.x;
    translation.y = -vec.y;

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
                        add_edge(i*cols+j, (i-1)*cols+j-1, 1);
                    }
                    if (i<rows-1 && j<cols-1 && map.data[(i+1)*cols+j+1] == 0) {
                        add_edge(i*cols+j, (i+1)*cols+j+1, 1);
                    }
                    if (i<rows-1 && j>0 && map.data[(i+1)*cols+j-1] == 0) {
                        add_edge(i*cols+j, (i+1)*cols+j-1, 1);
                    }
                    if (i>0 && j<cols-1 && map.data[(i-1)*cols+j+1] == 0) {
                        add_edge(i*cols+j, (i-1)*cols+j+1, 1);
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
        if (nodes[edge.from] != nodes[edge.to]) {
            // combine the two sets
            unordered_set<int> s(nodes[edge.from].begin(), nodes[edge.from].end());
            for (auto v : nodes[edge.to])
                s.insert(v);
            for (auto v : s)
                nodes[v] = s;

            // add edge to mst
            mst_edges.push_back(edge);
        }

        // remove edge from source tree
        edges.pop();
    }
}

void spanning_tree::add_edge (int from, int to, int cost)
{
    // add edge to priority queue
    edges.push(edge(from, to, cost));

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
