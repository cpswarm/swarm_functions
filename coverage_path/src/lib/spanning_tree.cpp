#include "lib/spanning_tree.h"

edge::edge (int f, int t, int c) : from(f), to(t), cost(c)
{
}

bool compare_edge::operator() (const edge& a, const edge& b) const
{
    return a.cost < b.cost;
}

spanning_tree::spanning_tree()
{
}

vector<edge> spanning_tree::get_mst_edges ()
{
    return new_edges;
}

nav_msgs::Path spanning_tree::get_path (nav_msgs::OccupancyGrid gridmap)
{
    nav_msgs::Path path;
    vector<geometry_msgs::PoseStamped> poses;
    geometry_msgs::PoseStamped pose;

    for (auto e : new_edges) {
        pose.pose.position.x = e.to % (2*gridmap.info.width) * gridmap.info.resolution + gridmap.info.origin.position.x;
        pose.pose.position.y = e.to / (2*gridmap.info.width) * gridmap.info.resolution + gridmap.info.origin.position.y;
        poses.push_back(pose);
    }

    path.poses = poses;
    path.header.stamp = Time::now();
    path.header.frame_id = "local_origin_ned";
    return path;
}

void spanning_tree::initialize_graph (int rows, int cols, valarray<bool> graph, bool connect4)
{
    nodes.resize(rows*cols);

    // iterate all rows and columns
    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols; j++) {
            // found a vertex
            if (graph[i*cols+j]) {
                // check von neumann neighborhood for connected vertices
                if (i>0 && graph[(i-1)*cols+j]) {
                    add_edge(i*cols+j, (i-1)*cols+j, 1);
                }
                if (i<rows-1 && graph[(i+1)*cols+j]) {
                    add_edge(i*cols+j, (i+1)*cols+j, 1);
                }
                if (j>0 && graph[i*cols+j-1]) {
                    add_edge(i*cols+j, i*cols+j-1, 1);
                }
                if (j<cols-1 && graph[i*cols+j+1]) {
                    add_edge(i*cols+j, i*cols+j+1, 1);
                }

                // check moore neighborhood for connected vertices
                if (!connect4) {
                    if (i>0 && j>0 && graph[(i-1)*cols+j-1]) {
                        add_edge(i*cols+j, (i-1)*cols+j-1, 1);
                    }
                    if (i<rows-1 && j<cols-1 && graph[(i+1)*cols+j+1]) {
                        add_edge(i*cols+j, (i+1)*cols+j+1, 1);
                    }
                    if (i<rows-1 && j>0 && graph[(i+1)*cols+j-1]) {
                        add_edge(i*cols+j, (i+1)*cols+j-1, 1);
                    }
                    if (i>0 && j<cols-1 && graph[(i-1)*cols+j+1]) {
                        add_edge(i*cols+j, (i-1)*cols+j+1, 1);
                    }
                }
            }
        }
    }
}

void spanning_tree::construct ()
{
    while (!edges.empty()) {
        // select shortest edge
        edge edge = edges.top();

        // edge connects different sets
        if (nodes[edge.from] != nodes[edge.to]) {
            unordered_set<int> src;
            int dst_idx;

            // unify the two sets
            if (nodes[edge.from].size() > nodes[edge.to].size()) {
                for (auto e : nodes[edge.to]) {
                    nodes[e] = nodes[edge.from];
                }
            }
            else {
                for (auto e : nodes[edge.from]) {
                    nodes[e] = nodes[edge.to];
                }
            }

            // add edge to mst
            new_edges.push_back(edge);
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
