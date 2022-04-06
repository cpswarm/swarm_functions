#ifndef EDGE_H
#define EDGE_H

using namespace std;

/**
 * @brief A class for representing undirected edges.
 */
class edge
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param v1 The first vertex.
     * @param v2 The second vertex.
     * @param c The cost of the edge.
     * @param v Whether the sweeping pattern is vertical or horizontal. Default horizontal.
     */
    edge(int v1, int v2, double c, bool v = false);

    /**
     * @brief Compare this edge to another one.
     * @param e The other edge to compare.
     * @return True, of both edges are identical, false otherwise.
     */
    bool operator== (const edge &e) const;

    /**
     * @brief Compare this edge to another one.
     * @param e The other edge to compare.
     * @return True, if the cost of the first edge is lower than of the second edge. If both edges have the same cost, true if the difference between the vertex indexes of the first edge is lower than of the second edge. If both edges have the same cost and vertex index difference, true if the vertex indexes of the first edge are lower than of the second edge. False otherwise.
     */
    bool operator< (const edge &e) const;

    /**
     * @brief The lower vertex of the edge.
     */
    int vlow;

    /**
     * @brief The higher vertex of the edge.
     */
    int vhigh;

    /**
     * @brief The cost of the edge, e.g., length.
     */
    double cost;

    /**
     * @brief Whether the sweeping pattern is vertical or horizontal.
     */
    bool vertical;
};

/**
 * @brief A struct that provides the comparison of edge objects. It allows sorting of edges for priority queues. The sorting follows three rules: First, edges are sorted by ascending cost. Edges with the same cost are sorted preferring horizontal/vertical edges. Edges with same cost and orientation are sorted ascending by index position, i.e., first bottom to top, then left to right.
 */
struct compare_edge
{
    /**
     * @brief Compare two edge objects. Edges are compared in terms of cost, orientation, and the position.
     * @param a First edge.
     * @param b Second edge.
     * @return True, if the first edge is greater than the second edge. False otherwise.
     */
    bool operator() (const edge& a, const edge& b) const;
};

/**
 * @brief A struct that provides the hash function of edge objects.
 */
struct hash_edge
{
    /**
     * @brief Generate the hash of an edge.
     * @param e The edge for which to compute the hash.
     * @return The hash of the edge.
     */
    size_t operator() (const edge& e) const;
};

#endif // EDGE_H
