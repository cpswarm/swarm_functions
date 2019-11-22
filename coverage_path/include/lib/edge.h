#ifndef EDGE_H
#define EDGE_H

using namespace std;

/**
 * @brief A class for representing edges.
 */
class edge
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param f The starting vertex.
     * @param t The ending vertex.
     * @param c The cost of the edge.
     */
    edge(int f, int t, int c);

    /**
     * @brief Compare this edge to another one.
     * @param e The other edge to compare.
     * @return True, of both edges are identical, false otherwise.
     */
    bool operator== (const edge &e) const;

    /**
     * @brief The starting vertex of the edge.
     */
    int from;

    /**
     * @brief The ending vertex of the edge.
     */
    int to;

    /**
     * @brief The cost of the edge, i.e., length.
     */
    int cost;
};

/**
 * @brief A struct that provides the comparison of edge objects. It allows sorting of edges for priority queues. The sorting follows three rules: First, edges are sorted by ascending cost. Edges with the same cost are sorted preferring horizontal edges over vertical ones. Edges with same cost and orientation are sorted ascending by horizontal position.
 */
struct compare_edge
{
    /**
     * @brief Compare two edge objects. Edges are compared in terms of cost, orientation, and the position.
     * @param a First edge.
     * @param b Second edge.
     * @return True, if the cost of the first edge is lower than of the second edge. If both edges have the same cost, true if the difference between the vertex indexes of the first edge is greater than of the second edge. If both edges have the same cost and vertex index difference, true if the vertex index of first edge is lower than of the second edge. False otherwise.
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
