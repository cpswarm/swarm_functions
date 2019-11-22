#include "lib/spanning_tree.h"

edge::edge (int f, int t, int c) : from(f), to(t), cost(c)
{
}

bool edge::operator== (const edge &e) const
{
    return from == e.from && to == e.to && cost == e.cost;
}

bool compare_edge::operator() (const edge& a, const edge& b) const
{
    if (a.cost == b.cost) {
        if (abs(a.to - a.from) == abs(b.to - b.from))
            // third, edges on the right
            return a.from < b.from;

        // second, horizontal edges
        else
            return abs(a.to - a.from) > abs(b.to - b.from);
    }

    // first, edges with lowest cost
    else
        return a.cost < b.cost;
}

size_t hash_edge::operator() (const edge& e) const
{
    return hash<string>()(to_string(e.cost) + to_string(e.from) + to_string(e.to));
}
