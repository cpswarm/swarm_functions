#include "lib/spanning_tree.h"

edge::edge (int f, int t, int c, bool v) : from(f), to(t), cost(c), vertical(v)
{
}

bool edge::operator== (const edge &e) const
{
    return from == e.from && to == e.to && cost == e.cost;
}

bool compare_edge::operator() (const edge& a, const edge& b) const
{
    if (a.cost == b.cost) {
        // third, edges on the top/right
        if (abs(a.to - a.from) == abs(b.to - b.from))
            return a.from < b.from;

        // second, horizontal/vertical edges
        else {
            if (a.vertical)
                return abs(a.to - a.from) <= abs(b.to - b.from);
            else
                return abs(a.to - a.from) > abs(b.to - b.from);
        }
    }

    // first, edges with lowest cost
    else
        return a.cost < b.cost;
}

size_t hash_edge::operator() (const edge& e) const
{
    return hash<string>()(to_string(e.cost) + to_string(e.from) + to_string(e.to));
}
