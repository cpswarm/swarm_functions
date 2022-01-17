#include "lib/spanning_tree.h"

edge::edge (int f, int t, int c, bool v) : from(f), to(t), cost(c), vertical(v)
{
}

bool edge::operator== (const edge &e) const
{
    return from == e.from && to == e.to && cost == e.cost;
}

bool edge::operator< (const edge &e) const
{
    if (cost == e.cost) {
        // third, edges on the top/right
        if (abs(to - from) == abs(e.to - e.from))
            return from < e.from;

        // second, horizontal/vertical edges
        else {
            if (vertical)
                return abs(to - from) <= abs(e.to - e.from);
            else
                return abs(to - from) > abs(e.to - e.from);
        }
    }

    // first, edges with lowest cost
    else
        return cost < e.cost;
}

bool compare_edge::operator() (const edge& a, const edge& b) const
{
    return a < b;
}

size_t hash_edge::operator() (const edge& e) const
{
    return hash<string>()(to_string(e.cost) + to_string(e.from) + to_string(e.to));
}
