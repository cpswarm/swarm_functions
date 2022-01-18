#include "lib/spanning_tree.h"

edge::edge (int v1, int v2, double c, bool v) : cost(c), vertical(v)
{
    vlow = min(v1, v2);
    vhigh = max(v1, v2);
}

bool edge::operator== (const edge &e) const
{
    return vlow == e.vlow && vhigh == e.vhigh && cost == e.cost;
}

bool edge::operator< (const edge &e) const
{
    if (cost == e.cost) {
        // third, edges on the top/right
        if (vhigh - vlow == e.vhigh - e.vlow)
            return vlow < e.vlow;

        // second, horizontal/vertical edges
        else {
            if (vertical)
                return vhigh - vlow <= e.vhigh - e.vlow;
            else
                return vhigh - vlow > e.vhigh - e.vlow;
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
    return hash<string>()(to_string(e.cost) + to_string(e.vlow) + to_string(e.vhigh));
}
