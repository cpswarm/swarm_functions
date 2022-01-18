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
        // third, lower edges
        if (vhigh - vlow == e.vhigh - e.vlow)
            return vlow < e.vlow;

        // second, shorter edges
        else
            return vhigh - vlow < e.vhigh - e.vlow;
    }

    // first, edges with lower cost
    else
        return cost < e.cost;
}

bool compare_edge::operator() (const edge& a, const edge& b) const
{
    if (a.cost == b.cost) {
        // third, lower edges
        if (a.vhigh - a.vlow == b.vhigh - b.vlow)
            return a.vlow > b.vlow;

        // second, horizontal/vertical edges
        else {
            if (a.vertical || b.vertical)
                return a.vhigh - a.vlow < b.vhigh - b.vlow;
            else
                return a.vhigh - a.vlow > b.vhigh - b.vlow;
        }
    }

    // first, edges with lower cost
    else
        return a.cost > b.cost;
}

size_t hash_edge::operator() (const edge& e) const
{
    return hash<string>()(to_string(e.vlow) + to_string(e.vhigh) + to_string(e.cost));
}
