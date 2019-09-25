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
    return a.cost < b.cost;
}

size_t hash_edge::operator() (const edge& e) const
{
    return hash<string>()(to_string(e.cost) + to_string(e.from) + to_string(e.to));
}
