#include "lib/auction_roi.h"

auction_roi::auction_roi ()
{
}

auction_roi::auction_roi (double distance, vector<geometry_msgs::Point> coords, double cost_param) : distance(distance), coords(coords), cost_param(cost_param)
{
    // sort coordinates for id generation
    set<pair<double,double>> coords_set;
    for (auto c : coords)
        coords_set.emplace(make_pair(c.x, c.y));

    // generate id
    id = "";
    for (auto c : coords_set)
        id += to_string(c.first) + "," + to_string(c.second) + " ";

    // calculate cost
    cost = pow(distance, 1-cost_param);
}

void auction_roi::add (string cps)
{
    // add cps
    cpss.insert(cps);

    // update cost
    cost = pow(distance, 1-cost_param) * pow(1 + cpss.size(), cost_param);

}

vector<geometry_msgs::Point> auction_roi::get_coords ()
{
    return coords;
}

double auction_roi::get_cost ()
{
    return cost;
}

string auction_roi::get_id ()
{
    return id;
}
