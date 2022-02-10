#include "lib/auction_roi.h"

auction_roi::auction_roi ()
{
}

auction_roi::auction_roi (double distance, vector<geometry_msgs::Point> coords) : distance(distance), coords(coords)
{
    // generate id
    id = 0;
    for (auto c : coords)
        id += c.x * c.y;

    // calculate cost
    cost = distance;
}

void auction_roi::add (string cps)
{
    // add cps
    cpss.insert(cps);

    // update cost
    cost = distance * (1 + cpss.size());

}

vector<geometry_msgs::Point> auction_roi::get_coords ()
{
    return coords;
}

double auction_roi::get_cost ()
{
    return cost;
}

int auction_roi::get_id ()
{
    return id;
}
