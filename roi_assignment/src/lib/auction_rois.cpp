#include "lib/auction_rois.h"

auction_rois::auction_rois ()
{
}

void auction_rois::add (int roi, string cps)
{
    if (roi_map.count(roi) < 1)
        throw runtime_error("Unknown ROI " + to_string(roi));

    roi_map[roi].add(cps);
}

double auction_rois::bid (int roi)
{
    if (roi_map.count(roi) < 1)
        throw runtime_error("Unknown ROI " + to_string(roi));

    return 1.0 / roi_map[roi].get_cost();
}

vector<geometry_msgs::Point> auction_rois::get_coords (int roi)
{
    if (roi_map.count(roi) < 1)
        throw runtime_error("Unknown ROI " + to_string(roi));

    return roi_map[roi].get_coords();
}

void auction_rois::init (vector<pair<double,vector<geometry_msgs::Point>>> coords)
{
    // populate roi map from coords
    for (auto c : coords) {
        auction_roi roi(c.first, c.second);
        roi_map[roi.get_id()] = roi;
    }
}

auction_roi auction_rois::select ()
{
    auction_roi cheapest;

    if (roi_map.size() == 0)
        throw runtime_error("No ROIs available");

    // get roi with lowest cost
    for (auto r : roi_map) {
        if (cheapest.get_cost() == 0 || r.second.get_cost() < cheapest.get_cost()) {
            cheapest = r.second;
        }
    }

    return cheapest;
}
