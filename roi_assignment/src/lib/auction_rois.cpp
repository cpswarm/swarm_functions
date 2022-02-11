#include "lib/auction_rois.h"

auction_rois::auction_rois ()
{
}

void auction_rois::add (string roi, string cps)
{
    if (roi_map.count(roi) < 1)
        throw runtime_error("Unknown ROI " + roi);

    roi_map[roi].add(cps);
}

double auction_rois::bid (string roi)
{
    if (roi_map.count(roi) < 1)
        throw runtime_error("Unknown ROI " + roi);

    // no cost roi
    if (roi_map[roi].get_cost() <= 0)
        return numeric_limits<double>::max();

    return 1.0 / roi_map[roi].get_cost();
}

vector<geometry_msgs::Point> auction_rois::get_coords (string roi)
{
    if (roi_map.count(roi) < 1)
        throw runtime_error("Unknown ROI " + roi);

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
        if (cheapest.get_id() == "" || r.second.get_cost() < cheapest.get_cost()) {
            cheapest = r.second;
        }
    }

    return cheapest;
}
