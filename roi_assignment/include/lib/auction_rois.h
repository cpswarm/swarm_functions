#ifndef AUCTION_ROIS_H
#define AUCTION_ROIS_H

#include <string>
#include <map>
#include "lib/auction_roi.h"

using namespace std;

/**
 * @brief A class that stores the data of all known regions of interest (ROIs) during auctioning.
 */
class auction_rois
{
public:
    /**
     * @brief Constructor.
     */
    auction_rois ();

    /**
     * @brief Add a CPS that has been assigned to this a ROI.
     * @param roi The ROI ID that the CPS has been assigned to.
     * @param cps The CPS that has been assigned.
     * @throws runtime_error if ROI is unknown.
     */
    void add (string roi, string cps);

    /**
     * @brief Get the bid of this CPS for a ROI.
     * @param roi The ROI ID to get the bid for.
     * @throws runtime_error if ROI is unknown.
     * @return The inverse of the ROI cost.
     */
    double bid (string roi);

    /**
     * @brief Get the coordinates defining a ROI.
     * @param roi The ROI ID to get the coordinates for.
     * @throws runtime_error if ROI is unknown.
     * @return A vector of points.
     */
    vector<geometry_msgs::Point> get_coords (string roi);

    /**
     * @brief Initialize the class.
     * @param coords All ROIs, given as distance from the CPS's current position and a vector of coordinates.
     */
    void init (vector<pair<double,vector<geometry_msgs::Point>>> coords);

    /**
     * @brief Select the ROI with least cost.
     * @throws runtime_error if no ROIs are available.
     * @return An object representing the ROI.
     */
    auction_roi select ();

private:
    /**
     * @brief The mapping between ROI IDs and ROI objects.
     */
    map<string,auction_roi> roi_map;
};

#endif // AUCTION_ROIS_H
