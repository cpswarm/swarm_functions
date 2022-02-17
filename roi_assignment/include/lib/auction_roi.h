#ifndef AUCTION_ROI_H
#define AUCTION_ROI_H

#include <string>
#include <set>
#include <vector>
#include <geometry_msgs/Point.h>

using namespace std;

/**
 * @brief A class that stores the data of a region of interest (ROI) during auctioning.
 */
class auction_roi
{
public:
    /**
     * @brief Constructor.
     */
    auction_roi ();

    /**
     * @brief Constructor that initializes the private members.
     *
     * @param distance The distance of the CPS to this ROI.
     * @param coords The coordinates defining the ROI.
     * @param cost_param Parametrization of the ROI selection cost function [0,1]. 0: only distance matters, 1: only agent density matters.
     */
    auction_roi (double distance, vector<geometry_msgs::Point> coords, double cost_param);

    /**
     * @brief Add a CPS that has been assigned to this ROI.
     * @param cps The UUID of the CPS.
     */
    void add (string cps);

    /**
     * @brief Get the coordinates that define the ROI.
     * @return A vector of points.
     */
    vector<geometry_msgs::Point> get_coords ();

    /**
     * @brief Get the cost associated to the ROI by this CPS.
     * @return The product of the distance and the number of CPS assigned to this ROI.
     */
    double get_cost ();

    /**
     * @brief Get the ID of this ROI.
     * @return A hash value that (more or less) uniquely represents this ROI.
     */
    string get_id ();

private:
    /**
     * @brief The ID for referencing this ROI. It is a hash value that (more or less) uniquely represents this ROI.
     */
    string id;

    /**
     * @brief The cost associated to this ROI by this CPS. It is the product of the distance and the number of CPS assigned to this ROI.
     */
    double cost;

    /**
     * @brief The coordinates that define this ROI.
     */
    vector<geometry_msgs::Point> coords;

    /**
     * @brief The set of CPSs assigned to this ROI.
     */
    set<string> cpss;

    /**
     * @brief The distance of the CPS to this ROI.
     */
    double distance;

    /**
     * @brief Parametrization of the ROI selection cost function [0,1]. 0: only distance matters, 1: only agent density matters.
     */
    double cost_param;
};

#endif // AUCTION_ROI_H
