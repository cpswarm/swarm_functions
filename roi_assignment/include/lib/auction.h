#ifndef ROI_ASSIGNMENT_AUCTION_H
#define ROI_ASSIGNMENT_AUCTION_H

#include <ros/ros.h>
#include <time.h>

using namespace std;
using namespace ros;

/**
 * @brief A structure that stores the data of a ROI auction.
 */
struct auction
{
    /**
     * @brief Constructor.
     */
    auction () : roi(""), bid(0), winner(""), start(Time(0)), end(Time(0)) {}

    /**
     * @brief Constructor that initializes the member variables.
     * @param roi The ID of the ROI at auction.
     * @param initial_bid The starting bid of the auction.
     * @param start The starting time of the auction.
     * @param end The closing time of the auction.
     */
    auction (string roi, double initial_bid, Time start, Time end) : roi(roi), bid(initial_bid), winner(""), start(start), end(end) {}

    /**
     * @brief Test whether an object represents a valid auction.
     * @return True if all members have been initialized properly. False otherwise.
     */
    bool is_valid ()
    {
        return roi != "" && bid > 0 && start.toSec() > 0 && end.toSec() > 0;
    }

    /**
     * @brief The ID of the ROI at auction.
     */
    string roi;

    /**
     * @brief The current highest bid.
     */
    double bid;

    /**
     * @brief The UUID of the CPS that is currently holding the highest bid.
     */
    string winner;

    /**
     * @param start The starting time of the auction.
     */
    Time start;

    /**
     * @param end The closing time of the auction.
     */
    Time end;
};

#endif // ROI_ASSIGNMENT_AUCTION_H
