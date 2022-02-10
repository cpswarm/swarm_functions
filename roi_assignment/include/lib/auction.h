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
    auction () {}

    /**
     * @brief Constructor that initializes some member variables.
     * @param roi The ID of the ROI at auction.
     * @param auctioneer The UUID of the auctioning CPS.
     * @param initial_bid The starting bid of the auction.
     */
    auction (int roi, string auctioneer, double initial_bid) : roi(roi), auctioneer(auctioneer), bid(initial_bid) {}

    /**
     * @brief Constructor that initializes the member variables.
     * @param roi The ID of the ROI at auction.
     * @param auctioneer The UUID of the auctioning CPS.
     * @param initial_bid The starting bid of the auction.
     * @param start The starting time of the auction.
     * @param end The closing time of the auction.
     */
    auction (int roi, string auctioneer, double initial_bid, Time start, Time end) : roi(roi), auctioneer(auctioneer), bid(initial_bid), start(start), end(end) {}

    /**
     * @brief The ID of the ROI at auction.
     */
    int roi;

    /**
     * @param auctioneer The UUID of the auctioning CPS.
     */
    string auctioneer;

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
