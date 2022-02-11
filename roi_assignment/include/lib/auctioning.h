#ifndef ROI_ASSIGNMENT_AUCTIONING_H
#define ROI_ASSIGNMENT_AUCTIONING_H

#include <string>
#include <map>
#include "lib/auction.h"

using namespace std;

/**
 * @brief A class to perform the auctioning during ROI assignment.
 */
class auctioning
{
public:
    /**
     * @brief Constructor.
     * @param self The UUID of this CPS.
     */
    auctioning (string self);

    /**
     * @brief Get the result of the latest auction held by this CPS.
     * @throws runtime_error if no auction was running previously or if an auction is currently running.
     * @return An auction object with the results.
     */
    auction get_result ();

    /**
     * @brief Get the ROI that this CPS has placed the highest bid for. Running or closed auctions. Auctions held by this CPS or other CPSs.
     * @return The ROI ID.
     */
    string get_roi ();

    /**
     * @brief Get the auction that is currently running.
     * @throws runtime_error if no auction is running.
     * @return An auction object.
     */
    auction get_running ();

    /**
     * @brief Start an auction.
     * @param roi The ID of the ROI that shall be auctioned.
     * @param bid The starting bid for the ROI.
     * @param timeout The duration of the auction.
     * @throws runtime_error if there is already an auction running.
     */
    void initiate (string roi, double bid, Duration timeout);

    /**
     * @brief Check whether there is currently an auction running.
     * @return True, if this CPS is holding an auction that already started but did not finish yet. False otherwise.
     */
    bool is_running ();

    /**
     * @brief Process a bid of another CPS for an auction held by this CPS.
     * @param roi The ID of the ROI that is auctioned.
     * @param cps The UUID of the CPS placing the bid.
     * @param bid The value of the bid.
     * @throws runtime_error if the auction is not running.
     */
    void participant (string roi, string cps, double bid);

    /**
     * @brief Store the auction held by another CPS.
     * @param roi The ID of the ROI that is auctioned.
     * @param auctioneer The UUID of the CPS holding the auction.
     * @param bid The bid placed by this CPS.
     */
    void participate (string roi, string auctioneer, double bid);

    /**
     * @brief Store the result of an auction held by another CPS.
     * @param roi The ID of the ROI that is auctioned.
     * @param auctioneer The UUID of the CPS holding the auction.
     * @param winner The UUID of the CPS that won the auction.
     * @throws runtime_error if no auction was opened by the auctioneer, if the auctioneer was auctioning another ROI, or a result has already been received.
     */
    void set_result (string roi, string auctioneer, string winner);

    /**
     * @brief Check whether this CPS has won an auction.
     * @return True, if there is at least one auction where this CPS has placed the highest bid. False otherwise.
     */
    bool won ();

private:
    /**
     * @brief The mapping between CPS UUID and auction.
     */
    map<string,auction> auctions;

    /**
     * @brief The UUID if this CPS.
     */
    string self;
};

#endif // ROI_ASSIGNMENT_AUCTIONING_H
