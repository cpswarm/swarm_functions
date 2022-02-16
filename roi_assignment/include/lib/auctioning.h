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
     * @brief Get the ROI that this CPS is assigned to.
     * @throws runtime_error if no auction is running or this CPS did not win.
     * @return An ROI ID.
     */
    string get_assigned ();

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
     * @throws runtime_error if the auction is not running (anymore).
     */
    void participant (string roi, string cps, double bid);

    /**
     * @brief Check whether this CPS has won the auction.
     * @return True, if the auction is closed and this CPS has placed the highest bid. False otherwise.
     */
    bool won ();

private:
    /**
     * @brief The auction currently held by this CPS.
     */
    auction current;

    /**
     * @brief The UUID if this CPS.
     */
    string self;
};

#endif // ROI_ASSIGNMENT_AUCTIONING_H
