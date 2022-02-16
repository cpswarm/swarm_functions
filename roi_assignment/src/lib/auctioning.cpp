#include "lib/auctioning.h"

auctioning::auctioning (string self) : self(self)
{
}

string auctioning::get_assigned ()
{
    // no auction running at all
    if (current.is_valid() == false)
        throw runtime_error("No auction running");

    // auction still running
    if (is_running())
        throw runtime_error("Auction for ROI " + current.roi + " still running");

    // auction still running
    if (current.winner != self)
        throw runtime_error("Auction for ROI " + current.roi + " lost");

    return current.roi;
}

auction auctioning::get_running ()
{
    // no auction running at all
    if (current.is_valid() == false)
        throw runtime_error("No auction running");

    // auction not running (anymore)
    if (is_running() == false)
        throw runtime_error("Auction for ROI " + current.roi + " not running");

    return current;
}

void auctioning::initiate (string roi, double bid, Duration timeout)
{
    // already running an auction
    if (is_running())
        throw runtime_error("Already running an auction for ROI " + current.roi);

    // create auction
    auction auction(roi, bid, Time::now(), Time::now()+timeout);
    auction.winner = self; // and make this cps the winner by default

    current = auction;
}

bool auctioning::is_running ()
{
    return current.is_valid() && current.start <= Time::now() && Time::now() <= current.end;
}

void auctioning::participant (string roi, string cps, double bid)
{
    // ignore bids if not auctioning
    if (current.is_valid() == false)
        return;

    // ignore bids for other rois
    if (roi != current.roi)
        return;

    // auction closed
    if (is_running() == false)
        throw runtime_error("Received from " + cps + " bid for ROI " + roi + ", but auction already closed (" + to_string(Time::now().toSec()) + "<" + to_string(current.end.toSec()) + ")");

    // ignore lower bids
    if(bid <= current.bid)
        return;

    // new highest bidder
    current.winner = cps;
    current.bid = bid;
}

bool auctioning::won ()
{
    return current.is_valid() && current.winner == self && current.end < Time::now();
}
