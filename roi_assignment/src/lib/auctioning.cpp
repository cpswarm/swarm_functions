#include "lib/auctioning.h"

auctioning::auctioning (string self) : self(self)
{
}

auction auctioning::get_result ()
{
    // no auction was running
    if (auctions.count(self) < 1)
        throw runtime_error("No auction was running");

    // auction still running
    if (Time::now() < auctions[self].end)
        throw runtime_error("Auction for ROI " + auctions[self].roi + " still running (" + to_string(Time::now().toSec()) + "<" + to_string(auctions[self].end.toSec()) + ")");

    return auctions[self];
}

string auctioning::get_roi ()
{
    auction highest;

    for (auto a : auctions) {
        if (a.second.winner == self && a.second.bid > highest.bid) {
            highest = a.second;
        }
    }

    return highest.roi;
}

auction auctioning::get_running ()
{
    // no auction running
    if (auctions.count(self) < 1)
        throw runtime_error("No auction running");

    // auction not running
    if (is_running() == false)
        throw runtime_error("Auction for ROI " + auctions[self].roi + " not running");

    return auctions[self];
}

void auctioning::initiate (string roi, double bid, Duration timeout)
{
    // already running an auction
    if (is_running())
        throw runtime_error("Already running an auction for ROI " + auctions[self].roi);

    // create auction
    auction auction(roi, self, bid, Time::now(), Time::now()+timeout);
    auction.winner = self; // and make this cps the winner by default

    auctions[self] = auction;
}

bool auctioning::is_running ()
{
    return auctions.count(self) > 0 && auctions[self].start <= Time::now() && Time::now() <= auctions[self].end;
}

void auctioning::participant (string roi, string cps, double bid)
{
    // ignore bids if not auctioning
    if (auctions.count(self) < 1)
        return;

    // ignore bids for other rois
    if (roi != auctions[self].roi)
        return;

    // auction closed
    if (is_running() == false)
        throw runtime_error("Received from " + cps + " bid for ROI " + roi + ", but auction already closed (" + to_string(Time::now().toSec()) + "<" + to_string(auctions[self].end.toSec()) + ")");

    // ignore lower bids
    if(bid <= auctions[self].bid)
        return;

    // new highest bidder
    auctions[self].winner = cps;
    auctions[self].bid = bid;
}

void auctioning::participate (string roi, string auctioneer, double bid)
{
    // store auction
    auction auction(roi, auctioneer, bid);
    auctions[auctioneer] = auction;
}

void auctioning::set_result (string roi, string auctioneer, string winner)
{
    // auctioneer was not active
    if (auctions.count(auctioneer) < 1)
        throw runtime_error("CPS " + auctioneer + " cannot announce result for ROI " + roi + " because it was not auctioning");

    // result roi does not match opening roi
    if (auctions[auctioneer].roi != roi)
        throw runtime_error("CPS " + auctioneer + " cannot announce result for ROI " + roi + " because it was assigning ROI " + auctions[auctioneer].roi);

    // result already received
    if (auctions[auctioneer].winner.compare("") != 0 && auctions[auctioneer].winner != winner)
        throw runtime_error("CPS " + auctioneer + " already announced result for ROI " + roi + " to be " + auctions[auctioneer].winner + " (!= " + winner + ")");

    // store winner
    auctions[auctioneer].winner = winner;
}

bool auctioning::won ()
{
    for (auto a : auctions) {
        if (a.second.winner == self && a.second.end < Time::now()) {
            return true;
        }
    }

    return false;
}
