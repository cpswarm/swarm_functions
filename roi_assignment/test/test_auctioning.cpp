#include <gtest/gtest.h>
#include "lib/auctioning.h"

/**
 * @brief Test the auctioning when no auction is running.
 */
TEST (UnitTestAuctioning, testIdle)
{
    // create an auction
    auctioning auction("me");

    // test the getters
    EXPECT_EQ(auction.get_roi(), "");
    try {
        auction.get_running();
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "No auction running");
    }
    EXPECT_FALSE(auction.is_running());
    EXPECT_FALSE(auction.won());
}

/**
 * @brief Test the auctioning with no bidders.
 */
TEST (UnitTestAuctioning, testAuctioneerOnly)
{
    // start an auction
    auctioning auction("me");
    auction.initiate("a roi", 1.234, Duration(1));

    // try starting another auction
    try {
        auction.initiate("another roi", 1.234, Duration(1));
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "Already running an auction for ROI a roi");
    }

    // test the getters
    EXPECT_EQ(auction.get_roi(), "a roi");
    EXPECT_EQ(auction.get_running().roi, "a roi");
    EXPECT_TRUE(auction.is_running());
    EXPECT_FALSE(auction.won());

    // wait until auction ends
    Duration(1).sleep();

    // test the getters
    EXPECT_EQ(auction.get_roi(), "a roi");
    try {
        auction.get_running();
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "Auction for ROI a roi not running");
    }
    EXPECT_FALSE(auction.is_running());
    EXPECT_TRUE(auction.won());
}

/**
 * @brief Test the auctioning when being the auctioneer.
 */
TEST (UnitTestAuctioning, testAuctioneer)
{
    // start an auction
    auctioning auction("me");
    auction.initiate("a roi", 1.234, Duration(1));

    // test the getters
    EXPECT_EQ(auction.get_roi(), "a roi");
    EXPECT_EQ(auction.get_running().roi, "a roi");
    EXPECT_TRUE(auction.is_running());
    EXPECT_FALSE(auction.won());

    // add a lower bid
    auction.participant("a roi", "other", 0.123);

    // wait until auction ends
    Duration(1).sleep();

    // test the getters
    EXPECT_EQ(auction.get_roi(), "a roi");
    try {
        auction.get_running();
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "Auction for ROI a roi not running");
    }
    EXPECT_FALSE(auction.is_running());
    EXPECT_TRUE(auction.won());


    //
    // start another auction
    //
    auction.initiate("another roi", 1.234, Duration(1));

    // test the getters
    EXPECT_EQ(auction.get_roi(), "another roi");
    EXPECT_EQ(auction.get_running().roi, "another roi");
    EXPECT_TRUE(auction.is_running());
    EXPECT_FALSE(auction.won());

    // add some bids
    auction.participant("another roi", "other", 0.123);
    auction.participant("another roi", "another", 2.345);
    auction.participant("another roi", "other", 3.456);

    // wait until auction ends
    Duration(1).sleep();

    // test the getters
    EXPECT_EQ(auction.get_roi(), "");
    try {
        auction.get_running();
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "Auction for ROI another roi not running");
    }
    EXPECT_FALSE(auction.is_running());
    EXPECT_FALSE(auction.won());
}

/**
 * @brief Test the auctioning when participating in another auction.
 */
TEST (UnitTestAuctioning, testParticipation)
{
    // create an auction
    auctioning auction("me");

    // try setting result too early
    try {
        auction.set_result("a roi", "other", "me");
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "CPS other cannot announce result for ROI a roi because it was not auctioning");
    }

    // participate
    auction.participate("a roi", "other", 1.234);

    // try setting result for another roi
    try {
        auction.set_result("another roi", "other", "me");
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "CPS other cannot announce result for ROI another roi because it was assigning ROI a roi");
    }

    // set correct result
    auction.set_result("a roi", "other", "me");

    // test the getters
    EXPECT_EQ(auction.get_roi(), "a roi");
    try {
        auction.get_running();
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "No auction running");
    }
    EXPECT_FALSE(auction.is_running());
    EXPECT_TRUE(auction.won());

    // try changing result
    try {
        auction.set_result("a roi", "other", "other");
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "CPS other already announced result for ROI a roi to be me (!= other)");
    }
}

/**
 * @brief Test the auctioning when participating in multiple other auctions.
 */
TEST (UnitTestAuctioning, testParticipationMulti)
{
    // create an auction
    auctioning auction("me");

    // participate in some auctions
    auction.participate("a roi", "other", 1.234);
    auction.participate("another roi", "another", 2.345);
    auction.participate("a roi", "yet another", 3.456);
    auction.participate("a roi", "and yet another", 4.567);

    // test result
    EXPECT_EQ(auction.get_roi(), "");
    EXPECT_FALSE(auction.won());

    // set the result for first auction
    auction.set_result("a roi", "other", "yet another");

    // test result
    EXPECT_EQ(auction.get_roi(), "");
    EXPECT_FALSE(auction.won());

    // set the result for second auction
    auction.set_result("another roi", "another", "me");

    // test getting result
    EXPECT_EQ(auction.get_roi(), "another roi");
    EXPECT_TRUE(auction.won());

    // set the result for third auction
    auction.set_result("a roi", "yet another", "other");

    // test result
    EXPECT_EQ(auction.get_roi(), "another roi");
    EXPECT_TRUE(auction.won());

    // set the result for fourth auction
    auction.set_result("a roi", "and yet another", "me");

    // test result
    EXPECT_EQ(auction.get_roi(), "a roi");
    EXPECT_TRUE(auction.won());
}

/**
 * @brief Test the auctioning with simultaneous auctioneering and participating.
 */
TEST (UnitTestAuctioning, testSimultaneous)
{
    // create an auction
    auctioning auction("me");

    // participate in some auctions
    auction.participate("a roi", "other", 1.234);
    auction.participate("another roi", "another", 3.456);

    // start an auction
    auction.initiate("my roi", 2.345, Duration(1));

    // test result
    EXPECT_EQ(auction.get_roi(), "my roi");
    EXPECT_FALSE(auction.won());

    // set the result for first auction
    auction.set_result("a roi", "other", "me");

    // test result
    EXPECT_EQ(auction.get_roi(), "my roi");
    EXPECT_TRUE(auction.won());

    // have another participant in my auction
    auction.participant("my roi", "other", 3.456);

    // test result
    EXPECT_EQ(auction.get_roi(), "a roi");
    EXPECT_TRUE(auction.won());

    // wait for my auction to close
    Duration(1).sleep();

    // test result
    EXPECT_EQ(auction.get_roi(), "a roi");
    EXPECT_TRUE(auction.won());

    // set the result for second auction
    auction.set_result("another roi", "another", "me");

    // test result
    EXPECT_EQ(auction.get_roi(), "another roi");
    EXPECT_TRUE(auction.won());
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    Time::init();
    return RUN_ALL_TESTS();
}
