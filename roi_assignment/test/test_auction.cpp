#include <gtest/gtest.h>
#include "lib/auction.h"

/**
 * @brief Test the auction constructors.
 */
TEST (UnitTestAuction, testConstruction)
{
    // test empty roi
    auction auction_empty;
    EXPECT_EQ(auction_empty.roi, "");
    EXPECT_DOUBLE_EQ(auction_empty.bid, 0);
    EXPECT_EQ(auction_empty.winner, "");
    EXPECT_DOUBLE_EQ(auction_empty.start.toSec(), 0);
    EXPECT_DOUBLE_EQ(auction_empty.end.toSec(), 0);

    // test with some initialization
    auction auction_some("asdf", 1.234);
    EXPECT_EQ(auction_some.roi, "asdf");
    EXPECT_DOUBLE_EQ(auction_some.bid, 1.234);
    EXPECT_EQ(auction_some.winner, "");
    EXPECT_DOUBLE_EQ(auction_some.start.toSec(), 0);
    EXPECT_DOUBLE_EQ(auction_some.end.toSec(), 0);

    // test with most members initialized
    auction auction_most("qwer", 0.123456789, Time(0.1234), Time(5.6789));
    EXPECT_EQ(auction_most.roi, "qwer");
    EXPECT_DOUBLE_EQ(auction_most.bid, 0.123456789);
    EXPECT_EQ(auction_most.winner, "");
    EXPECT_DOUBLE_EQ(auction_most.start.toSec(), 0.1234);
    EXPECT_DOUBLE_EQ(auction_most.end.toSec(), 5.6789);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
