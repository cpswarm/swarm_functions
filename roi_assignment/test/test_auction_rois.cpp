#include <gtest/gtest.h>
#include "lib/auction_rois.h"

/**
 * @brief Test the auction_rois setting and getting coordinates.
 */
TEST (UnitTestAuctionRois, testCoords)
{
    // test empty roi
    auction_rois rois;
    try {
        rois.get_coords("");
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "Unknown ROI ");
    }

    // add one roi
    vector<pair<double,vector<geometry_msgs::Point>>> vec;
    vector<geometry_msgs::Point> coords;
    geometry_msgs::Point coord;
    coords.push_back(coord);
    vec.emplace_back(0, coords);
    rois.init(vec);
    ASSERT_EQ(rois.get_coords("0.000000,0.000000 ").size(), 1);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 ")[0].x, 0);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 ")[0].y, 0);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 ")[0].z, 0);
    try {
        rois.get_coords("asdf");
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "Unknown ROI asdf");
    }

    // add another roi
    coord.x = 1.234;
    coord.y = 5.678;
    coords.push_back(coord);
    vec.emplace_back(9.876, coords);
    rois.init(vec);
    ASSERT_EQ(rois.get_coords("0.000000,0.000000 ").size(), 1);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 ")[0].x, 0);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 ")[0].y, 0);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 ")[0].z, 0);
    ASSERT_EQ(rois.get_coords("0.000000,0.000000 1.234000,5.678000 ").size(), 2);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 1.234000,5.678000 ")[0].x, 0);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 1.234000,5.678000 ")[0].y, 0);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 1.234000,5.678000 ")[0].z, 0);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 1.234000,5.678000 ")[1].x, 1.234);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 1.234000,5.678000 ")[1].y, 5.678);
    EXPECT_DOUBLE_EQ(rois.get_coords("0.000000,0.000000 1.234000,5.678000 ")[1].z, 0);
    try {
        rois.get_coords("jklö");
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "Unknown ROI jklö");
    }
}

/**
 * @brief Test the auction_rois cost calculation.
 */
TEST (UnitTestAuctionRois, testCost)
{
    // test empty roi
    auction_rois rois;
    try {
        rois.select();
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "No ROIs available");
    }
    try {
        rois.bid("");
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "Unknown ROI ");
    }

    // add one roi
    vector<pair<double,vector<geometry_msgs::Point>>> vec;
    vector<geometry_msgs::Point> coords;
    geometry_msgs::Point coord;
    coords.push_back(coord);
    vec.emplace_back(0, coords);
    rois.init(vec);
    auction_roi roi = rois.select();
    EXPECT_EQ(roi.get_id(), "0.000000,0.000000 ");
    EXPECT_DOUBLE_EQ(roi.get_cost(), 0);
    EXPECT_DOUBLE_EQ(rois.bid(roi.get_id()), numeric_limits<double>::max());

    // change roi distance
    vec.clear();
    coords.push_back(coord);
    vec.emplace_back(1.234, coords);
    rois.init(vec);
    roi = rois.select();
    EXPECT_EQ(roi.get_id(), "0.000000,0.000000 ");
    EXPECT_DOUBLE_EQ(roi.get_cost(), 1.234);
    EXPECT_DOUBLE_EQ(rois.bid(roi.get_id()), 1/1.234);

    // add a cps
    rois.add(roi.get_id(), "asdf");
    roi = rois.select();
    EXPECT_EQ(roi.get_id(), "0.000000,0.000000 ");
    EXPECT_DOUBLE_EQ(roi.get_cost(), 2.468);
    EXPECT_DOUBLE_EQ(rois.bid(roi.get_id()), 1/2.468);

    // add cps to invalid roi
    try {
        rois.add("asdf", "asdf");
        ADD_FAILURE();
    }
    catch (const exception& e) {
        EXPECT_STREQ(e.what(), "Unknown ROI asdf");
    }

    //
    // add another roi with higher cost
    //
    coord.x = 1.234;
    coord.y = 5.678;
    coords.push_back(coord);
    vec.emplace_back(4.567, coords);
    rois.init(vec);
    roi = rois.select(); // still the same roi selected
    EXPECT_EQ(roi.get_id(), "0.000000,0.000000 ");
    EXPECT_DOUBLE_EQ(roi.get_cost(), 1.234);
    EXPECT_DOUBLE_EQ(rois.bid(roi.get_id()), 1/1.234);

    // add a cps
    rois.add(roi.get_id(), "asdf");
    roi = rois.select(); // still the same roi selected
    EXPECT_EQ(roi.get_id(), "0.000000,0.000000 ");
    EXPECT_DOUBLE_EQ(roi.get_cost(), 2.468);
    EXPECT_DOUBLE_EQ(rois.bid(roi.get_id()), 1/2.468);

    // add another cps
    rois.add(roi.get_id(), "jklö");
    roi = rois.select(); // still the same roi selected
    EXPECT_EQ(roi.get_id(), "0.000000,0.000000 ");
    EXPECT_DOUBLE_EQ(roi.get_cost(), 3.702);
    EXPECT_DOUBLE_EQ(rois.bid(roi.get_id()), 1/3.702);

    // add the same cps
    rois.add(roi.get_id(), "jklö");
    roi = rois.select(); // still the same roi selected
    EXPECT_EQ(roi.get_id(), "0.000000,0.000000 ");
    EXPECT_DOUBLE_EQ(roi.get_cost(), 3.702);
    EXPECT_DOUBLE_EQ(rois.bid(roi.get_id()), 1/3.702);

    // add another cps
    rois.add(roi.get_id(), "ölkj");
    roi = rois.select(); // new roi selected
    EXPECT_EQ(roi.get_id(), "0.000000,0.000000 1.234000,5.678000 ");
    EXPECT_DOUBLE_EQ(roi.get_cost(), 4.567);
    EXPECT_DOUBLE_EQ(rois.bid(roi.get_id()), 1/4.567);

    // add same cps to new roi
    rois.add(roi.get_id(), "jklö");
    roi = rois.select(); // previous roi selected
    EXPECT_EQ(roi.get_id(), "0.000000,0.000000 ");
    EXPECT_DOUBLE_EQ(roi.get_cost(), 4.936);
    EXPECT_DOUBLE_EQ(rois.bid(roi.get_id()), 1/4.936);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
