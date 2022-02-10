#include <gtest/gtest.h>
#include "lib/auction_roi.h"

/**
 * @brief Test the auction_roi constructors.
 */
TEST (UnitTestAuctionRoi, testConstruction)
{
    // test empty roi
    auction_roi roi_empty;
    EXPECT_EQ(roi_empty.get_coords().size(), 0);
    EXPECT_DOUBLE_EQ(roi_empty.get_cost(), 0);
    EXPECT_EQ(roi_empty.get_id(), "");

    // test roi with simple data
    vector<geometry_msgs::Point> coords;
    geometry_msgs::Point coord;
    coords.push_back(coord);
    coord.x = 1;
    coords.push_back(coord);
    coord.y = 1;
    coords.push_back(coord);
    coord.x = 0;
    coords.push_back(coord);
    auction_roi roi_simple(1, coords);
    EXPECT_EQ(roi_simple.get_coords().size(), 4);
    EXPECT_DOUBLE_EQ(roi_simple.get_cost(), 1);
    EXPECT_EQ(roi_simple.get_id(), "0.000000,0.000000 0.000000,1.000000 1.000000,0.000000 1.000000,1.000000 ");
}

/**
 * @brief Test the auction_roi add function.
 */
TEST (UnitTestAuctionRoi, testAdd)
{
    // create roi with simple data
    vector<geometry_msgs::Point> coords;
    geometry_msgs::Point coord;
    coords.push_back(coord);
    coord.x = 1;
    coords.push_back(coord);
    coord.y = 1;
    coords.push_back(coord);
    coord.x = 0;
    coords.push_back(coord);
    auction_roi roi_simple(1, coords);
    EXPECT_EQ(roi_simple.get_coords().size(), 4);
    EXPECT_DOUBLE_EQ(roi_simple.get_cost(), 1);
    EXPECT_EQ(roi_simple.get_id(), "0.000000,0.000000 0.000000,1.000000 1.000000,0.000000 1.000000,1.000000 ");

    // add a few cpss
    for (int i=0; i<10; ++i) {
        roi_simple.add(to_string(i));
        EXPECT_DOUBLE_EQ(roi_simple.get_cost(), i+2);
    }

    // cpss with same id should be ignored
    roi_simple.add(to_string(5));
    EXPECT_DOUBLE_EQ(roi_simple.get_cost(), 11);
}

/**
 * @brief Test the auction_roi cost calculation.
 */
TEST (UnitTestAuctionRoi, testCost)
{
    // create roi with simple data
    vector<geometry_msgs::Point> coords;
    auction_roi roi_1(1, coords);
    EXPECT_EQ(roi_1.get_coords().size(), 0);
    EXPECT_DOUBLE_EQ(roi_1.get_cost(), 1);
    EXPECT_EQ(roi_1.get_id(), "");

    // add a few cpss
    for (int i=0; i<10; ++i) {
        roi_1.add(to_string(i));
        EXPECT_DOUBLE_EQ(roi_1.get_cost(), i+2);
    }

    // cpss with same id should be ignored
    roi_1.add(to_string(5));
    EXPECT_DOUBLE_EQ(roi_1.get_cost(), 11);

    // create another roi
    auction_roi roi_2(2, coords);
    EXPECT_DOUBLE_EQ(roi_2.get_cost(), 2);

    // add a few cpss
    for (int i=0; i<10; ++i) {
        roi_2.add(to_string(i));
        EXPECT_DOUBLE_EQ(roi_2.get_cost(), 2*(i+2));
    }

    // cpss with same id should be ignored
    roi_2.add(to_string(5));
    EXPECT_DOUBLE_EQ(roi_2.get_cost(), 22);

    // create another roi
    auction_roi roi_3(1.5, coords);
    EXPECT_DOUBLE_EQ(roi_3.get_cost(), 1.5);

    // add a few cpss
    for (int i=0; i<10; ++i) {
        roi_3.add(to_string(i));
        EXPECT_DOUBLE_EQ(roi_3.get_cost(), 1.5*(i+2));
    }

    // cpss with same id should be ignored
    roi_3.add(to_string(5));
    EXPECT_DOUBLE_EQ(roi_3.get_cost(), 16.5);
}

/**
 * @brief Test the auction_roi ID generation.
 */
TEST (UnitTestAuctionRoi, testId)
{
    // test roi with one coordinate (rounding)
    vector<geometry_msgs::Point> coords;
    geometry_msgs::Point coord;
    coord.x = 1.23456789;
    coords.push_back(coord);
    auction_roi roi_1(1, coords);
    EXPECT_EQ(roi_1.get_coords().size(), 1);
    EXPECT_DOUBLE_EQ(roi_1.get_cost(), 1);
    EXPECT_EQ(roi_1.get_id(), "1.234568,0.000000 ");

    // test roi with two coordinates
    coord.x = 9.87654321;
    coords.push_back(coord);
    auction_roi roi_2(1, coords);
    EXPECT_EQ(roi_2.get_coords().size(), 2);
    EXPECT_DOUBLE_EQ(roi_2.get_cost(), 1);
    EXPECT_EQ(roi_2.get_id(), "1.234568,0.000000 9.876543,0.000000 ");

    // test roi with three coordinates (rounding and sorting)
    coord.x = 0;
    coord.y = 999.9999999;
    coords.push_back(coord);
    auction_roi roi_3(1, coords);
    EXPECT_EQ(roi_3.get_coords().size(), 3);
    EXPECT_DOUBLE_EQ(roi_3.get_cost(), 1);
    EXPECT_EQ(roi_3.get_id(), "0.000000,1000.000000 1.234568,0.000000 9.876543,0.000000 ");

    // test roi with four coordinates (sorting)
    coord.x = 1.234568;
    coord.y = -1;
    coords.push_back(coord);
    auction_roi roi_4(1, coords);
    EXPECT_EQ(roi_4.get_coords().size(), 4);
    EXPECT_DOUBLE_EQ(roi_4.get_cost(), 1);
    EXPECT_EQ(roi_4.get_id(), "0.000000,1000.000000 1.234568,0.000000 1.234568,-1.000000 9.876543,0.000000 ");

    // test roi with five coordinates (sorting)
    coord.x = 1.23456789;
    coord.y = 1;
    coords.push_back(coord);
    auction_roi roi_5(1, coords);
    EXPECT_EQ(roi_5.get_coords().size(), 5);
    EXPECT_DOUBLE_EQ(roi_5.get_cost(), 1);
    EXPECT_EQ(roi_5.get_id(), "0.000000,1000.000000 1.234568,0.000000 1.234568,1.000000 1.234568,-1.000000 9.876543,0.000000 ");

    // test roi with six coordinates (almost duplicates)
    coord.x = 1.23456789;
    coord.y = -1;
    coords.push_back(coord);
    auction_roi roi_6(1, coords);
    EXPECT_EQ(roi_6.get_coords().size(), 6);
    EXPECT_DOUBLE_EQ(roi_6.get_cost(), 1);
    EXPECT_EQ(roi_6.get_id(), "0.000000,1000.000000 1.234568,-1.000000 1.234568,0.000000 1.234568,1.000000 1.234568,-1.000000 9.876543,0.000000 ");

    // test roi with seven coordinates (duplicates)
    coord.x = 1.23456789;
    coord.y = -1;
    coords.push_back(coord);
    auction_roi roi_7(1, coords);
    EXPECT_EQ(roi_7.get_coords().size(), 7); // this increased because it's a vector
    EXPECT_DOUBLE_EQ(roi_7.get_cost(), 1);
    EXPECT_EQ(roi_7.get_id(), "0.000000,1000.000000 1.234568,-1.000000 1.234568,0.000000 1.234568,1.000000 1.234568,-1.000000 9.876543,0.000000 ");
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
