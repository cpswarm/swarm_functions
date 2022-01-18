#include <gtest/gtest.h>
#include "lib/edge.h"
#include "lib/spanning_tree.h"
#include "lib/mst_path.h"
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

/**
 * @brief Test waypoints of path, reduced.
 */
TEST (UnitTestMstPath, testWaypoints)
{
    // create class
    mst_path path;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 1;
    grid.info.width = 10;
    grid.info.height = 15;
    grid.info.origin.position.x = -15;
    grid.info.origin.position.y = -10;
    for (int i=0; i<15; ++i) {
        for (int j=0; j<10; ++j) {
            grid.data.push_back(0);
        }
    }

    // create mst
    spanning_tree tree;
    tree.initialize_graph(grid);
    tree.construct();

    // initialize path
    path.initialize_graph(grid);
    path.initialize_map(grid.info.origin.position, 0, grid.info.width, grid.info.height);
    path.initialize_tree(tree.get_mst_edges());

    // generate path
    ros::Time::init();
    geometry_msgs::Point start;
    start.x = -14;
    start.y = -9;
    ASSERT_TRUE(path.generate_path(start));
    path.reduce();
    nav_msgs::Path nav_path = path.get_path();

    // test waypoints
    geometry_msgs::Point pos = start;
    geometry_msgs::Point wp = path.get_waypoint(pos, 0);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x -= 0.25; // not quite sure why signs don't match
    pos.y += 0.25;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.y -= 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x = -5.25;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.y -= 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x = -14.75;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.y = 4.75;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    // meander down
    for (int i=0; i<13; ++i) {
        pos.x = -5.25;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        pos.y -= 0.5;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        pos.x = -14.25;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        pos.y -= 0.5;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
    }
    pos.x = -5.25;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.y -= 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x = -13.25; // not sure why this exact value
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos = start;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_FALSE(path.valid());
}

/**
 * @brief Test equality of waypoints and path.
 */
TEST (UnitTestMstPath, testEquality)
{
    // create class
    mst_path path;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 0.1;
    grid.info.width = 10;
    grid.info.height = 15;
    grid.info.origin.position.x = -15;
    grid.info.origin.position.y = -10;
    for (int i=0; i<15; ++i) {
        for (int j=0; j<10; ++j) {
            grid.data.push_back(0);
        }
    }

    // create mst
    spanning_tree tree;
    tree.initialize_graph(grid);
    tree.construct();

    // initialize path
    path.initialize_graph(grid);
    path.initialize_map(grid.info.origin.position, 0, grid.info.width, grid.info.height);
    path.initialize_tree(tree.get_mst_edges());

    // generate path
    ros::Time::init();
    geometry_msgs::Point start;
    start.x = -14;
    start.y = -9;
    ASSERT_TRUE(path.generate_path(start));
    nav_msgs::Path nav_path = path.get_path();

    // test if waypoints match path
    geometry_msgs::Point wp = start;
    for (auto p : nav_path.poses) {
        // valid current waypoint
        ASSERT_TRUE(path.valid());

        // tests
        EXPECT_FLOAT_EQ(wp.x, p.pose.position.x);
        EXPECT_FLOAT_EQ(wp.y, p.pose.position.y);
        EXPECT_FLOAT_EQ(wp.z, p.pose.position.z);

        // get next waypoint
        wp = path.get_waypoint(p.pose.position, 0.00001);
    }

    // waypoint not valid anymore
    EXPECT_FALSE(path.valid());
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
