#include <gtest/gtest.h>
#include "lib/edge.h"
#include "lib/spanning_tree.h"
#include "lib/mst_path.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

/**
 * @brief Round the elements of a given point to the nearest integer.
 * @param point The point to round.
 * @return The rounded point.
 */
geometry_msgs::Point round (geometry_msgs::Point point)
{
    point.x = round(point.x);
    point.y = round(point.y);
    return point;
}

/**
 * @brief Translate a point by a given vector.
 * @param point The point to translate.
 * @param vector The offset to translate the point by.
 * @return The translated point.
 */
geometry_msgs::Point translate (geometry_msgs::Point point, geometry_msgs::Vector3 vector)
{
    point.x += vector.x;
    point.y += vector.y;
    return point;
}

/**
 * @brief Rotate a point by a given angle around the origin.
 * @param point The point to rotate.
 * @param angle The angle in radian, counter-clockwise.
 * @return The rotated point.
 */
geometry_msgs::Point rotate (geometry_msgs::Point point, double angle)
{
    geometry_msgs::Point rotated;
    rotated.x = point.x * cos(angle) - point.y * sin(angle);
    rotated.y = point.x * sin(angle) + point.y * cos(angle);
    return rotated;
}

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

    // test waypoints
    geometry_msgs::Point pos = start;
    geometry_msgs::Point wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x += 0.25;
    pos.y += 0.25;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x -= 0.5;
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
    pos.x = start.x + 0.25;
    pos.y = start.y + 0.25;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_FALSE(path.valid());
}

/**
 * @brief Test waypoints of rotated path, reduced.
 */
TEST (UnitTestMstPath, testRotation)
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
    path.initialize_map(grid.info.origin.position, 0.1234, grid.info.width, grid.info.height);
    path.initialize_tree(tree.get_mst_edges());

    // generate path
    ros::Time::init();
    geometry_msgs::Point start;
    start.x = -10;
    start.y = -5;
    ASSERT_TRUE(path.generate_path(start));
    path.reduce();

    // test waypoints
    geometry_msgs::Vector3 origin;
    origin.x = grid.info.origin.position.x;
    origin.y = grid.info.origin.position.y;
    geometry_msgs::Vector3 origin_neg;
    origin_neg.x = -grid.info.origin.position.x;
    origin_neg.y = -grid.info.origin.position.y;
    // start
    geometry_msgs::Point pos = start;
    geometry_msgs::Point pos_rt = round(translate(rotate(start, 0.1234), origin_neg));
    pos_rt.x += 0.25;
    pos_rt.y += 0.25;
    pos = rotate(translate(pos_rt, origin), -0.1234);
    geometry_msgs::Point wp = path.get_waypoint(pos, 0);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    // go left
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos_rt.x -= 5.5;
    pos = rotate(translate(pos_rt, origin), -0.1234);
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    // meander down
    for (int i=0; i<3; ++i) {
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos_rt.y -= 0.5;
        pos = rotate(translate(pos_rt, origin), -0.1234);
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos_rt.x += 9;
        pos = rotate(translate(pos_rt, origin), -0.1234);
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos_rt.y -= 0.5;
        pos = rotate(translate(pos_rt, origin), -0.1234);
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos_rt.x -= 9;
        pos = rotate(translate(pos_rt, origin), -0.1234);
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
    }
    // go left
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos_rt.y -= 0.5;
    pos = rotate(translate(pos_rt, origin), -0.1234);
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos_rt.x += 9;
    pos = rotate(translate(pos_rt, origin), -0.1234);
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos_rt.y -= 0.5;
    pos = rotate(translate(pos_rt, origin), -0.1234);
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos_rt.x -= 9.5;
    pos = rotate(translate(pos_rt, origin), -0.1234);
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    // go up and right
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos_rt.y += 14.5;
    pos = rotate(translate(pos_rt, origin), -0.1234);
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos_rt.x += 9.5;
    pos = rotate(translate(pos_rt, origin), -0.1234);
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    // meander down
    for (int i=0; i<10; ++i) {
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos_rt.y -= 0.5;
        pos = rotate(translate(pos_rt, origin), -0.1234);
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos_rt.x -= 9;
        pos = rotate(translate(pos_rt, origin), -0.1234);
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos_rt.y -= 0.5;
        pos = rotate(translate(pos_rt, origin), -0.1234);
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos_rt.x += 9;
        pos = rotate(translate(pos_rt, origin), -0.1234);
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
    }
    // return to start
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos_rt.y -= 0.5;
    pos = rotate(translate(pos_rt, origin), -0.1234);
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos_rt.x -= 3.5;
    pos = rotate(translate(pos_rt, origin), -0.1234);
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
    nav_msgs::Path nav_path = path.get_path();

    // test if waypoints match path
    geometry_msgs::Point wp = start;
    wp.x += 0.25;
    wp.y += 0.25;
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
