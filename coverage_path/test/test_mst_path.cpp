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
 * @brief Test get_waypoint function.
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
    path.initialize_map(grid, 0);
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
 * @brief Test rotation.
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
    path.initialize_map(grid, 0.1234);
    path.initialize_tree(tree.get_mst_edges());

    // generate path
    ros::Time::init();
    geometry_msgs::Point start;
    start.x = -10.5;
    start.y = -4.5;
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
    pos_rt.x -= 4.5;
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
    pos_rt.x -= 4.5;
    pos = rotate(translate(pos_rt, origin), -0.1234);
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_FALSE(path.valid());
}

/**
 * @brief Test obstacles
 */
TEST (UnitTestMstPath, testObstacles)
{
    // create class
    mst_path path;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 1;
    grid.info.width = 20;
    grid.info.height = 20;
    grid.info.origin.position.x = -10;
    grid.info.origin.position.y = -10;
    for (int i=0; i<20; ++i) {
        for (int j=0; j<20; ++j) {
            grid.data.push_back(0);
        }
    }

    // add obstacles
    grid.data[9*20 + 9] = 100;
    grid.data[9*20 + 9] = 100;
    grid.data[10*20 + 10] = 100;
    grid.data[10*20 + 10] = 100;

    // create mst
    spanning_tree tree;
    tree.initialize_graph(grid);
    tree.construct();

    // initialize path
    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());

    // generate path
    ros::Time::init();
    geometry_msgs::Point start;
    start.x = -10;
    start.y = -10;
    ASSERT_TRUE(path.generate_path(start));
    path.reduce();

    // test waypoints

    // start
    geometry_msgs::Point pos = start;
    geometry_msgs::Point wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x += 0.25;
    pos.y += 0.25;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    // go right
    pos.x += 19.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    // meander half-way up
    for (int i=0; i<8; ++i) {
        pos.y += 0.5;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        pos.x -= 19;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        pos.y += 0.5;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        pos.x += 19;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
    }

    // go around obstacle
    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x -= 9;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x += 9;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x -= 8;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x += 8;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x -= 8.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    pos.y -= 1;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x -= 1;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.y -= 1;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x -= 9.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x += 8;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x -= 8;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x += 9;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x -= 9;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x += 19;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    // meander all the way up
    for (int i=0; i<8; ++i) {
        pos.y += 0.5;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        pos.x -= 19;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        pos.y += 0.5;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        pos.x += 19;
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
    }

    // go left
    pos.y += 0.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    pos.x -= 19.5;
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    // go down
    pos.y -= 19.5;
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
    path.initialize_map(grid, 0);
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
 * @brief Test different starting points for horizontal path.
 */
TEST (UnitTestMstPath, testStartHorizontal)
{
    // create class
    mst_path path;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 1;
    grid.info.width = 20;
    grid.info.height = 20;
    grid.info.origin.position.x = -10;
    grid.info.origin.position.y = -10;
    for (int i=0; i<20; ++i) {
        for (int j=0; j<20; ++j) {
            grid.data.push_back(0);
        }
    }

    // create mst
    spanning_tree tree;
    tree.initialize_graph(grid);
    tree.construct();

    // other initialization
    ros::Time::init();
    geometry_msgs::Point start;
    vector<geometry_msgs::PoseStamped> nav_path;

    // test center
    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 0; start.y = 0;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    // test inside
    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 5; start.y = 0;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -5; start.y = 0;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 0; start.y = -5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 0; start.y = 5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 5; start.y = 5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -5; start.y = 5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 5; start.y = -5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -5; start.y = -5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    // test sides
    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -10; start.y = 0;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 10; start.y = 0;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 0; start.y = -10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 0; start.y = 10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    // test corners
    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -10; start.y = -10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 10; start.y = 10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 10; start.y = -10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -10; start.y = 10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);
}

/**
 * @brief Test different starting points for vertical path.
 */
TEST (UnitTestMstPath, testStartVertical)
{
    // create class
    mst_path path;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 1;
    grid.info.width = 20;
    grid.info.height = 20;
    grid.info.origin.position.x = -10;
    grid.info.origin.position.y = -10;
    for (int i=0; i<20; ++i) {
        for (int j=0; j<20; ++j) {
            grid.data.push_back(0);
        }
    }

    // create mst
    spanning_tree tree;
    tree.initialize_graph(grid, true);
    tree.construct();

    // other initialization
    ros::Time::init();
    geometry_msgs::Point start;
    vector<geometry_msgs::PoseStamped> nav_path;

    // test center
    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 0; start.y = 0;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    // test inside
    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 5; start.y = 0;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -5; start.y = 0;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 0; start.y = -5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 0; start.y = 5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 5; start.y = 5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -5; start.y = 5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 5; start.y = -5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -5; start.y = -5;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    // test sides
    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -10; start.y = 0;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 10; start.y = 0;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 0; start.y = -10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 0; start.y = 10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    // test corners
    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -10; start.y = -10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 10; start.y = 10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = 10; start.y = -10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);

    path.initialize_map(grid, 0, true);
    path.initialize_tree(tree.get_mst_edges());
    start.x = -10; start.y = 10;
    EXPECT_TRUE(path.generate_path(start));
    nav_path = path.get_path().poses;
    EXPECT_EQ(nav_path.size(), 1601);
}

/**
 * @brief Test high resolution.
 */
TEST (UnitTestMstPath, testHighRes)
{
    // create class
    mst_path path;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 0.1;
    grid.info.width = 100;
    grid.info.height = 50;
    grid.info.origin.position.x = 0;
    grid.info.origin.position.y = 0;
    for (int i=0; i<50; ++i) {
        for (int j=0; j<100; ++j) {
            grid.data.push_back(0);
        }
    }

    // create mst
    spanning_tree tree;
    tree.initialize_graph(grid);
    tree.construct();

    // initialize path
    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());

    // generate path
    ros::Time::init();
    geometry_msgs::Point start;
    start.x = 0;
    start.y = 5;
    ASSERT_TRUE(path.generate_path(start));
    path.reduce();

    // test waypoints

    // start point
    geometry_msgs::Point pos = start;
    geometry_msgs::Point wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x += 0.025;
    pos.y -= 0.025;
    EXPECT_NEAR(wp.x, pos.x, 0.00001);
    EXPECT_NEAR(wp.y, pos.y, 0.00001);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    // go down and right
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.y -= 4.95;
    EXPECT_NEAR(wp.x, pos.x, 0.00001);
    EXPECT_NEAR(wp.y, pos.y, 0.00001);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x += 9.95;
    EXPECT_NEAR(wp.x, pos.x, 0.00001);
    EXPECT_NEAR(wp.y, pos.y, 0.00001);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    // meander up
    for (int i=0; i<49; ++i) {
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.y += 0.05;
        EXPECT_NEAR(wp.x, pos.x, 0.00001);
        EXPECT_NEAR(wp.y, pos.y, 0.00001);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.x -= 9.9;
        EXPECT_NEAR(wp.x, pos.x, 0.00001);
        EXPECT_NEAR(wp.y, pos.y, 0.00001);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.y += 0.05;
        EXPECT_NEAR(wp.x, pos.x, 0.00001);
        EXPECT_NEAR(wp.y, pos.y, 0.00001);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.x += 9.9;
        EXPECT_NEAR(wp.x, pos.x, 0.00001);
        EXPECT_NEAR(wp.y, pos.y, 0.00001);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
    }

    // return to start
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.y += 0.05;
    EXPECT_NEAR(wp.x, pos.x, 0.00001);
    EXPECT_NEAR(wp.y, pos.y, 0.00001);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x -= 9.95;
    EXPECT_NEAR(wp.x, pos.x, 0.00001);
    EXPECT_NEAR(wp.y, pos.y, 0.00001);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_FALSE(path.valid());
}

/**
 * @brief Test low resolution.
 */
TEST (UnitTestMstPath, testLowRes)
{
    // create class
    mst_path path;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 10;
    grid.info.width = 10;
    grid.info.height = 5;
    grid.info.origin.position.x = 0;
    grid.info.origin.position.y = 0;
    for (int i=0; i<5; ++i) {
        for (int j=0; j<10; ++j) {
            grid.data.push_back(0);
        }
    }

    // create mst
    spanning_tree tree;
    tree.initialize_graph(grid);
    tree.construct();

    // initialize path
    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());

    // generate path
    ros::Time::init();
    geometry_msgs::Point start;
    start.x = 100;
    start.y = 50;
    ASSERT_TRUE(path.generate_path(start));
    path.reduce();

    // test waypoints

    // start point
    geometry_msgs::Point pos = start;
    geometry_msgs::Point wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x -= 2.5;
    pos.y -= 2.5;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    // go left, down, right
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x -= 95;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.y -= 45;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x += 95;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    // meander up
    for (int i=0; i<4; ++i) {
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.y += 5;
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.x -= 90;
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.y += 5;
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.x += 90;
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
    }

    // return to start
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.y += 5;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_FALSE(path.valid());
}

/**
 * @brief Test weird resolution.
 */
TEST (UnitTestMstPath, testWeirdRes)
{
    // create class
    mst_path path;

    // create an gridmap for testing (10x5 downsampled from 1 to 1.5)
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 1.5;
    grid.info.width = 7;
    grid.info.height = 4;
    grid.info.origin.position.x = 0;
    grid.info.origin.position.y = 0;
    for (int i=0; i<4; ++i) {
        for (int j=0; j<7; ++j) {
            if (i < 3)
                grid.data.push_back(0);
            else
                grid.data.push_back(100);
        }
    }

    // create mst
    spanning_tree tree;
    tree.initialize_graph(grid);
    tree.construct();

    // initialize path
    path.initialize_map(grid, 0);
    path.initialize_tree(tree.get_mst_edges());

    // generate path
    ros::Time::init();
    geometry_msgs::Point start;
    start.x = 10.5;
    start.y = 6;
    ASSERT_TRUE(path.generate_path(start));
    path.reduce();

    // test waypoints

    // start point
    geometry_msgs::Point pos = start;
    geometry_msgs::Point wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x = 10.125;
    pos.y = 4.125;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    // go left, down, right
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x -= 9.75;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.y -= 3.75;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.x += 9.75;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);

    // meander up
    for (int i=0; i<2; ++i) {
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.y += 0.75;
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.x -= 9;
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.y += 0.75;
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
        wp = path.get_waypoint(pos, 0.00001);
        ASSERT_TRUE(path.valid());
        pos.x += 9;
        EXPECT_FLOAT_EQ(wp.x, pos.x);
        EXPECT_FLOAT_EQ(wp.y, pos.y);
        EXPECT_FLOAT_EQ(wp.z, pos.z);
    }

    // return to start
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_TRUE(path.valid());
    pos.y += 0.75;
    EXPECT_FLOAT_EQ(wp.x, pos.x);
    EXPECT_FLOAT_EQ(wp.y, pos.y);
    EXPECT_FLOAT_EQ(wp.z, pos.z);
    wp = path.get_waypoint(pos, 0.00001);
    ASSERT_FALSE(path.valid());
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
