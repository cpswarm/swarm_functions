#include <gtest/gtest.h>
#include "lib/edge.h"
#include "lib/spanning_tree.h"
#include <tf2/utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>

/**
 * @brief Test the minimum spanning tree construction.
 */
TEST (UnitTestSpanningTree, testConstruct)
{
    // create class
    spanning_tree tree;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.width = 10;
    grid.info.height = 15;
    for (int i=0; i<15; ++i) {
        for (int j=0; j<10; ++j) {
            grid.data.push_back(0);
        }
    }

    // initialize grid underlying tree
    tree.initialize_graph(grid);

    // generate tree
    tree.construct();

    // test if expected edges are there
    edge e(0, 0, 1);
    for (int i=0; i<15; ++i) {
        // horizontal edges
        for (int j=0; j<9; ++j) {
            e.vlow = i*10 + j;
            e.vhigh = i*10 + j + 1;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }

        // vertical edges
        if (i < 14) {
            e.vlow = i*10;
            e.vhigh = (i+1)*10;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }
    }
}

/**
 * @brief Test the minimum spanning tree get_tree member.
 */
TEST (UnitTestSpanningTree, testGetTree)
{
    // create class
    spanning_tree tree;

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

    // initialize grid underlying tree
    tree.initialize_graph(grid);

    // generate tree
    tree.construct();

    // get tree
    ros::Time::init();
    geometry_msgs::PoseArray pa = tree.get_tree();

    // test if expected edges are there
    geometry_msgs::Pose p;
    vector<geometry_msgs::Pose>::iterator pi;
    for (int i=0; i<15; ++i) {
        // horizontal edges
        for (int j=0; j<9; ++j) {
            p.position.x = j * grid.info.resolution + grid.info.origin.position.x + 0.5 * grid.info.resolution;
            p.position.y = i * grid.info.resolution + grid.info.origin.position.y + 0.5 * grid.info.resolution;
            tf2::Quaternion direction;
            direction.setRPY(0, 0, 0);
            p.orientation = tf2::toMsg(direction);
            pi = find (pa.poses.begin(), pa.poses.end(), p);
            EXPECT_TRUE(pi != pa.poses.end());
        }

        // vertical edges
        if (i < 14) {
            p.position.x = grid.info.origin.position.x + 0.5 * grid.info.resolution;
            p.position.y = i * grid.info.resolution + grid.info.origin.position.y + 0.5 * grid.info.resolution;
            tf2::Quaternion direction;
            direction.setRPY(0, 0, M_PI/2.0);
            p.orientation = tf2::toMsg(direction);
            pi = find (pa.poses.begin(), pa.poses.end(), p);
            EXPECT_TRUE(pi != pa.poses.end());
        }
    }
}

/**
 * @brief Test the minimum spanning tree construction when vertical edges are prioritized.
 */
TEST (UnitTestSpanningTree, testConstructVertical)
{
    // create class
    spanning_tree tree;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.width = 10;
    grid.info.height = 15;
    for (int i=0; i<15; ++i) {
        for (int j=0; j<10; ++j) {
            grid.data.push_back(0);
        }
    }

    // initialize grid underlying tree
    tree.initialize_graph(grid, true); // vertical

    // generate tree
    tree.construct();

    // test if expected edges are there
    edge e(0, 0, 1);
    for (int i=0; i<14; ++i) {
        // vertical edges
        for (int j=0; j<10; ++j) {
            e.vlow = i*10 + j;
            e.vhigh = (i+1)*10 + j;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }
    }
    // horizontal edges
    for (int j=0; j<9; ++j) {
        e.vlow = j;
        e.vhigh = j + 1;
        EXPECT_EQ(tree.get_mst_edges().count(e), 1);
    }
}

/**
 * @brief Test the minimum spanning tree construction with Moore neighborhood.
 */
TEST (UnitTestSpanningTree, testConstructMoore)
{
    // create class
    spanning_tree tree;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.width = 10;
    grid.info.height = 15;
    for (int i=0; i<15; ++i) {
        for (int j=0; j<10; ++j) {
            grid.data.push_back(0);
        }
    }

    // initialize grid underlying tree
    tree.initialize_graph(grid, false, false); // Moore neighborhood

    // generate tree
    tree.construct();

    // test if expected edges are there
    edge e(0, 0, 1);
    for (int i=0; i<15; ++i) {
        // horizontal edges
        for (int j=0; j<9; ++j) {
            e.vlow = i*10 + j;
            e.vhigh = i*10 + j + 1;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }

        // vertical edges
        if (i < 14) {
            e.vlow = i*10;
            e.vhigh = (i+1)*10;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }
    }
}

/**
 * @brief Test the minimum spanning tree construction with obstacles.
 */
TEST (UnitTestSpanningTree, testObstacles)
{
    // create class
    spanning_tree tree;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.width = 10;
    grid.info.height = 15;
    for (int i=0; i<15; ++i) {
        for (int j=0; j<10; ++j) {
            grid.data.push_back(0);
        }
    }

    // add obstacles
    grid.data[6*10 + 4] = 100;
    grid.data[6*10 + 5] = 100;
    grid.data[7*10 + 4] = 100;
    grid.data[7*10 + 5] = 100;

    // initialize grid underlying tree
    tree.initialize_graph(grid);

    // generate tree
    tree.construct();

    // test if expected edges are there
    edge e(0, 0, 1);
    for (int i=0; i<15; ++i) {
        // horizontal edges
        for (int j=0; j<9; ++j) {
            if ((j==3 || j==4 || j==5) && (i==6 || i==7))
                continue;
            e.vlow = i*10 + j;
            e.vhigh = i*10 + j + 1;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }

        // vertical edges
        if (i < 14) {
            e.vlow = i*10;
            e.vhigh = (i+1)*10;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }
    }
    // vertical edges
    e.vlow = 5*10 + 6;
    e.vhigh = 6*10 + 6;
    EXPECT_EQ(tree.get_mst_edges().count(e), 1);
    e.vlow = 6*10 + 6;
    e.vhigh = 7*10 + 6;
    EXPECT_EQ(tree.get_mst_edges().count(e), 1);
}

/**
 * @brief Test the minimum spanning tree construction with obstacles where once cell is not reachable with von Neumann neighborhood.
 */
TEST (UnitTestSpanningTree, testObstaclesNeumann)
{
    // create class
    spanning_tree tree;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.width = 10;
    grid.info.height = 15;
    for (int i=0; i<15; ++i) {
        for (int j=0; j<10; ++j) {
            grid.data.push_back(0);
        }
    }

    // add obstacles
    grid.data[6*10 + 4] = 100;
    grid.data[6*10 + 5] = 100;
    grid.data[7*10 + 4] = 100;
    grid.data[7*10 + 6] = 100;
    grid.data[8*10 + 4] = 100;
    grid.data[8*10 + 5] = 100;
    grid.data[8*10 + 6] = 100;

    // initialize grid underlying tree
    tree.initialize_graph(grid);

    // generate tree
    tree.construct();

    // test if expected edges are there
    edge e(0, 0, 1);
    for (int i=0; i<15; ++i) {
        // horizontal edges
        for (int j=0; j<9; ++j) {
            if ((j==3 || j==4 || j==5 || j==6) && (i==6 || i==7 || i==8))
                continue;
            e.vlow = i*10 + j;
            e.vhigh = i*10 + j + 1;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }
        if (i == 6) {
            e.vlow = i*10 + 6;
            e.vhigh = i*10 + 7;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }

        // vertical edges
        if (i < 14) {
            e.vlow = i*10;
            e.vhigh = (i+1)*10;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }
    }
    // vertical edges
    e.vlow = 5*10 + 6;
    e.vhigh = 6*10 + 6;
    EXPECT_EQ(tree.get_mst_edges().count(e), 1);
    e.vlow = 6*10 + 7;
    e.vhigh = 7*10 + 7;
    EXPECT_EQ(tree.get_mst_edges().count(e), 1);
    e.vlow = 7*10 + 7;
    e.vhigh = 8*10 + 7;
    EXPECT_EQ(tree.get_mst_edges().count(e), 1);
}

/**
 * @brief Test the minimum spanning tree construction with obstacles where once cell is reachable only with Moore neighborhood.
 */
TEST (UnitTestSpanningTree, testObstaclesMoore)
{
    // create class
    spanning_tree tree;

    // create an empty gridmap for testing
    nav_msgs::OccupancyGrid grid;
    grid.info.width = 10;
    grid.info.height = 15;
    for (int i=0; i<15; ++i) {
        for (int j=0; j<10; ++j) {
            grid.data.push_back(0);
        }
    }

    // add obstacles
    grid.data[6*10 + 4] = 100;
    grid.data[6*10 + 5] = 100;
    grid.data[7*10 + 4] = 100;
    grid.data[7*10 + 6] = 100;
    grid.data[8*10 + 4] = 100;
    grid.data[8*10 + 5] = 100;
    grid.data[8*10 + 6] = 100;

    // initialize grid underlying tree
    tree.initialize_graph(grid, false, false); // Moore neighborhood

    // generate tree
    tree.construct();

    // test if expected edges are there
    edge e(0, 0, 1);
    for (int i=0; i<15; ++i) {
        // horizontal edges
        for (int j=0; j<9; ++j) {
            if ((j==3 || j==4 || j==5 || j==6) && (i==6 || i==7 || i==8))
                continue;
            e.vlow = i*10 + j;
            e.vhigh = i*10 + j + 1;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }
        if (i == 6) {
            e.vlow = i*10 + 6;
            e.vhigh = i*10 + 7;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }

        // vertical edges
        if (i < 14) {
            e.vlow = i*10;
            e.vhigh = (i+1)*10;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }
    }
    // vertical edges
    e.vlow = 5*10 + 6;
    e.vhigh = 6*10 + 6;
    EXPECT_EQ(tree.get_mst_edges().count(e), 1);
    e.vlow = 6*10 + 7;
    e.vhigh = 7*10 + 7;
    EXPECT_EQ(tree.get_mst_edges().count(e), 1);
    e.vlow = 7*10 + 7;
    e.vhigh = 8*10 + 7;
    EXPECT_EQ(tree.get_mst_edges().count(e), 1);
    // diagonal edge
    e.vlow = 6*10 + 6;
    e.vhigh = 7*10 + 5;
    e.cost = sqrt(2);
    EXPECT_EQ(tree.get_mst_edges().count(e), 1);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
