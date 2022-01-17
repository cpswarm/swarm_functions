#include <gtest/gtest.h>
#include "lib/edge.h"
#include "lib/spanning_tree.h"
#include "nav_msgs/OccupancyGrid.h"

/**
 * @brief Test the minimum spanning tree construction.
 */
TEST (UnitTestSpanningTree, testConstruct)
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
            e.vlow = i*10 + 9;
            e.vhigh = (i+1)*10 + 9;
            EXPECT_EQ(tree.get_mst_edges().count(e), 1);
        }
    }
}

// TO TEST:
// - tree.get_tree();
// - grid with obstacles
// - vertical
// - 8-neighborhood

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
