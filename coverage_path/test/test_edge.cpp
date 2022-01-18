#include <gtest/gtest.h>
#include "lib/edge.h"

/**
 * @brief Test the edge class members.
 */
TEST (UnitTestEdge, testMembers)
{
    // create class
    edge e1(0, 1, 2);
    // test if members can be retrieved correctly
    EXPECT_EQ(e1.vlow, 0);
    EXPECT_EQ(e1.vhigh, 1);
    EXPECT_EQ(e1.cost, 2);
    EXPECT_FALSE(e1.vertical);

    // create class
    edge e2(0, 1, 2, false);
    // test if members can be retrieved correctly
    EXPECT_EQ(e2.vlow, 0);
    EXPECT_EQ(e2.vhigh, 1);
    EXPECT_EQ(e2.cost, 2);
    EXPECT_FALSE(e2.vertical);

    // create class
    edge e3(0, 1, 2, true);
    // test if members can be retrieved correctly
    EXPECT_EQ(e3.vlow, 0);
    EXPECT_EQ(e3.vhigh, 1);
    EXPECT_EQ(e3.cost, 2);
    EXPECT_TRUE(e3.vertical);

    // test equality
    EXPECT_TRUE(e1 == e2);

    // test less than

    // test equal edges
    EXPECT_FALSE(e1 < e2);

    // test different costs
    e1.cost = 3;
    EXPECT_FALSE(e1 < e2);
    e2.cost = 4;
    EXPECT_TRUE(e1 < e2);

    // test different edge lengths
    e1.cost = 2;
    e2.cost = 2;
    e1.vhigh = 2;
    EXPECT_FALSE(e1 < e2);
    e2.vhigh = 3;
    EXPECT_TRUE(e1 < e2);

    // test different first index
    e2.vlow = 1;
    EXPECT_TRUE(e1 < e2);
}

/**
 * @brief Test the edge comparison.
 */
TEST (UnitTestEdge, testComparison)
{
    // comparison object
    compare_edge comp;

    // test equal edges
    edge e1(0, 1, 2);
    edge e2(0, 1, 2);
    EXPECT_FALSE(comp(e1, e2));

    // test different costs
    e1.cost = 3;
    EXPECT_TRUE(comp(e1, e2));
    e2.cost = 4;
    EXPECT_FALSE(comp(e1, e2));

    // test different edge lengths (horizontal)
    e1.cost = 2;
    e2.cost = 2;
    e1.vhigh = 2;
    EXPECT_TRUE(comp(e1, e2));
    e2.vhigh = 3;
    EXPECT_FALSE(comp(e1, e2));

    // test different edge lengths (vertical)
    e1.vertical = true;
    e1.vhigh = 2;
    e2.vhigh = 1;
    EXPECT_FALSE(comp(e1, e2));
    e2.vhigh = 3;
    EXPECT_TRUE(comp(e1, e2));

    // test different first index
    e2.vlow = 1;
    EXPECT_FALSE(comp(e1, e2));
}

/**
 * @brief Test the edge hasing.
 */
TEST (UnitTestEdge, testHashing)
{
    // edge hash object
    hash_edge e_hash;

    // string hash object
    hash<string> str_hash;

    // create class
    edge e1(0, 1, 2);

    // test if hashed correctly
    EXPECT_TRUE(e_hash(e1) == str_hash("012.000000"));
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
