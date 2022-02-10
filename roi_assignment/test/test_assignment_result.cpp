#include <gtest/gtest.h>
#include "lib/assignment_result.h"

/**
 * @brief Test the assignment_result get_cpss function.
 */
TEST (UnitTestAssignmentResult, testGetCpss)
{
    // create class
    assignment_result result;

    // create some rois
    set<pair<double,double>> roi1;
    roi1.emplace(1.0,2.0);
    roi1.emplace(2.0,3.0);
    roi1.emplace(3.0,4.0);
    set<pair<double,double>> roi2;
    roi2.emplace(4.0,5.0);
    roi2.emplace(5.0,6.0);
    roi2.emplace(6.0,7.0);
    set<pair<double,double>> roi3;
    roi3.emplace(7.0,8.0);
    roi3.emplace(8.0,9.0);
    roi3.emplace(9.0,0.0);

    // create some cpss
    pair<string, pair<double,double>> cps1(piecewise_construct, forward_as_tuple("CPS1"), forward_as_tuple(1.2,2.3));
    pair<string, pair<double,double>> cps2(piecewise_construct, forward_as_tuple("CPS2"), forward_as_tuple(3.4,4.5));
    pair<string, pair<double,double>> cps3(piecewise_construct, forward_as_tuple("CPS3"), forward_as_tuple(5.6,6.7));
    pair<string, pair<double,double>> cps4(piecewise_construct, forward_as_tuple("CPS4"), forward_as_tuple(7.8,8.9));
    pair<string, pair<double,double>> cps5(piecewise_construct, forward_as_tuple("CPS5"), forward_as_tuple(9.0,0.1));
    pair<string, pair<double,double>> cps6(piecewise_construct, forward_as_tuple("CPS6"), forward_as_tuple(1.2,2.3));

    // add the cpss to the rois
    result.add(roi1, cps1);
    result.add(roi2, cps2);
    result.add(roi2, cps3);
    result.add(roi3, cps4);
    result.add(roi3, cps5);
    result.add(roi3, cps6);

    // test if cpss can be retrieved correctly
    map<string, pair<double,double>> cpss = result.get_cpss (roi1);
    EXPECT_EQ(1, cpss.size());
    EXPECT_FLOAT_EQ(cpss[cps1.first].first, cps1.second.first);
    EXPECT_FLOAT_EQ(cpss[cps1.first].second, cps1.second.second);
    cpss = result.get_cpss (roi2);
    EXPECT_EQ(2, cpss.size());
    EXPECT_FLOAT_EQ(cpss[cps2.first].first, cps2.second.first);
    EXPECT_FLOAT_EQ(cpss[cps2.first].second, cps2.second.second);
    EXPECT_FLOAT_EQ(cpss[cps3.first].first, cps3.second.first);
    EXPECT_FLOAT_EQ(cpss[cps3.first].second, cps3.second.second);
    cpss = result.get_cpss (roi3);
    EXPECT_EQ(3, cpss.size());
    EXPECT_FLOAT_EQ(cpss[cps4.first].first, cps4.second.first);
    EXPECT_FLOAT_EQ(cpss[cps4.first].second, cps4.second.second);
    EXPECT_FLOAT_EQ(cpss[cps5.first].first, cps5.second.first);
    EXPECT_FLOAT_EQ(cpss[cps6.first].second, cps6.second.second);
}

/**
 * @brief Test the assignment_result get_rois function.
 */
TEST (UnitTestAssignmentResult, testGetRois)
{
    // create class
    assignment_result result;

    // create some rois
    set<pair<double,double>> roi1;
    roi1.emplace(1.0,2.0);
    roi1.emplace(2.0,3.0);
    roi1.emplace(3.0,4.0);
    set<pair<double,double>> roi2;
    roi2.emplace(4.0,5.0);
    roi2.emplace(5.0,6.0);
    roi2.emplace(6.0,7.0);
    set<pair<double,double>> roi3;
    roi3.emplace(7.0,8.0);
    roi3.emplace(8.0,9.0);
    roi3.emplace(9.0,0.0);

    // create some cpss
    pair<string, pair<double,double>> cps1(piecewise_construct, forward_as_tuple("CPS1"), forward_as_tuple(1.2,2.3));
    pair<string, pair<double,double>> cps2(piecewise_construct, forward_as_tuple("CPS2"), forward_as_tuple(3.4,4.5));
    pair<string, pair<double,double>> cps3(piecewise_construct, forward_as_tuple("CPS3"), forward_as_tuple(5.6,6.7));
    pair<string, pair<double,double>> cps4(piecewise_construct, forward_as_tuple("CPS4"), forward_as_tuple(7.8,8.9));
    pair<string, pair<double,double>> cps5(piecewise_construct, forward_as_tuple("CPS5"), forward_as_tuple(9.0,0.1));
    pair<string, pair<double,double>> cps6(piecewise_construct, forward_as_tuple("CPS6"), forward_as_tuple(1.2,2.3));

    // add the cpss to the rois
    result.add(roi1, cps1);
    result.add(roi2, cps2);
    result.add(roi2, cps3);
    result.add(roi3, cps4);
    result.add(roi3, cps5);
    result.add(roi3, cps6);

    // test if cpss can be retrieved correctly
    set<set<pair<double,double>>> rois = result.get_rois();
    EXPECT_EQ(3, rois.size());
    set<set<pair<double,double>>>::iterator roi = rois.find(roi1);
    EXPECT_EQ(roi->size(), 3);
    pair<double,double> c (1.0,2.0);
    EXPECT_EQ(roi->count(c), 1);
    c = make_pair(2.0,3.0);
    EXPECT_EQ(roi->count(c), 1);
    c = make_pair(3.0,4.0);
    EXPECT_EQ(roi->count(c), 1);
    c = make_pair(4.0,5.0);
    EXPECT_EQ(roi->count(c), 0);
    roi = rois.find(roi2);
    EXPECT_EQ(roi->count(c), 1);
    c = make_pair(5.0,6.0);
    EXPECT_EQ(roi->count(c), 1);
    c = make_pair(6.0,7.0);
    EXPECT_EQ(roi->count(c), 1);
    c = make_pair(7.0,8.0);
    EXPECT_EQ(roi->count(c), 0);
    roi = rois.find(roi3);
    EXPECT_EQ(roi->count(c), 1);
    c = make_pair(8.0,9.0);
    EXPECT_EQ(roi->count(c), 1);
    c = make_pair(9.0,0.0);
    EXPECT_EQ(roi->count(c), 1);
    c = make_pair(0.0,1.0);
    EXPECT_EQ(roi->count(c), 0);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
