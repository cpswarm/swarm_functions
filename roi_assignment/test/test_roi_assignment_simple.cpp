#include <gtest/gtest.h>
#include <ros/ros.h>
#include "lib/roi_assignment_simple.h"

/**
 * @brief Test the roi_assignment_simple library.
 */
TEST (UnitTestRoiAssignmentSimple, testLib)
{
    // create class
    roi_assignment_simple ass;

    // add some rois
    set<pair<double,double>> roi0;
    roi0.emplace(1.0,1.0);
    roi0.emplace(1.0,2.0);
    set<pair<double,double>> roi1;
    roi1.emplace(1.0,1.0);
    roi1.emplace(1.0,2.0);
    roi1.emplace(2.0,1.0);
    roi1.emplace(2.0,2.0);
    set<pair<double,double>> roi2;
    roi2.emplace(-1.0,1.0);
    roi2.emplace(-1.0,2.0);
    roi2.emplace(-2.0,1.0);
    roi2.emplace(-2.0,2.0);
    set<pair<double,double>> roi3;
    roi3.emplace(-1.0,-1.0);
    roi3.emplace(-1.0,-2.0);
    roi3.emplace(-2.0,-1.0);
    roi3.emplace(-2.0,-2.0);
    set<pair<double,double>> roi4;
    roi4.emplace(1.0,-1.0);
    roi4.emplace(1.0,-2.0);
    roi4.emplace(2.0,-1.0);
    roi4.emplace(2.0,-2.0);
    set<set<pair<double,double>>> rois;
    rois.insert(roi0);
    rois.insert(roi1);
    rois.insert(roi2);
    rois.insert(roi3);
    rois.insert(roi4);
    ass.set_rois(rois);

    // add some cpss
    pair<double,double> cps1(-0.5,-0.5);
    pair<double,double> cps2(0,0);
    pair<double,double> cps3(0.5,0.5);
    pair<double,double> cps4(1,0);
    pair<double,double> cps5(1,1);
    pair<double,double> cps6(1,1.5);
    pair<double,double> cps7(1.5,1.5);
    map<string, pair<double,double>> cpss;
    cpss["CPS1"] = cps1;
    cpss["CPS2"] = cps2;
    cpss["CPS3"] = cps3;
    cpss["CPS4"] = cps4;
    cpss["CPS5"] = cps5;
    cpss["CPS6"] = cps6;
    cpss["CPS7"] = cps7;
    ass.set_cpss(cpss);

    // perform assignment
    ROS_INFO("Assignment in progress...");
    float progress;
    do {
        progress = ass.assign();
        ROS_INFO("%.0f%%", progress * 100.0);
    }
    while (progress < 1.0);

    // test result
    set<set<pair<double,double>>> rois_used;
    rois_used.insert(roi1);
    rois_used.insert(roi3);
    rois_used.insert(roi4);
    assignment_result result = ass.get_result();
    set<set<pair<double,double>>> rois_result = result.get_rois();
    EXPECT_TRUE (rois_result == rois_used);
    set<set<pair<double,double>>>::iterator roi = rois.find(roi1);
    map<string, pair<double,double>> cpss_result = result.get_cpss(*roi);
    EXPECT_EQ(cpss_result.size(), 4);
    EXPECT_TRUE (cpss_result["CPS3"] == cps3);
    EXPECT_TRUE (cpss_result["CPS5"] == cps5);
    EXPECT_TRUE (cpss_result["CPS6"] == cps6);
    roi = rois.find(roi2);
    cpss_result = result.get_cpss(*roi);
    EXPECT_EQ(cpss_result.size(), 0);
    roi = rois.find(roi3);
    cpss_result = result.get_cpss(*roi);
    EXPECT_EQ(cpss_result.size(), 2);
    EXPECT_TRUE (cpss_result["CPS1"] == cps1);
    EXPECT_TRUE (cpss_result["CPS2"] == cps2);
    roi = rois.find(roi4);
    cpss_result = result.get_cpss(*roi);
    EXPECT_EQ(cpss_result.size(), 1);
    EXPECT_TRUE (cpss_result["CPS4"] == cps4);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
