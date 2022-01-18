#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <cpswarm_msgs/GetWaypoint.h>
#include <cpswarm_msgs/PathGenerationAction.h>

using namespace std;
using namespace ros;

/**
 * @brief Test the coverage path action server.
 */
TEST (NodeTestCoveragePath, testActionServer)
{
    // create action client
    actionlib::SimpleActionClient<cpswarm_msgs::PathGenerationAction> path_client("coverage_path/generate", true); // new thread
    ASSERT_TRUE(path_client.waitForServer(Duration(5.0))); // failure, if server does not respond within 5 seconds

    // call action server
    cpswarm_msgs::PathGenerationGoal goal;
    goal.start.x = -7;
    goal.start.y = 1;
    path_client.sendGoal(goal);
    ASSERT_TRUE(path_client.waitForResult(Duration(5.0))); // failure, if result is not ready within 5 seconds

    // test result
    vector<geometry_msgs::PoseStamped> path = path_client.getResult()->path.poses;
    // TODO
    EXPECT_EQ(path.size(), 25);
}

/**
 * @brief Test the coverage path services.
 */
TEST (NodeTestCoveragePath, testServices)
{
    NodeHandle nh;

    // create path service client
    ServiceClient get_path_client = nh.serviceClient<nav_msgs::GetPlan>("coverage_path/path");
    ASSERT_TRUE(get_path_client.waitForExistence(Duration(5.0)));

    // call service
    nav_msgs::GetPlan gp;
    gp.request.start.pose.position.x = 0; // TODO
    gp.request.start.pose.position.y = 0;
    ASSERT_TRUE(get_path_client.call(gp));

    // test result
    vector<geometry_msgs::PoseStamped> path = gp.response.plan.poses;
    // TODO
    EXPECT_EQ(path.size(), 25);

    // create waypoint service client
    ServiceClient get_wp_client = nh.serviceClient<cpswarm_msgs::GetWaypoint>("coverage_path/waypoint");
    ASSERT_TRUE(get_wp_client.waitForExistence(Duration(5.0)));

    // call service
    cpswarm_msgs::GetWaypoint gwp;

    // test all waypoints
    gwp.request.position.x = 0; // TODO
    gwp.request.position.y = 0;
    gwp.request.tolerance = 0.1;
    ASSERT_TRUE(get_wp_client.call(gwp));
    EXPECT_FLOAT_EQ(gwp.response.point.x, 0.1);
    EXPECT_TRUE(gwp.response.valid);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    init(argc, argv, "test_area");
    return RUN_ALL_TESTS();
}
