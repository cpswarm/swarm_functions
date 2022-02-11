#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <swarmros/String.h>
#include <cpswarm_msgs/RoiAssignmentAction.h>

using namespace std;
using namespace ros;

/**
 * @brief Test the ROI assignment action server where each CPS is assigned to a different ROI.
 */
TEST (NodeTestRoiAssignment, testAssignment)
{
    NodeHandle nh;
    Time::init();

    // publish my position
    Publisher position_publisher = nh.advertise<geometry_msgs::PoseStamped>("position_provider/pose", 1, true); // latched
    geometry_msgs::PoseStamped my_position;
    my_position.header.stamp = Time::now();
    my_position.pose.position.x = 1;
    my_position.pose.position.y = 1;
    position_publisher.publish(my_position);

    // publish my uuid
    Publisher uuid_publisher = nh.advertise<swarmros::String>("bridge/uuid", 1, true); // latched
    swarmros::String my_uuid;
    my_uuid.value = "me";
    uuid_publisher.publish(my_uuid);

    // create action client
    actionlib::SimpleActionClient<cpswarm_msgs::RoiAssignmentAction> assignment_client("rois/assign", true); // new thread
    ASSERT_TRUE(assignment_client.waitForServer(Duration(5.0))); // failure, if server does not respond within 5 seconds

    // call action server
    cpswarm_msgs::RoiAssignmentGoal goal;
    assignment_client.sendGoal(goal);
    ASSERT_TRUE(assignment_client.waitForResult(Duration(5.0))); // failure, if result is not ready within 5 seconds

    // test result TODO
    FAIL();
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    init(argc, argv, "test_roi_assignment");
    return RUN_ALL_TESTS();
}
