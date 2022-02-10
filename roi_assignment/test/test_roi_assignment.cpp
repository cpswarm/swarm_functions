#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <cpswarm_msgs/RoiAssignmentAction.h>
#include <cpswarm_msgs/PointArrayEvent.h>
#include <cpswarm_msgs/Position.h>
#include <cpswarm_msgs/StateEvent.h>
#include <cpswarm_msgs/ArrayOfPositions.h>
#include <cpswarm_msgs/ArrayOfStates.h>

using namespace std;
using namespace ros;

/**
 * @brief Assignment result.
 */
map<string, set<pair<double,double>>> assignment;

/**
 * @brief Callback function to receive the assignment result.
 * @param msg An array of positions of the other CPSs in a coordinate system local to this ROS instance.
 */
void assignment_callback (const cpswarm_msgs::PointArrayEvent::ConstPtr& msg)
{
    ASSERT_EQ(msg->x.size(), msg->y.size());
    set<pair<double,double>> roi;
    for (int i=0; i<msg->x.size(); ++i)
        roi.emplace(msg->x[i], msg->y[i]);

    assignment[msg->swarmio.node] = roi;
}

/**
 * @brief Test the ROI assignment action server where each CPS is assigned to a different ROI.
 */
TEST (NodeTestRoiAssignment, testAssignment)
{
    NodeHandle nh;

    // clear any previous assignments
    assignment.clear();

    // subscribe to assignment result
    Subscriber assignment_subscriber = nh.subscribe("rois/assignment", 10, assignment_callback);

    // publish cpss positions
    Publisher position_publisher = nh.advertise<cpswarm_msgs::ArrayOfPositions>("swarm_position", 1, true); // latched
    cpswarm_msgs::ArrayOfPositions swarm_position;
    for (int i=0; i<3; ++i) {
        cpswarm_msgs::Position position;
        position.swarmio.node = "CPS" + to_string(i);
        position.pose.position.x = i;
        position.pose.position.y = i;
        swarm_position.positions.push_back(position);
    }
    position_publisher.publish(swarm_position);

    // publish cpss states
    Publisher state_publisher = nh.advertise<cpswarm_msgs::ArrayOfStates>("swarm_state", 1, true); // latched
    cpswarm_msgs::ArrayOfStates swarm_state;
    for (int i=0; i<3; ++i) {
        cpswarm_msgs::StateEvent state;
        state.swarmio.node = "CPS" + to_string(i);
        state.state = "valid";
        swarm_state.states.push_back(state);
    }
    state_publisher.publish(swarm_state);

    // create action client
    actionlib::SimpleActionClient<cpswarm_msgs::RoiAssignmentAction> assignment_client("rois/assign", true); // new thread
    ASSERT_TRUE(assignment_client.waitForServer(Duration(5.0))); // failure, if server does not respond within 5 seconds

    // call action server
    cpswarm_msgs::RoiAssignmentGoal goal;
    assignment_client.sendGoal(goal);
    ASSERT_TRUE(assignment_client.waitForResult(Duration(5.0))); // failure, if result is not ready within 5 seconds

    // get assignment result
    spinOnce();

    // test result
    EXPECT_EQ(assignment["CPS0"].size(), 4);
    EXPECT_EQ(assignment["CPS1"].size(), 4);
    EXPECT_EQ(assignment["CPS2"].size(), 4);
    EXPECT_EQ(assignment["CPS0"].count(make_pair(1,-3)), 1);
    EXPECT_EQ(assignment["CPS0"].count(make_pair(1,-1)), 1);
    EXPECT_EQ(assignment["CPS0"].count(make_pair(3,-3)), 1);
    EXPECT_EQ(assignment["CPS0"].count(make_pair(3,-1)), 1);
    EXPECT_EQ(assignment["CPS1"].count(make_pair(-4,2)), 1);
    EXPECT_EQ(assignment["CPS1"].count(make_pair(-4,6)), 1);
    EXPECT_EQ(assignment["CPS1"].count(make_pair(0,2)), 1);
    EXPECT_EQ(assignment["CPS1"].count(make_pair(0,6)), 1);
    EXPECT_EQ(assignment["CPS2"].count(make_pair(3,-1)), 1);
    EXPECT_EQ(assignment["CPS2"].count(make_pair(3,1)), 1);
    EXPECT_EQ(assignment["CPS2"].count(make_pair(5,-1)), 1);
    EXPECT_EQ(assignment["CPS2"].count(make_pair(5,1)), 1);
}

/**
 * @brief Test the ROI assignment action server including ROI division.
 */
TEST (NodeTestRoiAssignment, testAssignmentWithDivision)
{
    NodeHandle nh;

    // clear any previous assignments
    assignment.clear();

    // subscribe to assignment result
    Subscriber assignment_subscriber = nh.subscribe("rois/assignment", 10, assignment_callback);

    // publish cpss positions
    Publisher position_publisher = nh.advertise<cpswarm_msgs::ArrayOfPositions>("swarm_position", 1, true); // latched
    cpswarm_msgs::ArrayOfPositions swarm_position;
    for (int i=0; i<3; ++i) {
        cpswarm_msgs::Position position;
        position.swarmio.node = "CPS" + to_string(i);
        position.pose.position.x = 0; // this changed
        position.pose.position.y = i;
        swarm_position.positions.push_back(position);
    }
    position_publisher.publish(swarm_position);

    // publish cpss states
    Publisher state_publisher = nh.advertise<cpswarm_msgs::ArrayOfStates>("swarm_state", 1, true); // latched
    cpswarm_msgs::ArrayOfStates swarm_state;
    for (int i=0; i<3; ++i) {
        cpswarm_msgs::StateEvent state;
        state.swarmio.node = "CPS" + to_string(i);
        state.state = "valid";
        swarm_state.states.push_back(state);
    }
    state_publisher.publish(swarm_state);

    // create action client
    actionlib::SimpleActionClient<cpswarm_msgs::RoiAssignmentAction> assignment_client("rois/assign", true); // new thread
    ASSERT_TRUE(assignment_client.waitForServer(Duration(5.0))); // failure, if server does not respond within 5 seconds

    // call action server
    cpswarm_msgs::RoiAssignmentGoal goal;
    assignment_client.sendGoal(goal);
    ASSERT_TRUE(assignment_client.waitForResult(Duration(5.0))); // failure, if result is not ready within 5 seconds

    // get assignment result
    spinOnce();

    // test result
    EXPECT_EQ(assignment["CPS0"].size(), 4);
    EXPECT_EQ(assignment["CPS1"].size(), 4);
    EXPECT_EQ(assignment["CPS2"].size(), 4);
    EXPECT_EQ(assignment["CPS0"].count(make_pair(1,-3)), 1);
    EXPECT_EQ(assignment["CPS0"].count(make_pair(1,-1)), 1);
    EXPECT_EQ(assignment["CPS0"].count(make_pair(3,-3)), 1);
    EXPECT_EQ(assignment["CPS0"].count(make_pair(3,-1)), 1);
    EXPECT_EQ(assignment["CPS1"].count(make_pair(-4,2)), 1);
    EXPECT_EQ(assignment["CPS1"].count(make_pair(-4,5)), 1);
    EXPECT_EQ(assignment["CPS1"].count(make_pair(-3,2)), 1);
    EXPECT_EQ(assignment["CPS1"].count(make_pair(-3,5)), 1);
    EXPECT_EQ(assignment["CPS2"].count(make_pair(-2,2)), 1);
    EXPECT_EQ(assignment["CPS2"].count(make_pair(-2,5)), 1);
    EXPECT_EQ(assignment["CPS2"].count(make_pair(-1,2)), 1);
    EXPECT_EQ(assignment["CPS2"].count(make_pair(-1,5)), 1);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    init(argc, argv, "test_area");
    return RUN_ALL_TESTS();
}
