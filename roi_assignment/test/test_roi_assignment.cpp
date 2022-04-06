#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <swarmros/String.h>
#include <cpswarm_msgs/RoiAssignmentAction.h>
#include <cpswarm_msgs/TaskAllocationEvent.h>
#include <cpswarm_msgs/TaskAllocatedEvent.h>

using namespace std;
using namespace ros;

/**
 * @brief Auction received from ROS topic.
 */
cpswarm_msgs::TaskAllocationEvent auction;

/**
 * @brief Auction result received from ROS topic.
 */
vector<cpswarm_msgs::TaskAllocatedEvent> result;

/**
 * @brief Callback function to receive auctions.
 * @param msg The auction consisting of ROI ID and bid.
 */
void auction_cb (const cpswarm_msgs::TaskAllocationEvent::ConstPtr& msg)
{
    cerr << "AUCTION: " << msg->swarmio.name << "," << msg->bid << "," << msg->id << endl;
    auction = *msg;
}

/**
 * @brief Callback function to receive auction results.
 * @param msg The assigned CPS together with some meta data.
 */
void result_cb (const cpswarm_msgs::TaskAllocatedEvent::ConstPtr& msg)
{
    cerr << "RESULT: " << msg->swarmio.name << "," << msg->cps_id << "," << msg->task_id << endl;
    result.push_back(*msg);
}

/**
 * @brief Test the ROI assignment action server with an auctioneer only.
 */
TEST (NodeTestRoiAssignment, testAssignmentAuctioneer)
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

    // listen to auction opening
    auction = cpswarm_msgs::TaskAllocationEvent();
    Subscriber auction_subscriber = nh.subscribe("rois/assignment/auction", 1, auction_cb);

    // listen to auction result
    result.clear();
    Subscriber result_subscriber = nh.subscribe("rois/assignment/result", 1, result_cb);

    // create action client
    actionlib::SimpleActionClient<cpswarm_msgs::RoiAssignmentAction> assignment_client("rois/assign", true); // new thread
    ASSERT_TRUE(assignment_client.waitForServer(Duration(5.0))); // failure, if server does not respond within 5 seconds

    // call action server
    cpswarm_msgs::RoiAssignmentGoal goal;
    assignment_client.sendGoal(goal);
    ASSERT_TRUE(assignment_client.waitForResult(Duration(5.0))); // failure, if result is not ready within 5 seconds

    // check if auction was received
    spinOnce();
    EXPECT_EQ(auction.swarmio.name, "roi_assignment_auction");
    EXPECT_EQ(auction.id, "-4.000000,2.000000 -4.000000,6.000000 0.000000,2.000000 0.000000,6.000000 ");
    EXPECT_DOUBLE_EQ(auction.bid, 1/sqrt(2));

    // check if result was received
    ASSERT_EQ(result.size(), 1);
    EXPECT_EQ(result[0].swarmio.name, "roi_assignment_result");
    EXPECT_EQ(result[0].task_id, "-4.000000,2.000000 -4.000000,6.000000 0.000000,2.000000 0.000000,6.000000 ");
    EXPECT_EQ(result[0].cps_id, "me");

    // test action result
    ASSERT_EQ(assignment_client.getResult()->roi.size(), 4);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[0].x, -4);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[0].y, 2);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[0].z, 0);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[1].x, -4);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[1].y, 6);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[1].z, 0);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[2].x, 0);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[2].y, 2);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[2].z, 0);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[3].x, 0);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[3].y, 6);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[3].z, 0);
}

/**
 * @brief Test the ROI assignment action server with one auctioneer and some bidders.
 */
TEST (NodeTestRoiAssignment, testAssignmentBidder)
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

    // publish bids
    Publisher bid_publisher = nh.advertise<cpswarm_msgs::TaskAllocationEvent>("rois/assignment/bid", 3);

    // listen to auction opening
    auction = cpswarm_msgs::TaskAllocationEvent();
    Subscriber auction_subscriber = nh.subscribe("rois/assignment/auction", 1, auction_cb);

    // listen to auction result
    result.clear();
    Subscriber result_subscriber = nh.subscribe("rois/assignment/result", 4, result_cb);

    // create action client
    actionlib::SimpleActionClient<cpswarm_msgs::RoiAssignmentAction> assignment_client("rois/assign", true); // new thread
    ASSERT_TRUE(assignment_client.waitForServer(Duration(5.0))); // failure, if server does not respond within 5 seconds

    // call action server
    cpswarm_msgs::RoiAssignmentGoal goal;
    assignment_client.sendGoal(goal);

    // place higher bids
    Rate rate(5);
    cpswarm_msgs::TaskAllocationEvent bid;
    bid.header.stamp = Time::now();
    bid.swarmio.name = "roi_assignment_bid";
    // first auction
    while (auction.id != "-4.000000,2.000000 -4.000000,6.000000 0.000000,2.000000 0.000000,6.000000 ") {
        rate.sleep();
        spinOnce();
        if (bid.header.stamp + Duration(5) > Time::now())
            break;
    }
    bid.swarmio.node = "other";
    bid.id = "-4.000000,2.000000 -4.000000,6.000000 0.000000,2.000000 0.000000,6.000000 ";
    bid.bid = 1.234;
    bid_publisher.publish(bid);
    // second auction
    while (auction.id != "1.000000,-3.000000 1.000000,-1.000000 3.000000,-3.000000 3.000000,-1.000000 ") {
        rate.sleep();
        spinOnce();
        if (bid.header.stamp + Duration(10) > Time::now())
            break;
    }
    bid.swarmio.node = "another";
    bid.id = "1.000000,-3.000000 1.000000,-1.000000 3.000000,-3.000000 3.000000,-1.000000 ";
    bid.bid = 0.567;
    bid_publisher.publish(bid);
    // third auction
    while (auction.id != "3.000000,-1.000000 3.000000,1.000000 5.000000,-1.000000 5.000000,1.000000 ") {
        rate.sleep();
        spinOnce();
        if (bid.header.stamp + Duration(15) > Time::now())
            break;
    }
    bid.swarmio.node = "yet another";
    bid.id = "3.000000,-1.000000 3.000000,1.000000 5.000000,-1.000000 5.000000,1.000000 ";
    bid.bid = 0.678;
    bid_publisher.publish(bid);

    // wait for result
    ASSERT_TRUE(assignment_client.waitForResult(Duration(5.0))); // failure, if result is not ready within 5 seconds

    // test auction results
    spinOnce();
    ASSERT_EQ(result.size(), 4);
    EXPECT_EQ(result[0].swarmio.name, "roi_assignment_result");
    EXPECT_EQ(result[0].task_id, "-4.000000,2.000000 -4.000000,6.000000 0.000000,2.000000 0.000000,6.000000 ");
    EXPECT_EQ(result[0].cps_id, "other");
    EXPECT_EQ(result[0].swarmio.name, "roi_assignment_result");
    EXPECT_EQ(result[0].task_id, "1.000000,-3.000000 1.000000,-1.000000 3.000000,-3.000000 3.000000,-1.000000 ");
    EXPECT_EQ(result[0].cps_id, "another");
    EXPECT_EQ(result[0].swarmio.name, "roi_assignment_result");
    EXPECT_EQ(result[0].task_id, "3.000000,-1.000000 3.000000,1.000000 5.000000,-1.000000 5.000000,1.000000 ");
    EXPECT_EQ(result[0].cps_id, "yet another");
    EXPECT_EQ(result[1].swarmio.name, "roi_assignment_result");
    EXPECT_EQ(result[1].task_id, "-4.000000,2.000000 -4.000000,6.000000 0.000000,2.000000 0.000000,6.000000 ");
    EXPECT_EQ(result[1].cps_id, "me");

    // test action result
    ASSERT_EQ(assignment_client.getResult()->roi.size(), 4);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[0].x, -4);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[0].y, 2);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[0].z, 0);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[1].x, -4);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[1].y, 6);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[1].z, 0);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[2].x, 0);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[2].y, 2);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[2].z, 0);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[3].x, 0);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[3].y, 6);
    EXPECT_DOUBLE_EQ(assignment_client.getResult()->roi[3].z, 0);
}

/**
 * @brief Test the ROI assignment action server with multiple auctioneers.
 */
// TEST (NodeTestRoiAssignment, testAssignmentMultiAuctioneers)
// {
//     NodeHandle nh;
//     Time::init();

//     // publish my position
//     Publisher position_publisher = nh.advertise<geometry_msgs::PoseStamped>("position_provider/pose", 1, true); // latched
//     geometry_msgs::PoseStamped my_position;
//     my_position.header.stamp = Time::now();
//     my_position.pose.position.x = 1;
//     my_position.pose.position.y = 1;
//     position_publisher.publish(my_position);

//     // publish my uuid
//     Publisher uuid_publisher = nh.advertise<swarmros::String>("bridge/uuid", 1, true); // latched
//     swarmros::String my_uuid;
//     my_uuid.value = "me";
//     uuid_publisher.publish(my_uuid);
// }

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    init(argc, argv, "test_roi_assignment");
    return RUN_ALL_TESTS();
}
