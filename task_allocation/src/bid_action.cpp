#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <swarmros/String.h>
#include <cpswarm_msgs/TaskAllocationEvent.h>
#include <cpswarm_msgs/TaskAllocatedEvent.h>
#include <cpswarm_msgs/TaskAllocationAction.h>

using namespace std;
using namespace ros;

/**
 * @brief The server object that offers the task allocation auction as action server.
 */
typedef actionlib::SimpleActionServer<cpswarm_msgs::TaskAllocationAction> Server;

/**
 * @brief Publisher to submit a bid to the auction.
 */
Publisher publisher;

/**
 * @brief Position of this CPS.
 */
geometry_msgs::Pose pose;

/**
 * @brief The result of the task allocation auction.
 */
cpswarm_msgs::TaskAllocatedEvent allocation;

/**
 * @brief The UUID if this CPS.
 */
string uuid;

/**
 * @brief Whether a position of this CPS has been received.
 */
bool pose_received;

/**
 * @brief The ID of the task that is auctioned.
 */
int task_id;

/**
 * @brief Callback function to receive the position of this CPS.
 * @param msg The pose of this CPS.
 */
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = msg->pose;
    pose_received = true;
}

/**
 * @brief Callback function to receive the task allocation result.
 * @param msg The result of the task allocation auction.
 */
void allocation_callback(const cpswarm_msgs::TaskAllocatedEvent::ConstPtr& msg)
{
    if (msg->task_id == task_id) {
        allocation = *msg;
    }
}

/**
 * @brief Callback function to receive the UUID of this CPS.
 * @param msg The UUID of this CPS.
 */
void uuid_callback(const swarmros::String::ConstPtr& msg)
{
    uuid = msg->value;
}

/**
 * @brief Callback function to participate in a task allocation auction.
 * @param goal The bidding goal.
 * @param as Reference to the action server object.
 */
void bid_callback(const cpswarm_msgs::TaskAllocationGoal::ConstPtr& goal, Server* as)
{
    // compute bid
    double distance = hypot(goal->task_pose.pose.position.x - pose.position.x, goal->task_pose.pose.position.y - pose.position.y);
    task_id = goal->task_id;

    ROS_INFO("TASK_BID - Compute bid for task %d, value: %.2f", task_id, 1.0/distance);

    // create bid message
    cpswarm_msgs::TaskAllocationEvent task_allocation;
    task_allocation.header.stamp = Time::now();
    task_allocation.header.frame_id = "local_origin_ned";
    task_allocation.swarmio.name = "cps_selection";
    task_allocation.swarmio.node = goal->auctioneer;
    task_allocation.id = task_id;
    task_allocation.bid = 1.0 / distance;

    // publish bid until auction ends
    NodeHandle nh;
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    while (ok() && !as->isPreemptRequested() && allocation.task_id < 0) {
        ROS_DEBUG_ONCE("TASK_BID - Waiting for auction to end");
        publisher.publish(task_allocation);
        spinOnce();
        rate.sleep();
    }

    ROS_INFO("TASK_BID - Task allocated to: %s", allocation.cps_id.c_str());

    // action server has been preempted
    if (as->isPreemptRequested()) {
        as->setPreempted();
    }

    // this cps has won the auction, return result
    else if (allocation.cps_id.compare(uuid) == 0) {
        cpswarm_msgs::TaskAllocationResult result;
        result.task_id = allocation.task_id;
        result.task_pose = goal->task_pose;
        as->setSucceeded(result);
    }

    // auction lost, return negative result
    else {
        as->setAborted();
    }

    // reset global variables
    task_id = -1;
    allocation.task_id = -1;
}

/**
 * @brief A ROS node that exchanges relative kinematics between CPSs in a swarm.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main(int argc, char **argv)
{
    // init ros node
    init(argc, argv, "task_allocation_bid");
    NodeHandle nh;

    // init global variables
    uuid = "";
    pose_received = false;
    task_id = -1;
    allocation.task_id = -1;

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);

    // publishers and subscribers
    Subscriber pose_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("pos_provider/pose", queue_size, pose_callback);
    Subscriber allocation_subscriber = nh.subscribe<cpswarm_msgs::TaskAllocatedEvent>("bridge/events/cps_selected", queue_size, allocation_callback);
    Subscriber uuid_subscriber = nh.subscribe<swarmros::String>("bridge/uuid", 1, uuid_callback);
    publisher = nh.advertise<cpswarm_msgs::TaskAllocationEvent>("cps_selection", queue_size);

    // wait for pose and uuid
    while (ok() && (!pose_received || uuid.compare("") == 0)) {
        ROS_DEBUG_ONCE("TASK_BID - Waiting for position or UUID...");
        spinOnce();
        rate.sleep();
    }

    // start action server and wait
    Server server(nh, "cmd/task_allocation_bid", boost::bind(&bid_callback, _1, &server), false);
    server.start();
    ROS_INFO("TASK_BID - Task allocation bid action available");
    spin();
    return 0;
}
