#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
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
 * @brief Publisher to announce the winner of the auction.
 */
Publisher publisher;

/**
 * @brief ID of the task that is auctioned.
 */
int task_id;

/**
 * @brief Location of the task that is auctioned.
 */
geometry_msgs::Pose task_pose;

/**
 * @brief Whether the auction is currently open for bidding.
 */
bool auction_open;

/**
 * @brief The auction duration.
 */
double timeout;

/**
 * @brief The highest current bid.
 */
double highest_bid;

/**
 * @brief The UUID of the CPS that is the current highest bidder.
 */
string winner;

/**
 * @brief Callback function to receive auction bids.
 * @param msg The bid received.
 */
void bid_callback(const cpswarm_msgs::TaskAllocationEvent::ConstPtr& msg)
{
    if (auction_open && (task_id == msg->id)) {
        ROS_DEBUG("TASK_AUCTION - New bid received from %s", msg->swarmio.node.c_str());
        // new highest bid received
        if (msg->bid > highest_bid) {
            winner = msg->swarmio.node;
            highest_bid = msg->bid;
            ROS_INFO("TASK_AUCTION - Highest bid %.4f received from %s", msg->bid, msg->swarmio.node.c_str());
        }
    }
}

/**
 * @brief Callback function to start a task allocation auction.
 * @param goal The action goal.
 * @param as Reference to the action server object.
 */
void auction_callback(const cpswarm_msgs::TaskAllocationGoal::ConstPtr& goal, Server* as)
{
    // start a new auction
    task_id = goal->task_id;
    task_pose = goal->task_pose.pose;
    highest_bid = 0;
    winner = "";
    auction_open = true;
    ROS_INFO("TASK_AUCTION - Starting task allocation auction for task %d at position (%.6f, %.6f)", task_id, task_pose.position.x, task_pose.position.y);

    // wait for bids
    NodeHandle nh;
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    Time start_time = Time::now();
    while (ok() && !as->isPreemptRequested() && Time::now() - start_time < Duration(timeout)) {
        spinOnce();
        rate.sleep();
    }

    // close auction
    auction_open = false;
    if (winner.compare("") != 0) {
        cpswarm_msgs::TaskAllocatedEvent allocation;
        allocation.header.stamp = Time::now();
        allocation.header.frame_id = "local_origin_ned";
        allocation.swarmio.name = "cps_selected";
        allocation.task_id = task_id;
        allocation.cps_id = winner;
        publisher.publish(allocation);
        ROS_INFO("TASK_AUCTION - Task %d allocated to %s", task_id, winner.c_str());
    }

    // action server has been preempted
    if (as->isPreemptRequested()) {
        as->setPreempted();
    }

    else {
        // set action result
        cpswarm_msgs::TaskAllocationResult result;
        result.winner = winner;
        result.task_id = task_id;
        result.task_pose.pose = task_pose;

        // task allocation failed, return negative result
        if (winner.compare("") == 0) {
            as->setAborted(result);
        }

        // task allocation succeeded
        else {
            as->setSucceeded(result);
        }
    }
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
    init(argc, argv, "task_allocation_auction");
    NodeHandle nh;

    // init global variables
    auction_open = false;
    task_id = -1;
    winner = "";

    // read parameters
    nh.param(this_node::getName() + "/timeout", timeout, 10.0);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);

    // publishers and subscribers
    Subscriber cost_subscriber = nh.subscribe<cpswarm_msgs::TaskAllocationEvent>("bridge/events/cps_selection", queue_size, bid_callback);
    publisher = nh.advertise<cpswarm_msgs::TaskAllocatedEvent>("cps_selected", queue_size, true);

    // start the action server and wait
    Server server(nh, "cmd/task_allocation_auction", boost::bind(&auction_callback, _1, &server), false);
    server.start();
    ROS_INFO("TASK_AUCTION - Task allocation auction action available");
    spin();
    return 0;
}
