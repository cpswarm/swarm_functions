#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cpswarm_msgs/TargetAssignmentEvent.h>
#include <cpswarm_msgs/TargetAssignedEvent.h>
#include <assign_task/AssignTaskAction.h>
#include <geometry_msgs/Pose.h>

using namespace std;

typedef actionlib::SimpleActionServer<assign_task::AssignTaskAction> Server;

//Variables
ros::Publisher selection_pub; //to send message for selected cps

ros::Subscriber cost_sub; // to receive costs from target from other CPSs

int target_id; // ID of the target
geometry_msgs::Pose target_position; // position of the target
bool listen_for_cost;
float timeout;
double current_best_cost;
string selected_cps_UUID;

//**************** CALLBACKS *******************************************************************************
void costReceived_cb(const cpswarm_msgs::TargetAssignmentEvent::ConstPtr& msg) {
	if (listen_for_cost && (target_id == msg->id)) {
		ROS_INFO("ASS_TASK - NEW cost received from %s", msg->swarmio.node.c_str());
		if (msg->cost < current_best_cost) {
			selected_cps_UUID = msg->swarmio.node;
			current_best_cost = msg->cost;
			ROS_INFO("ASS_TASK - BEST cost!! %.4f", msg->cost);
		}
	}
}

void execute_cb(const assign_task::AssignTaskGoal::ConstPtr& goal, Server* as) {
	ROS_INFO("ASS_TASK - Executing AssignTask action..");
	target_id = goal->target_id;
	target_position = goal->target_pose;
	ROS_INFO("ASS_TASK - Target %d Position - x: %.6f, y: %.6f", target_id, target_position.position.x, target_position.position.y);
	current_best_cost = numeric_limits<double>::max();
	selected_cps_UUID = "";
	listen_for_cost = true;

	ros::Rate rate(20.0);
	ros::Time start_time = ros::Time::now();

	//Wait for answer
	while (ros::ok() && !as->isPreemptRequested() && ros::Time::now() - start_time < ros::Duration(timeout)) {
		ros::spinOnce();
		rate.sleep();
	}

	listen_for_cost = false;
	if (selected_cps_UUID.compare("") != 0) {
		cpswarm_msgs::TargetAssignedEvent target_assigned;
		target_assigned.header.stamp = ros::Time::now();
		target_assigned.header.frame_id = "";
		target_assigned.swarmio.name = "cps_selected";
		//target_assigned.swarmio.node = "";
		target_assigned.target_id = target_id;
		target_assigned.cps_id = selected_cps_UUID;
		selection_pub.publish(target_assigned);
		ROS_INFO("ASS_TASK - Target %d assigned to: %s", target_id, selected_cps_UUID.c_str());
	}

	if (as->isPreemptRequested()) {
		as->setPreempted();
	} else if (selected_cps_UUID.compare("") == 0) {
		assign_task::AssignTaskResult result;
		result.target_id = target_id;
		result.selected_cps_UUID = selected_cps_UUID;
		result.target_pose = target_position;
		as->setAborted(result);
	} else {
		assign_task::AssignTaskResult result;
		result.target_id = target_id;
		result.selected_cps_UUID = selected_cps_UUID;
		result.target_pose = target_position;
		as->setSucceeded(result);
	}

}

//*******************************************************************************************************************

int main(int argc, char **argv) {
	ros::init(argc, argv, "assig_task");
	ros::NodeHandle nh;

	listen_for_cost = false;
	target_id = -1;
	selected_cps_UUID = "";

	nh.getParam(ros::this_node::getName() + "/timeout", timeout);
	string cost_topic;
	nh.getParam(ros::this_node::getName() + "/cost_topic", cost_topic);
	string cps_selected_topic;
	nh.getParam(ros::this_node::getName() + "/cps_selected_topic", cps_selected_topic);

	//Initialize subscribers
	cost_sub = nh.subscribe < cpswarm_msgs::TargetAssignmentEvent > (cost_topic, 10, costReceived_cb);
	//Initialize publishers
	selection_pub = nh.advertise < cpswarm_msgs::TargetAssignedEvent > (cps_selected_topic, 1);

	Server server(nh, "cmd/assign_task", boost::bind(&execute_cb, _1, &server), false);
	server.start();
	ROS_INFO("ASS_TASK - AssignTask action available");
	ros::spin();
	return 0;
}
