#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/NavSatFix.h>
#include <swarmros/String.h>
#include <cpswarm_msgs/TargetAssignmentEvent.h>
#include <cpswarm_msgs/TargetAssignedEvent.h>
#include <compute_cost/ComputeCostAction.h>

using namespace std;

typedef actionlib::SimpleActionServer<compute_cost::ComputeCostAction> Server;

//Variables
ros::Publisher cost_pub;

ros::Subscriber global_pos_sub;
ros::Subscriber selection_sub;
ros::Subscriber UUID_sub;

sensor_msgs::NavSatFix gps_position;
cpswarm_msgs::TargetAssignedEvent target_assigned;
string UUID; //UUID of the CPS
bool gps_position_received; //TRUE when gps position has been received
bool cps_selected; //TRUE when cps_selected answer has been received
int target_id;

//**********************************************************************************************************
/**
 * Return distance in meter of 2 GPS points
 */
double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
	double earth_radius = 6378137.0;
	double pi = 3.141592653589793238462;

	double rlat1 = lat1 * (pi / 180.0);
	double rlat2 = lat2 * (pi / 180.0);
	double dlon = (lon2 - lon1) * (pi / 180.0);
	double dlat = (lat2 - lat1) * (pi / 180.0);
	double a = pow(sin(dlat / 2.0), 2) + cos(rlat1) * cos(rlat2) * pow(sin(dlon / 2.0), 2);

	double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
	return earth_radius * c;
}

//**************** CALLBACKS *******************************************************************************

void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	gps_position = *msg;
	gps_position_received = true;
}

void cpsSelected_cb(const cpswarm_msgs::TargetAssignedEvent::ConstPtr& msg) {
	if (msg->target_id == target_id) {
		target_assigned = *msg;
		cps_selected = true;
		ROS_INFO("CMP_COST - Target ASSIGNED event callback");
	}
}

void UUID_cb(const swarmros::String::ConstPtr& msg) {
	UUID = msg->value;
	ROS_INFO("CMP_COST - Received node UUID: %s", UUID.c_str());
}

void execute_cb(const compute_cost::ComputeCostGoal::ConstPtr& goal, Server* as) {
	ROS_INFO("CMP_COST - Executing ComputeCost action..");
	//Compute distance from target
	double distance = haversine_distance(goal->pose.latitude, goal->pose.longitude, gps_position.latitude, gps_position.longitude);
	target_id = goal->target_id;
	cps_selected = false;

	ROS_INFO("CMP_COST - Computed cost for target %d, value: %.5f", target_id, distance);

	cpswarm_msgs::TargetAssignmentEvent target_assignement;
	target_assignement.header.stamp = ros::Time::now();
	target_assignement.header.frame_id = "";
	target_assignement.swarmio.name = "cps_selection";
	target_assignement.swarmio.node = goal->sender_UUID;
	target_assignement.id = target_id;
	target_assignement.cost = distance;

	ros::Rate rate(1.0);
	//Wait for answer
	while (ros::ok() && !as->isPreemptRequested() && !cps_selected) {
		ROS_INFO("CMP_COST - Waiting for cps_selected event");
		cost_pub.publish(target_assignement);
		ros::spinOnce();
		rate.sleep();
	}

	bool selected = (target_assigned.cps_id.compare(UUID) == 0);
	ROS_INFO("CMP_COST - TargetAssignedEvent received: %d", selected);

	//Reset variables
	cps_selected = false;
	target_id = -1;

	if (as->isPreemptRequested()) {
		as->setPreempted();
	} else if (selected) {
		compute_cost::ComputeCostResult result;
		result.target_id = target_assigned.target_id;
		result.pose = goal->pose;
		as->setSucceeded(result);
	} else {
		as->setAborted();
	}
}

//*******************************************************************************************************************

int main(int argc, char **argv) {
	ros::init(argc, argv, "compute_cost");
	ros::NodeHandle nh;

	string target_cost_topic;
	nh.getParam(ros::this_node::getName() + "/target_cost_topic", target_cost_topic);
	string gps_topic;
	nh.getParam(ros::this_node::getName() + "/gps_topic", gps_topic);
	string cps_selected_topic;
	nh.getParam(ros::this_node::getName() + "/cps_selected_topic", cps_selected_topic);
	string UUID_topic;
	nh.getParam(ros::this_node::getName() + "/UUID_topic", UUID_topic);

	//Init variables
	UUID = "";
	gps_position_received = false;
	cps_selected = false;
	target_id = -1;

	//Initialize subscribers
	global_pos_sub = nh.subscribe < sensor_msgs::NavSatFix > (gps_topic, 10, globalPosition_cb);
	selection_sub = nh.subscribe < cpswarm_msgs::TargetAssignedEvent > (cps_selected_topic, 10, cpsSelected_cb);
	UUID_sub = nh.subscribe < swarmros::String > (UUID_topic, 1, UUID_cb);
	//Initialize publishers
	cost_pub = nh.advertise < cpswarm_msgs::TargetAssignmentEvent > (target_cost_topic, 10);

	ros::Rate rate(10.0);
	// wait for gps position and UUID
	while (ros::ok() && (!gps_position_received || UUID.compare("") == 0)) {
		ROS_DEBUG_ONCE("CMP_COST - Waiting for gps position OR UUID..");
		ros::spinOnce();
		rate.sleep();
	}

	Server server(nh, "cmd/compute_cost", boost::bind(&execute_cb, _1, &server), false);
	server.start();
	ROS_INFO("CMP_COST - ComputeCost action available");
	ros::spin();
	return 0;
}
