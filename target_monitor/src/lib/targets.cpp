#include "lib/targets.h"

targets::targets ()
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    nh.param(this_node::getName() + "/fov", fov, 0.5);

    // uuid of this cps
    cps = "";

    // read done targets from parameter server
    vector<int> done;
    nh.getParam(this_node::getNamespace() + "/targets_done", done);
    for (int t : done) {
        target_map.emplace(piecewise_construct, forward_as_tuple(t), forward_as_tuple(make_shared<target>(t, TARGET_DONE)));
    }

    // publishers and subscribers
    tracking_pub = nh.advertise<cpswarm_msgs::TargetTracking>("target_tracking", queue_size, true);
    uuid_sub = nh.subscribe("bridge/uuid", queue_size, &targets::uuid_callback, this);
    // subscriber of the ar marker topic
    ar_marker_sub = nh.subscribe("ar_pose_marker", queue_size, &targets::ar_marker_callback, this);

    // init uuid
    while (ok() && cps == "") {
        rate.sleep();
        spinOnce();
    }
}

void targets::simulate ()
{/*
    // read all potential targets from parameter file
    vector<double> targets_x;
    vector<double> targets_y;
    nh.getParam(this_node::getName() + "/targets_x", targets_x);
    nh.getParam(this_node::getName() + "/targets_y", targets_y);
    if (targets_x.size() != targets_y.size()) {
        ROS_FATAL("Invalid targets specified! Exiting...");
        shutdown();
    }
    else if (targets_x.size() < 1)
        ROS_INFO("There are no targets!");
    for (int i = 0; i < targets_x.size(); ++i) {
        ROS_DEBUG("Target %d at [%.2f, %.2f]", i, targets_x[i], targets_y[i]);
        geometry_msgs::Pose new_target_pose;
        new_target_pose.position.x = targets_x[i];
        new_target_pose.position.y = targets_y[i];
        new_target_pose.orientation.w = 1;
        simulated_targets.emplace(piecewise_construct, forward_as_tuple(i), forward_as_tuple(make_shared<target>(i, TARGET_UNKNOWN, new_target_pose)));
    }
    * */
}

void targets::update (geometry_msgs::Pose pose)
{
	/*
    // check if a target is lost
    for (auto t : target_map) {
        // update target and inform others in case target is lost
        t.second->lost();
    }
    */
	bool checked_before = false;
    // check if a new target is found in simulation
    for (int i=0; i<markers.size(); i++) {
        
        
        /*
		if (id_saved_markers.size() != 0){
			for (int j=0; j<id_saved_markers.size(); j++){
					if (id_saved_markers[j] == markers[i].id){
						checked_before = true;
						break;
						
					}
				
			}
			if (!checked_before){
				id_saved_markers.push_back(markers[i].id);*/
				cpswarm_msgs::TargetTracking track;
				track.header.stamp = Time::now();
				track.id = markers[i].id;
				track.tf = transform(markers[i].pose.pose, pose);
				tracking_pub.publish(track);
				
			} 
        
      /*} else{
		  
		  
				id_saved_markers.push_back(markers[i].id);
				cpswarm_msgs::TargetTracking track;
				track.header.stamp = Time::now();
				track.id = markers[i].id;
				track.tf = transform(markers[i].pose.pose, pose);
				tracking_pub.publish(track);
			
			
			
		
	  }
    }*/
}

void targets::update (cpswarm_msgs::TargetPositionEvent msg, target_state_t state)
{
    // determine target pose
    geometry_msgs::Pose pose;
    pose = msg.pose.pose;

    ROS_DEBUG("Target %d at [%.2f, %.2f]", msg.id, pose.position.x, pose.position.y);

    // update existing target
    if (target_map.count(msg.id) > 0) {
        target_map[msg.id]->update(state, pose, msg.header.stamp);
    }

    // add new target
    else {
        target_map.emplace(piecewise_construct, forward_as_tuple(msg.id), forward_as_tuple(make_shared<target>(msg.id, state, pose, msg.header.stamp)));
    }
}

geometry_msgs::Transform targets::transform (geometry_msgs::Pose p1, geometry_msgs::Pose p2) const
{
    // orientation of first point
    tf2::Quaternion orientation1;/*
    tf2::fromMsg(p1.orientation, orientation1);

    // relative coordinates of second point
    double dx = p2.position.x - p1.position.x;
    double dy = p2.position.y - p1.position.y;
    double distance = hypot(dx, dy);
    double direction = (M_PI / 2.0) - tf2::getYaw(orientation1) + atan2(dy, dx);

    // compute transform
    geometry_msgs::Transform tf;

    // translation
    tf.translation.x = distance * cos(direction); // x is inverted in tracking camera tf
    tf.translation.y = distance * sin(direction);

    // rotation
    //p1.orientation.w = 1; 
    tf2::fromMsg(p1.orientation, orientation1);
    tf2::Quaternion orientation2;
    tf2::fromMsg(p2.orientation, orientation2);
    tf2::Quaternion rotation = orientation2 * orientation1;
    tf.rotation = tf2::toMsg(rotation);
  */

     // translation 1 meter before the frame
    geometry_msgs::Transform tf;
    tf2::fromMsg(p1.orientation, orientation1);
    tf2::Quaternion rotation = orientation1;
    tf.translation.x = p1.position.x + 1*cos(tf2::getYaw(orientation1)); // x is inverted in tracking camera tf
    tf.translation.y = p1.position.y + 1*sin(tf2::getYaw(orientation1));
    
    //rotation of 180 degrees to get the desired pre pick robot orientation
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(rotation , q_orig);
    double r=0, p=0, y=3.14159;
    q_rot.setRPY(r, p, y);
    q_new = q_rot*q_orig;  // Calculate the new orientation
    q_new.normalize();
    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    tf2::convert(q_new, rotation);   
   
    tf.rotation = tf2::toMsg(rotation);

    return tf;
}

void targets::uuid_callback (const swarmros::String::ConstPtr& msg)
{
    cps = msg->value;
}

void targets::ar_marker_callback (const ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg)
{
    markers = msg->markers;
    //
}
