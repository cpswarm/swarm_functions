#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cpswarm_msgs/VectorStamped.h>
#include <cpswarm_msgs/ArrayOfVectors.h>

using namespace std;
using namespace ros;

/**
 * @brief Intermediate goal position during collision avoidance.
 */
geometry_msgs::PoseStamped ca_goal;

/**
 * @brief Intermediate target velocity during collision avoidance
 */
geometry_msgs::Twist ca_vel;

/**
 * @brief Direction for visualizing collision avoidance.
 */
geometry_msgs::PoseStamped ca_dir;

/**
 * @brief Callback function to receive the collision avoidance goal position.
 * @param msg A 2D position.
 */
void goal_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ca_goal = *msg;
}

/**
 * @brief Callback function to receive the collision avoidance target velocity.
 * @param msg A linear 2D velocity.
 */
void vel_callback (const geometry_msgs::Twist::ConstPtr& msg)
{
    ca_vel = *msg;
}

/**
 * @brief Callback function to receive the collision avoidance direction.
 * @param msg A 2D pose (position and orientation).
 */
void dir_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ca_dir = *msg;
}

/**
 * @brief Test the collision avoidance result to be equal to the provided objects.
 * @param pos Reference to a PoseStamped object containing the values expected from the get_pos function.
 * @param vel Reference to a Twist object containing the values expected from the get_vel function.
 * @param dir Reference to a PoseStamped object containing the values expected from the get_dir function.
 */
void test_results (geometry_msgs::PoseStamped& pos, geometry_msgs::Twist& vel, geometry_msgs::PoseStamped& dir)
{
    // EXPECT_EQ(ca_goal.header.seq, pos.header.seq);
    // EXPECT_NEAR(ca_goal.header.stamp.toSec(), pos.header.stamp.toSec(), 0.001);
    EXPECT_EQ(ca_goal.header.frame_id, pos.header.frame_id);
    EXPECT_NEAR(ca_goal.pose.position.x, pos.pose.position.x, 0.001);
    EXPECT_NEAR(ca_goal.pose.position.y, pos.pose.position.y, 0.001);
    EXPECT_NEAR(ca_goal.pose.position.z, pos.pose.position.z, 0.001);
    EXPECT_NEAR(ca_goal.pose.orientation.x, pos.pose.orientation.x, 0.001);
    EXPECT_NEAR(ca_goal.pose.orientation.y, pos.pose.orientation.y, 0.001);
    EXPECT_NEAR(ca_goal.pose.orientation.z, pos.pose.orientation.z, 0.001);
    EXPECT_NEAR(ca_goal.pose.orientation.w, pos.pose.orientation.w, 0.001);

    EXPECT_NEAR(ca_vel.linear.x, vel.linear.x, 0.001);
    EXPECT_NEAR(ca_vel.linear.y, vel.linear.y, 0.001);
    EXPECT_NEAR(ca_vel.linear.z, vel.linear.z, 0.001);
    EXPECT_NEAR(ca_vel.angular.x, vel.angular.x, 0.001);
    EXPECT_NEAR(ca_vel.angular.y, vel.angular.y, 0.001);
    EXPECT_NEAR(ca_vel.angular.z, vel.angular.z, 0.001);

    // EXPECT_EQ(ca_dir.header.seq, dir.header.seq);
    // EXPECT_NEAR(ca_dir.header.stamp.toSec(), dir.header.stamp.toSec(), 0.001);
    EXPECT_EQ(ca_dir.header.frame_id, dir.header.frame_id);
    EXPECT_NEAR(ca_dir.pose.position.x, dir.pose.position.x, 0.001);
    EXPECT_NEAR(ca_dir.pose.position.y, dir.pose.position.y, 0.001);
    EXPECT_NEAR(ca_dir.pose.position.z, dir.pose.position.z, 0.001);
    EXPECT_NEAR(ca_dir.pose.orientation.x, dir.pose.orientation.x, 0.001);
    EXPECT_NEAR(ca_dir.pose.orientation.y, dir.pose.orientation.y, 0.001);
    EXPECT_NEAR(ca_dir.pose.orientation.z, dir.pose.orientation.z, 0.001);
    EXPECT_NEAR(ca_dir.pose.orientation.w, dir.pose.orientation.w, 0.001);
}

/**
 * @brief Delete the collision avoidance result values.
 */
void reset_results ()
{
    ca_goal = geometry_msgs::PoseStamped();
    ca_vel = geometry_msgs::Twist();
    ca_dir = geometry_msgs::PoseStamped();
}

/**
 * @brief Test the collision avoidance publishers and subscribers with position setpoint.
 */
TEST (NodeTestCollisionAvoidance, testPositionSetpoint)
{
    NodeHandle nh;

    // create result objects
    geometry_msgs::PoseStamped dir;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // subscribers
    Subscriber goal_subscriber = nh.subscribe("pos_controller/ca_goal_position", 1, goal_callback);
    Subscriber vel_subscriber = nh.subscribe("vel_controller/ca_target_velocity", 1, vel_callback);
    Subscriber dir_subscriber = nh.subscribe("collision_avoidance/direction", 1, dir_callback);

    // publish behavior state with collision avoidance
    Publisher state_publisher = nh.advertise<std_msgs::String>("flexbe/behavior_update", 1, true); // latched
    std_msgs::String state;
    state.data = "foo";
    state_publisher.publish(state);
    Duration(1).sleep();

    // test collision avoidance result
    spinOnce();
    test_results(pos, vel, dir);

    // publish relative swarm position
    Publisher swarm_publisher = nh.advertise<cpswarm_msgs::ArrayOfVectors>("swarm_position_rel", 1, true); // latched
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 8;
    agent.vector.direction = M_PI / 2;
    cpswarm_msgs::ArrayOfVectors swarm;
    swarm.vectors.push_back(agent);
    swarm_publisher.publish(swarm);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    test_results(pos, vel, dir);

    // publish empty position setpoint
    Publisher sp_pos_publisher = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", 1, true); // latched
    geometry_msgs::PoseStamped sp_pos;
    sp_pos_publisher.publish(sp_pos);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    test_results(pos, vel, dir);

    // publish empty position
    Publisher pos_publisher = nh.advertise<geometry_msgs::PoseStamped>("pos_provider/pose", 1, true); // latched
    geometry_msgs::PoseStamped pose;
    pos_publisher.publish(pose);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    pos.pose.position.y = -4;
    dir.pose.orientation.z = -0.707;
    dir.pose.orientation.w = 0.707;
    test_results(pos, vel, dir);

    // publish position setpoint
    sp_pos.pose.position.x = -10;
    sp_pos_publisher.publish(sp_pos);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    pos.pose.position.x = -4 / sqrt(2);
    pos.pose.position.y = -4 / sqrt(2);
    dir.pose.orientation.z = -0.924;
    dir.pose.orientation.w = 0.383;
    test_results(pos, vel, dir);

    // publish behavior state without collision avoidance
    state.data = "bar";
    state_publisher.publish(state);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    pos = geometry_msgs::PoseStamped();
    dir = geometry_msgs::PoseStamped();
    test_results(pos, vel, dir);

    // publish behavior state with collision avoidance
    state.data = "asdf";
    state_publisher.publish(state);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    pos.pose.position.x = -4 / sqrt(2);
    pos.pose.position.y = -4 / sqrt(2);
    dir.pose.orientation.z = -0.924;
    dir.pose.orientation.w = 0.383;
    test_results(pos, vel, dir);
}

/**
 * @brief Test the collision avoidance publishers and subscribers with velocity setpoint.
 */
TEST (NodeTestCollisionAvoidance, testVelocitySetpoint)
{
    NodeHandle nh;

    // create result objects
    geometry_msgs::PoseStamped dir;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // subscribers
    Subscriber goal_subscriber = nh.subscribe("pos_controller/ca_goal_position", 1, goal_callback);
    Subscriber vel_subscriber = nh.subscribe("vel_controller/ca_target_velocity", 1, vel_callback);
    Subscriber dir_subscriber = nh.subscribe("collision_avoidance/direction", 1, dir_callback);

    // publish behavior state with collision avoidance
    Publisher state_publisher = nh.advertise<std_msgs::String>("flexbe/behavior_update", 1, true); // latched
    std_msgs::String state;
    state.data = "foo";
    state_publisher.publish(state);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    pos.pose.position.x = -4 / sqrt(2);
    pos.pose.position.y = -4 / sqrt(2);
    dir.pose.orientation.z = -0.924;
    dir.pose.orientation.w = 0.383;
    test_results(pos, vel, dir);

    // publish relative swarm position
    Publisher swarm_publisher = nh.advertise<cpswarm_msgs::ArrayOfVectors>("swarm_position_rel", 1, true); // latched
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 8;
    agent.vector.direction = M_PI / 2;
    cpswarm_msgs::ArrayOfVectors swarm;
    swarm.vectors.push_back(agent);
    swarm_publisher.publish(swarm);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    test_results(pos, vel, dir);

    // publish empty velocity setpoint
    Publisher sp_vel_publisher = nh.advertise<geometry_msgs::Twist>("vel_controller/target_velocity", 1, true); // latched
    geometry_msgs::Twist sp_vel;
    sp_vel_publisher.publish(sp_vel);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    vel.linear.y = -4;
    dir.pose.orientation.z = -0.707;
    dir.pose.orientation.w = 0.707;
    test_results(pos, vel, dir); // old pos is still in the queue

    // publish empty position
    Publisher pos_publisher = nh.advertise<geometry_msgs::PoseStamped>("pos_provider/pose", 1, true); // latched
    geometry_msgs::PoseStamped pose;
    pos_publisher.publish(pose);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    pos = geometry_msgs::PoseStamped(); // old pos is now gone
    test_results(pos, vel, dir);

    // publish velocity setpoint
    sp_vel.linear.x = -1;
    sp_vel_publisher.publish(sp_vel);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    vel.linear.x = -4 / sqrt(2);
    vel.linear.y = -4 / sqrt(2);
    dir.pose.orientation.z = -0.924;
    dir.pose.orientation.w = 0.383;
    test_results(pos, vel, dir);

    // publish behavior state without collision avoidance
    state.data = "bar";
    state_publisher.publish(state);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    vel = geometry_msgs::Twist();
    dir = geometry_msgs::PoseStamped();
    test_results(pos, vel, dir);

    // publish behavior state with collision avoidance
    state.data = "asdf";
    state_publisher.publish(state);
    Duration(1).sleep();

    // test collision avoidance result
    reset_results();
    spinOnce();
    vel.linear.x = -4 / sqrt(2);
    vel.linear.y = -4 / sqrt(2);
    dir.pose.orientation.z = -0.924;
    dir.pose.orientation.w = 0.383;
    test_results(pos, vel, dir);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    init(argc, argv, "test_area");
    return RUN_ALL_TESTS();
}
