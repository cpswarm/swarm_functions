#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include "lib/repulsion.h"

using namespace ros;

 // quaternions calculated with https://quaternions.online/

/**
 * @brief Test repulsion class getters to be equal to the provided objects.
 * @param rep Reference to an repulsion class object.
 * @param dir Reference to a PoseStamped object containing the values expected from the get_dir function.
 * @param pos Reference to a PoseStamped object containing the values expected from the get_pos function.
 * @param vel Reference to a Twist object containing the values expected from the get_vel function.
 */
void test_getters (repulsion& rep, geometry_msgs::PoseStamped& dir, geometry_msgs::PoseStamped& pos, geometry_msgs::Twist& vel)
{
    EXPECT_EQ(rep.get_dir().header.seq, dir.header.seq);
    EXPECT_NEAR(rep.get_dir().header.stamp.toSec(), dir.header.stamp.toSec(), 0.001);
    EXPECT_EQ(rep.get_dir().header.frame_id, dir.header.frame_id);
    EXPECT_NEAR(rep.get_dir().pose.position.x, dir.pose.position.x, 0.001);
    EXPECT_NEAR(rep.get_dir().pose.position.y, dir.pose.position.y, 0.001);
    EXPECT_NEAR(rep.get_dir().pose.position.z, dir.pose.position.z, 0.001);
    EXPECT_NEAR(rep.get_dir().pose.orientation.x, dir.pose.orientation.x, 0.001);
    EXPECT_NEAR(rep.get_dir().pose.orientation.y, dir.pose.orientation.y, 0.001);
    EXPECT_NEAR(rep.get_dir().pose.orientation.z, dir.pose.orientation.z, 0.001);
    EXPECT_NEAR(rep.get_dir().pose.orientation.w, dir.pose.orientation.w, 0.001);

    EXPECT_EQ(rep.get_pos().header.seq, pos.header.seq);
    EXPECT_NEAR(rep.get_pos().header.stamp.toSec(), pos.header.stamp.toSec(), 0.001);
    EXPECT_EQ(rep.get_pos().header.frame_id, pos.header.frame_id);
    EXPECT_NEAR(rep.get_pos().pose.position.x, pos.pose.position.x, 0.001);
    EXPECT_NEAR(rep.get_pos().pose.position.y, pos.pose.position.y, 0.001);
    EXPECT_NEAR(rep.get_pos().pose.position.z, pos.pose.position.z, 0.001);
    EXPECT_NEAR(rep.get_pos().pose.orientation.x, pos.pose.orientation.x, 0.001);
    EXPECT_NEAR(rep.get_pos().pose.orientation.y, pos.pose.orientation.y, 0.001);
    EXPECT_NEAR(rep.get_pos().pose.orientation.z, pos.pose.orientation.z, 0.001);
    EXPECT_NEAR(rep.get_pos().pose.orientation.w, pos.pose.orientation.w, 0.001);

    EXPECT_NEAR(rep.get_vel().linear.x, vel.linear.x, 0.001);
    EXPECT_NEAR(rep.get_vel().linear.y, vel.linear.y, 0.001);
    EXPECT_NEAR(rep.get_vel().linear.z, vel.linear.z, 0.001);
    EXPECT_NEAR(rep.get_vel().angular.x, vel.angular.x, 0.001);
    EXPECT_NEAR(rep.get_vel().angular.y, vel.angular.y, 0.001);
    EXPECT_NEAR(rep.get_vel().angular.z, vel.angular.z, 0.001);
}

/**
 * @brief Test the uninitialized repulsion class.
 */
TEST (UnitTestRepulsion, testUninitialized)
{
    repulsion rep;

    EXPECT_FALSE(rep.calc());
    EXPECT_FALSE(rep.sp_pos());
    EXPECT_FALSE(rep.sp_vel());

    geometry_msgs::PoseStamped dir;
    dir.pose.orientation.w = 1; // no rotation
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;
    test_getters(rep, dir, pos, vel);
}

/**
 * @brief Test invalid usage of repulsion class: Calculate repulsion with missing information.
 */
TEST (UnitTestRepulsion, testMissingInfo)
{
    // create repulsion object
    repulsion rep;

    // create result objects
    geometry_msgs::PoseStamped dir;
    dir.pose.orientation.w = 1; // no rotation
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // initialize repulsion
    rep.init(5, 10, "linear", 0);

    // test without calculation
    test_getters(rep, dir, pos, vel);

    // test without swarm, only calculate
    EXPECT_FALSE(rep.calc());
    test_getters(rep, dir, pos, vel);

    // add swarm, test without position
    cpswarm_msgs::ArrayOfVectors swarm;
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);
    EXPECT_FALSE(rep.calc());
    test_getters(rep, dir, pos, vel);

    // add position, test without setpoint
    geometry_msgs::PoseStamped pose;
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_pos(pos_p);
    EXPECT_FALSE(rep.calc());
    test_getters(rep, dir, pos, vel);

    // add setpoint, test with all empty values
    rep.set_sp_pos(pos_p);
    EXPECT_FALSE(rep.calc());
    test_getters(rep, dir, pos, vel);

    // define swarm, add one agent far away
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 100;
    swarm_p->vectors.push_back(agent);
    rep.set_swarm(swarm_p);
    EXPECT_FALSE(rep.calc());
    // no repulsion
    test_getters(rep, dir, pos, vel);

    // move agent close by
    swarm_p->vectors[0].vector.magnitude = 1;
    rep.set_swarm(swarm_p);
    EXPECT_TRUE(rep.calc());
    // only repulsion: target x=1 (because yaw bearing = 0), repulsion x=-1 (because other agent at yaw = 0)
    dir.pose.orientation.z = 1;
    dir.pose.orientation.w =  0;
    pos.pose.position.x = -2.5;
    test_getters(rep, dir, pos, vel);

    // test with changed position
    pos_p->pose.position.y = 1;
    rep.set_pos(pos_p);
    EXPECT_TRUE(rep.calc());
    // repulsion x=-2.5, y=-2.5: target y=-1 (because no setpoint given = 0), repulsion x=-1 (because other agent at yaw = 0)
    dir.pose.position.y = 1;
    dir.pose.orientation.z = -0.924;
    dir.pose.orientation.w =  0.383;
    pos.pose.position.x = -2.5 / sqrt(2);
    pos.pose.position.y = 1 - 2.5 / sqrt(2);
    test_getters(rep, dir, pos, vel);

    // test with changed setpoint
    pos_p->pose.position.y = 2;
    rep.set_sp_pos(pos_p);
    EXPECT_TRUE(rep.calc());
    // repulsion x=-2.5, y=2.5: target y=+1 (because setpoint at y=2), repulsion x=-1 (because other agent at yaw = 0)
    dir.pose.orientation.z = 0.924;
    dir.pose.orientation.w =  0.383;
    pos.pose.position.x = -2.5 / sqrt(2);
    pos.pose.position.y = 1 + 2.5 / sqrt(2);
    test_getters(rep, dir, pos, vel);
}

/**
 * @brief Test invalid usage of repulsion class: Calculate repulsion with missing information. Velocity setpoint.
 */
TEST (UnitTestRepulsion, testMissingInfoVelocity)
{
    // create repulsion object
    repulsion rep;

    // create result objects
    geometry_msgs::PoseStamped dir;
    dir.pose.orientation.w = 1; // no rotation
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // initialize repulsion
    rep.init(5, 10, "linear", 0);

    // add swarm with one agent close by
    cpswarm_msgs::ArrayOfVectors swarm;
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 1;
    swarm.vectors.push_back(agent);
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);

    // add velocity setpoint, test without position
    geometry_msgs::Twist velo;
    boost::shared_ptr<geometry_msgs::Twist> vel_p = boost::make_shared<geometry_msgs::Twist>(velo);
    rep.set_sp_vel(vel_p);
    EXPECT_FALSE(rep.calc());
    test_getters(rep, dir, pos, vel);

    // add position, test with all empty values
    geometry_msgs::PoseStamped pose;
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_pos(pos_p);
    EXPECT_TRUE(rep.calc());
    // repulsion x=-2.5, y=0: target = 0 (because no setpoint given = 0), repulsion x=-1 (because other agent at yaw = 0)
    dir.pose.orientation.z = 1;
    dir.pose.orientation.w = 0;
    vel.linear.x = -2.5;
    test_getters(rep, dir, pos, vel);
}

/**
 * @brief Test changing setpoint.
 */
TEST (UnitTestRepulsion, testSetpointOnly)
{
    // create repulsion object
    repulsion rep;

    // create result objects
    geometry_msgs::PoseStamped dir;
    dir.pose.orientation.w = 1; // no rotation
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // test position setpoint without initialization
    geometry_msgs::PoseStamped pose;
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_sp_pos(pos_p);
    EXPECT_TRUE(rep.sp_pos());
    EXPECT_FALSE(rep.sp_vel());

    // test velocity setpoint without initialization
    geometry_msgs::Twist velo;
    boost::shared_ptr<geometry_msgs::Twist> vel_p = boost::make_shared<geometry_msgs::Twist>(velo);
    rep.set_sp_vel(vel_p);
    EXPECT_FALSE(rep.sp_pos());
    EXPECT_TRUE(rep.sp_vel());

    // initialize repulsion
    rep.init(5, 10, "linear", 0);

    // add swarm with one agent close by
    cpswarm_msgs::ArrayOfVectors swarm;
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 1;
    swarm.vectors.push_back(agent);
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);

    // add position, test with all empty values
    rep.set_pos(pos_p);
    EXPECT_TRUE(rep.calc());
    // repulsion x=-2.5, y=0: target = 0 (because no setpoint given = 0), repulsion x=-1 (because other agent at yaw = 0)
    dir.pose.orientation.z = 1;
    dir.pose.orientation.w = 0;
    vel.linear.x = -2.5;
    test_getters(rep, dir, pos, vel);

    // change to position setpoint, test with all empty values
    rep.set_sp_pos(pos_p);
    EXPECT_TRUE(rep.calc());
    // only repulsion: target x=1 (because yaw bearing = 0), repulsion x=-1 (because other agent at yaw = 0)
    pos.pose.position.x = -2.5;
    dir.pose.orientation.z = 1;
    dir.pose.orientation.w = 0;
    vel.linear.x = 0;
    test_getters(rep, dir, pos, vel);
}

/**
 * @brief Test position based repulsion with changing agent orientation.
 */
TEST (UnitTestRepulsion, testPositionOrientation)
{
    // create repulsion object
    repulsion rep;

    // initialize repulsion
    rep.init(5, 10, "linear", 0);

    // create result objects
    geometry_msgs::PoseStamped dir;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // add swarm with one agent close by, straight ahead
    cpswarm_msgs::ArrayOfVectors swarm;
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 1;
    swarm.vectors.push_back(agent);
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);

    // set position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 5;
    pose.pose.position.y = 9;
    pose.pose.orientation.w = 1; // facing in x direction
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_pos(pos_p);

    // set poisition setpoint in x direction
    pos_p->pose.position.x = 10;
    pos_p->pose.position.y = 9;
    rep.set_sp_pos(pos_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // deadlock, only apply repulsion and move in -x: target x=1 (because setpoint x=1), repulsion x=-1 (because other agent at yaw = 0)
    pos.pose.position.x = 5 - 2.5;
    pos.pose.position.y = 9;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 1; // -x direction
    dir.pose.orientation.w = 0; // -x direction
    test_getters(rep, dir, pos, vel);

    // turn into y direction
    pos_p->pose.position.x = 5;
    pos_p->pose.orientation.z = 0.707;
    pos_p->pose.orientation.w = 0.707;
    rep.set_pos(pos_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in x and -y direction: target x=1 (because setpoint x=1), repulsion y=-1 (because other agent at y=+1)
    pos.pose.position.x = 5 + 2.5 / sqrt(2);
    pos.pose.position.y = 9 - 2.5 / sqrt(2);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.383; // x and -y direction
    dir.pose.orientation.w = 0.924; // x and -y direction
    test_getters(rep, dir, pos, vel);

    // turn into negative y direction
    pos_p->pose.orientation.z = -0.707;
    pos_p->pose.orientation.w = 0.707;
    rep.set_pos(pos_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in +x and +y direction: target x=1 (because setpoint x=1), repulsion y=+1 (because other agent at y=-1)
    pos.pose.position.x = 5 + 2.5 / sqrt(2);
    pos.pose.position.y = 9 + 2.5 / sqrt(2);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.383; // x and y direction
    dir.pose.orientation.w = 0.924; // x and y direction
    test_getters(rep, dir, pos, vel);

    // turn into negative x direction
    pos_p->pose.orientation.z = 1;
    pos_p->pose.orientation.w = 0;
    rep.set_pos(pos_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in +x: target x=1 (because setpoint x=1), repulsion x=1 (because other agent at x=-1)
    pos.pose.position.x = 5 + 2.5;
    pos.pose.position.y = 9;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0; // x direction
    dir.pose.orientation.w = 1; // x direction
    test_getters(rep, dir, pos, vel);

}

/**
 * @brief Test position based repulsion with changing obstacle direction.
 */
TEST (UnitTestRepulsion, testPositionObstDir)
{
    // create repulsion object
    repulsion rep;

    // initialize repulsion
    rep.init(5, 10, "linear", 0);

    // create result objects
    geometry_msgs::PoseStamped dir;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // add swarm with one agent close by, straight ahead
    cpswarm_msgs::ArrayOfVectors swarm;
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 1;
    swarm.vectors.push_back(agent);
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);

    // set position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 5;
    pose.pose.position.y = 9;
    pose.pose.orientation.w = 1; // facing in x direction
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_pos(pos_p);

    // set poisition setpoint in x direction
    pos_p->pose.position.x = 10;
    pos_p->pose.position.y = 9;
    rep.set_sp_pos(pos_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // deadlock, only apply repulsion and move in -x: target x=1 (because setpoint x=1), repulsion x=-1 (because other agent at yaw = 0)
    pos.pose.position.x = 5 - 2.5;
    pos.pose.position.y = 9;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 1; // -x direction
    dir.pose.orientation.w = 0; // -x direction
    test_getters(rep, dir, pos, vel);

    // obstacle left
    swarm_p->vectors[0].vector.direction = 1.571;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in x and -y direction: target x=1 (because setpoint x=1), repulsion y=-1 (because other agent at y=+1)
    pos.pose.position.x = 5 + 2.5 / sqrt(2);
    pos.pose.position.y = 9 - 2.5 / sqrt(2);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.383; // x and -y direction
    dir.pose.orientation.w = 0.924; // x and -y direction
    test_getters(rep, dir, pos, vel);

    // obstacle right
    swarm_p->vectors[0].vector.direction = -1.571;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in +x and +y direction: target x=1 (because setpoint x=1), repulsion y=+1 (because other agent at y=-1)
    pos.pose.position.x = 5 + 2.5 / sqrt(2);
    pos.pose.position.y = 9 + 2.5 / sqrt(2);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.383; // x and y direction
    dir.pose.orientation.w = 0.924; // x and y direction
    test_getters(rep, dir, pos, vel);

    // obstacle behind
    swarm_p->vectors[0].vector.direction = 3.1415;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in +x: target x=1 (because setpoint x=1), repulsion x=1 (because other agent at x=-1)
    pos.pose.position.x = 5 + 2.5;
    pos.pose.position.y = 9;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0; // x direction
    dir.pose.orientation.w = 1; // x direction
    test_getters(rep, dir, pos, vel);
}

/**
 * @brief Test position based repulsion with changing obstacle distance.
 */
TEST (UnitTestRepulsion, testPositionObstDist)
{
    // create repulsion object
    repulsion rep;

    // initialize repulsion
    rep.init(5, 10, "linear", 0);

    // create result objects
    geometry_msgs::PoseStamped dir;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // add swarm with one agent close by, straight ahead
    cpswarm_msgs::ArrayOfVectors swarm;
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 1;
    swarm.vectors.push_back(agent);
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);

    // set position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 5;
    pose.pose.position.y = 9;
    pose.pose.orientation.z = 0.707; // facing in y direction
    pose.pose.orientation.w = 0.707; // facing in y direction
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_pos(pos_p);

    // set poisition setpoint in x direction
    pos_p->pose.position.x = 10;
    pos_p->pose.position.y = 9;
    rep.set_sp_pos(pos_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in x and -y direction: target x=1 (because setpoint x=1), repulsion y=-1 (because other agent at y=+1)
    pos.pose.position.x = 5 + 2.5 / sqrt(2);
    pos.pose.position.y = 9 - 2.5 / sqrt(2);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.383; // x and -y direction
    dir.pose.orientation.w = 0.924; // x and -y direction
    test_getters(rep, dir, pos, vel);

    // obstacle at critical distance
    swarm_p->vectors[0].vector.magnitude = 5;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in x and -y direction: target x=1 (because setpoint x=1), repulsion y=-1 (because other agent at y=+1)
    pos.pose.position.x = 5 + 2.5 / sqrt(2);
    pos.pose.position.y = 9 - 2.5 / sqrt(2);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.383; // x and -y direction
    dir.pose.orientation.w = 0.924; // x and -y direction
    test_getters(rep, dir, pos, vel);

    // obstacle between critical and avoidance distance
    swarm_p->vectors[0].vector.magnitude = 7.5;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in 0.5*x and -0.25*y direction: target x=1 (because setpoint x=1), repulsion y=-0.5 (because other agent at y=+1)
    pos.pose.position.x = 5 + 7.5/2 / sqrt(5/4.0);
    pos.pose.position.y = 9 - 7.5/2 / sqrt(5);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.230; // 0.5*x and -0.25*y direction = 26.565°
    dir.pose.orientation.w = 0.973; //
    test_getters(rep, dir, pos, vel);

    // obstacle at avoidance distance
    swarm_p->vectors[0].vector.magnitude = 10;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_FALSE(rep.calc());
    // no movement because other agent out of range
    pos.pose.position.x = 0; // avoidance position is reset
    pos.pose.position.y = 0;
    dir.pose.position.x = 5; // agent position is kept
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0; // avoidance direction is reset
    dir.pose.orientation.w = 1; // no rotation
    test_getters(rep, dir, pos, vel);
}

/**
 * @brief Test position based repulsion with changing position setpoint.
 */
TEST (UnitTestRepulsion, testPositionSetpoint)
{
    // create repulsion object
    repulsion rep;

    // initialize repulsion
    rep.init(5, 10, "linear", 0);

    // create result objects
    geometry_msgs::PoseStamped dir;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // add swarm with one agent close by, straight ahead
    cpswarm_msgs::ArrayOfVectors swarm;
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 1;
    swarm.vectors.push_back(agent);
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);

    // set position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 5;
    pose.pose.position.y = 9;
    pose.pose.orientation.w = 1; // facing in x direction
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_pos(pos_p);

    // set poisition setpoint in x direction
    pos_p->pose.position.x = 10;
    pos_p->pose.position.y = 9;
    rep.set_sp_pos(pos_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // deadlock, only apply repulsion and move in -x: target x=1 (because setpoint x=1), repulsion x=-1 (because other agent at yaw = 0)
    pos.pose.position.x = 5 - 2.5;
    pos.pose.position.y = 9;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 1; // -x direction
    dir.pose.orientation.w = 0; // -x direction
    test_getters(rep, dir, pos, vel);

    // change setpoint to negative y direction
    pos_p->pose.position.x = 5;
    pos_p->pose.position.y = 0;
    rep.set_sp_pos(pos_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in -x and -y direction: target y=-1 (because setpoint y=-1), repulsion x=-1 (because other agent at yaw = 0)
    pos.pose.position.x = 5 - 2.5 / sqrt(2);
    pos.pose.position.y = 9 - 2.5 / sqrt(2);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.924; // x and -y direction
    dir.pose.orientation.w = 0.383; // x and -y direction
    test_getters(rep, dir, pos, vel);

    // change setpoint to positive y direction
    pos_p->pose.position.y = 10;
    rep.set_sp_pos(pos_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in -x and +y direction: target y=1 (because setpoint y=1), repulsion x=-1 (because other agent at yaw = 0)
    pos.pose.position.x = 5 - 2.5 / sqrt(2);
    pos.pose.position.y = 9 + 2.5 / sqrt(2);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.924; // x and y direction
    dir.pose.orientation.w = 0.383; // x and y direction
    test_getters(rep, dir, pos, vel);

    // change setpoint to negative x direction
    pos_p->pose.position.x = 0;
    pos_p->pose.position.y = 9;
    rep.set_sp_pos(pos_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in -x: target x=-1 (because setpoint x=-1), repulsion x=-1 (because other agent at yaw = 0)
    pos.pose.position.x = 5 - 2.5;
    pos.pose.position.y = 9;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 1; // -x direction
    dir.pose.orientation.w = 0; // -x direction
    test_getters(rep, dir, pos, vel);
}

/**
 * @brief Test position based repulsion with changing repulsion shape.
 */
TEST (UnitTestRepulsion, testPositionRepShape)
{
    // create repulsion object
    repulsion rep;

    // create result objects
    geometry_msgs::PoseStamped dir;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // add swarm with one agent straight ahead, between critical and avoidance distance
    cpswarm_msgs::ArrayOfVectors swarm;
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 8.75;
    swarm.vectors.push_back(agent);
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);

    // set position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 5;
    pose.pose.position.y = 9;
    pose.pose.orientation.z = 0.707; // facing in y direction
    pose.pose.orientation.w = 0.707; // facing in y direction
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_pos(pos_p);

    // set poisition setpoint in x direction
    pos_p->pose.position.x = 10;
    pos_p->pose.position.y = 9;
    rep.set_sp_pos(pos_p);

    // initialize repulsion with linear function
    rep.init(5, 10, "linear", 0);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in x and -y direction: target x=1 (because setpoint x=1), repulsion y=-1 (because other agent at y=+1)
    double pot = 0.25;
    double mag = hypot(1, pot);
    pos.pose.position.x = 5 + 0.5 * 8.75 * 1 / mag;
    pos.pose.position.y = 9 - 0.5 * 8.75 * pot/mag;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.122; // 0.5*x and -0.125*y direction = 14.0362°
    dir.pose.orientation.w = 0.993; // x and -y direction
    test_getters(rep, dir, pos, vel);

    // initialize repulsion with sine function
    rep.init(5, 10, "sine", 0);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in x and -y direction: target x=1 (because setpoint x=1), repulsion y=-1 (because other agent at y=+1)
    pot = 0.1464466;
    mag = hypot(1, pot);
    pos.pose.position.x = 5 + 0.5 * 8.75 * 1 / mag;
    pos.pose.position.y = 9 - 0.5 * 8.75 * pot/mag;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.073; // 0.5*x and -0.0732*y direction = 8.3315°
    dir.pose.orientation.w = 0.997; // x and -y direction
    test_getters(rep, dir, pos, vel);

    // initialize repulsion with sine function
    rep.init(5, 10, "exp", 0);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in x and -y direction: target x=1 (because setpoint x=1), repulsion y=-1 (because other agent at y=+1)
    pot = 0.35355339;
    mag = hypot(1, pot);
    pos.pose.position.x = 5 + 0.5 * 8.75 * 1 / mag;
    pos.pose.position.y = 9 - 0.5 * 8.75 * pot/mag;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.169; // 0.5*x and -0.1768*y direction = 19.4712°
    dir.pose.orientation.w = 0.986; // x and -y direction
    test_getters(rep, dir, pos, vel);
}

/**
 * @brief Test position based repulsion with changing avoidance and critical distances.
 */
TEST (UnitTestRepulsion, testPositionDistances)
{
    // create repulsion object
    repulsion rep;

    // create result objects
    geometry_msgs::PoseStamped dir;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // add swarm with one agent straight ahead
    cpswarm_msgs::ArrayOfVectors swarm;
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 5;
    swarm.vectors.push_back(agent);
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);

    // set position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 5;
    pose.pose.position.y = 9;
    pose.pose.orientation.z = 0.707; // facing in y direction
    pose.pose.orientation.w = 0.707; // facing in y direction
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_pos(pos_p);

    // set poisition setpoint in x direction
    pos_p->pose.position.x = 10;
    pos_p->pose.position.y = 9;
    rep.set_sp_pos(pos_p);

    // initialize repulsion with invalid distances
    rep.init(10, 5, "linear", 0);

    // calculate repulsion
    EXPECT_FALSE(rep.calc());
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.w = 1; // no rotation
    test_getters(rep, dir, pos, vel);

    // initialize repulsion with equal distances, agent far away
    rep.init(1, 1, "linear", 0);

    // calculate repulsion
    EXPECT_FALSE(rep.calc());
    test_getters(rep, dir, pos, vel);

    // initialize repulsion with equal distances, agent critically close
    rep.init(10, 10, "linear", 0);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in x and -y direction: target x=1 (because setpoint x=1), repulsion y=-1 (because other agent at y=+1)
    pos.pose.position.x = 5 + 5 / sqrt(2);
    pos.pose.position.y = 9 - 5 / sqrt(2) ;
    dir.pose.orientation.z = -0.383; // x and -y direction
    dir.pose.orientation.w = 0.924; // x and -y direction
    test_getters(rep, dir, pos, vel);
}

// /**
//  * @brief Test position based repulsion with changing avoidance bias.
//  */
// TEST (UnitTestRepulsion, testPositionBias)
// {
//     // create repulsion object
//     repulsion rep;

//     // create result objects
//     geometry_msgs::PoseStamped dir;
//     geometry_msgs::PoseStamped pos;
//     geometry_msgs::Twist vel;

//     // add swarm with one agent close by, straight ahead
//     cpswarm_msgs::ArrayOfVectors swarm;
//     cpswarm_msgs::VectorStamped agent;
//     agent.vector.magnitude = 1;
//     swarm.vectors.push_back(agent);
//     boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
//     rep.set_swarm(swarm_p);

//     // set position
//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = 5;
//     pose.pose.position.y = 9;
//     pose.pose.orientation.w = 1; // facing in x direction
//     boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
//     rep.set_pos(pos_p);

//     // set poisition setpoint in x direction
//     pos_p->pose.position.x = 10;
//     pos_p->pose.position.y = 9;
//     rep.set_sp_pos(pos_p);

//     // initialize repulsion with no bias
//     rep.init(5, 10, "linear", 0);

//     // calculate repulsion
//     EXPECT_TRUE(rep.calc());
//     // deadlock, no movement: target x=1 (because setpoint x=1), repulsion x=-1 (because other agent at yaw = 0)
//     pos.pose.position.x = 5;
//     pos.pose.position.y = 9;
//     dir.pose.position.x = 5;
//     dir.pose.position.y = 9;
//     dir.pose.orientation.w = 1; // x direction
//     test_getters(rep, dir, pos, vel);

//     // initialize repulsion with small positive bias
//     rep.init(5, 10, "linear", 0.1);
//     // TODO: what should the bias actually do??

//     // calculate repulsion
//     EXPECT_TRUE(rep.calc());
//     // avoid a bit counter-clockwise: target x=1 (because setpoint x=1), repulsion x=-1 (because other agent at yaw = 0)
//     pos.pose.position.x = 5 + 5 * (1-cos(0.1))/2;
//     pos.pose.position.y = 9 + 5 * sin(0.1)/2;
//     dir.pose.position.x = 5;
//     dir.pose.position.y = 9;
//     dir.pose.orientation.w = 1; // x direction
//     test_getters(rep, dir, pos, vel);
// }

/**
 * @brief Test position based repulsion with different number of agents in swarm.
 */
TEST (UnitTestRepulsion, testPositionSwarmSize)
{
    // create repulsion object
    repulsion rep;

    // initialize repulsion
    rep.init(5, 10, "linear", 0);

    // create result objects
    geometry_msgs::PoseStamped dir;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // set position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 5;
    pose.pose.position.y = 9;
    pose.pose.orientation.z = 0.707; // facing in y direction
    pose.pose.orientation.w = 0.707; // facing in y direction
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_pos(pos_p);

    // set poisition setpoint in y direction
    pos_p->pose.position.x = 5;
    pos_p->pose.position.y = 10;
    rep.set_sp_pos(pos_p);

    // add swarm with two agent close by, one left and one right
    cpswarm_msgs::ArrayOfVectors swarm;
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 1;
    agent.vector.direction = M_PI / 2.0;
    swarm.vectors.push_back(agent);
    agent.vector.direction = -M_PI / 2.0;
    swarm.vectors.push_back(agent);
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in y direction: target in y direction (because setpoint in y direction), no repulsion (because other agents repulsion cancels out)
    pos.pose.position.x = 5;
    pos.pose.position.y = 9 + 2.5;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.707; // facing in y direction
    dir.pose.orientation.w = 0.707; // facing in y direction
    test_getters(rep, dir, pos, vel);

    // place agents: one forward left and one forward right
    swarm_p->vectors[0].vector.direction = M_PI / 4.0;
    swarm_p->vectors[1].vector.direction = -M_PI / 4.0;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in -y direction: target in y direction (because setpoint in y direction), repulsion in -y direction (because x repulsion cancels out)
    pos.pose.position.x = 5;
    pos.pose.position.y = 9 - 2.5;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.707; // facing in -y direction
    dir.pose.orientation.w = 0.707; // facing in -y direction
    test_getters(rep, dir, pos, vel);

    // place agents: both ahead
    swarm_p->vectors[0].vector.direction = 0;
    swarm_p->vectors[1].vector.direction = 0;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in -y direction: target in y direction (because setpoint in y direction), repulsion in -y direction
    pos.pose.position.x = 5;
    pos.pose.position.y = 9 - 2.5;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = -0.707; // facing in -y direction
    dir.pose.orientation.w = 0.707; // facing in -y direction
    test_getters(rep, dir, pos, vel);

    // place agents: both behind left and right
    swarm_p->vectors[0].vector.direction = 3 * M_PI / 4.0;
    swarm_p->vectors[1].vector.direction = -3 * M_PI / 4.0;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in y direction: target in y direction (because setpoint in y direction), repulsion in y direction (because x repulsion cancels out)
    pos.pose.position.x = 5;
    pos.pose.position.y = 9 + 2.5;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.707; // facing in y direction
    dir.pose.orientation.w = 0.707; // facing in y direction
    test_getters(rep, dir, pos, vel);

    // place agents: both behind
    swarm_p->vectors[0].vector.direction = M_PI;
    swarm_p->vectors[1].vector.direction = M_PI;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in y direction: target in y direction (because setpoint in y direction), repulsion in y direction
    pos.pose.position.x = 5;
    pos.pose.position.y = 9 + 2.5;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.707; // facing in y direction
    dir.pose.orientation.w = 0.707; // facing in y direction
    test_getters(rep, dir, pos, vel);

    // place agents: one ahead, one behind
    swarm_p->vectors[0].vector.direction = 0;
    swarm_p->vectors[1].vector.direction = M_PI;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in y direction: target in y direction (because setpoint in y direction), no repulsion (cancels out)
    pos.pose.position.x = 5;
    pos.pose.position.y = 9 + 2.5;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.707; // facing in y direction
    dir.pose.orientation.w = 0.707; // facing in y direction
    test_getters(rep, dir, pos, vel);

    // add a third agent to the left
    agent.vector.direction = M_PI / 2.0;
    swarm_p->vectors.push_back(agent);
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in xy direction: target in xy direction (because setpoint in y direction), repulsion in x direction
    pos.pose.position.x = 5 + 2.5 / sqrt(2);
    pos.pose.position.y = 9 + 2.5 / sqrt(2);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.383; // facing in xy direction
    dir.pose.orientation.w = 0.924; // facing in xy direction
    test_getters(rep, dir, pos, vel);

    // move agent ahead further away
    swarm_p->vectors[0].vector.magnitude = 5;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in xy direction: target in y direction (because setpoint in y direction), repulsion in x direction
    pos.pose.position.x = 5 + 2.5 / sqrt(2);
    pos.pose.position.y = 9 + 2.5 / sqrt(2);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.383; // facing in xy direction
    dir.pose.orientation.w = 0.924; // facing in xy direction
    test_getters(rep, dir, pos, vel);

    // move agent ahead further away
    swarm_p->vectors[0].vector.magnitude = 7.5;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in xyy direction: target in y direction (because setpoint in y direction), repulsion in x direction
    pos.pose.position.x = 5 + 2.5 / sqrt(13/4.0);
    pos.pose.position.y = 9 + 2.5 / sqrt(13/9.0);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.472; // facing in xyy direction
    dir.pose.orientation.w = 0.882; // facing in xyy direction
    test_getters(rep, dir, pos, vel);

    // move agent ahead further away
    swarm_p->vectors[0].vector.magnitude = 10;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in xyy direction: target in y direction (because setpoint in y direction), repulsion in x direction
    pos.pose.position.x = 5 + 2.5 / sqrt(5);
    pos.pose.position.y = 9 + 2.5 / sqrt(1.25);
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.526; // facing in xyy direction
    dir.pose.orientation.w = 0.851; // facing in xyy direction
    test_getters(rep, dir, pos, vel);
}

/**
 * @brief Test velocity based repulsion.
 */
TEST (UnitTestRepulsion, testVelocity)
{
    // create repulsion object
    repulsion rep;

    // initialize repulsion
    rep.init(5, 10, "linear", 0);

    // create result objects
    geometry_msgs::PoseStamped dir;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    // set position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 5;
    pose.pose.position.y = 9;
    pose.pose.orientation.z = 0.707; // facing in y direction
    pose.pose.orientation.w = 0.707; // facing in y direction
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pose);
    rep.set_pos(pos_p);

    // add swarm with two agent close by, one left and one right
    cpswarm_msgs::ArrayOfVectors swarm;
    cpswarm_msgs::VectorStamped agent;
    agent.vector.magnitude = 1;
    agent.vector.direction = M_PI / 2.0;
    swarm.vectors.push_back(agent);
    agent.vector.direction = -M_PI / 2.0;
    swarm.vectors.push_back(agent);
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> swarm_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(swarm);
    rep.set_swarm(swarm_p);

    // set velocity setpoint in y direction
    geometry_msgs::Twist velo;
    velo.linear.x = 0;
    velo.linear.y = 1;
    boost::shared_ptr<geometry_msgs::Twist> vel_p = boost::make_shared<geometry_msgs::Twist>(velo);
    rep.set_sp_vel(vel_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in y direction: target in y direction (because setpoint in y direction), no repulsion (because other agents repulsion cancels out)
    vel.linear.x = 0;
    vel.linear.y = 2.5;
    dir.pose.position.x = 5;
    dir.pose.position.y = 9;
    dir.pose.orientation.z = 0.707; // facing in y direction
    dir.pose.orientation.w = 0.707; // facing in y direction
    test_getters(rep, dir, pos, vel);

    // set velocity setpoint in x direction
    vel_p->linear.x = 1;
    vel_p->linear.y = 0;
    rep.set_sp_vel(vel_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in x direction: target in x direction (because setpoint in x direction), no repulsion (because other agents repulsion cancels out)
    vel.linear.x = 2.5;
    vel.linear.y = 0;
    dir.pose.orientation.z = 0; // facing in x direction
    dir.pose.orientation.w = 1; // facing in x direction
    test_getters(rep, dir, pos, vel);

    // set velocity setpoint in xy direction
    vel_p->linear.x = 1;
    vel_p->linear.y = 1;
    rep.set_sp_vel(vel_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in xy direction: target in xy direction (because setpoint in x direction), no repulsion (because other agents repulsion cancels out)
    vel.linear.x = 2.5 / sqrt(2);
    vel.linear.y = 2.5 / sqrt(2);
    dir.pose.orientation.z =  0.383; // facing in xy direction
    dir.pose.orientation.w = 0.924; // facing in xy direction
    test_getters(rep, dir, pos, vel);

    // move agent left further away
    swarm_p->vectors[0].vector.magnitude = 7.5;
    rep.set_swarm(swarm_p);

    // calculate repulsion
    EXPECT_TRUE(rep.calc());
    // movement in xyy direction: target in xy direction (because setpoint in xy direction), repulsion in -x direction
    vel.linear.x = 2.5 * 0.28108463771482;
    vel.linear.y = 2.5 * 0.959682982260667;
    dir.pose.orientation.z = 0.6; // facing in xyy direction
    dir.pose.orientation.w = 0.8; // facing in xyy direction
    test_getters(rep, dir, pos, vel);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
