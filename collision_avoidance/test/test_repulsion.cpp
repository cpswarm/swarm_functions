#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include "lib/repulsion.h"

using namespace ros;

/**
 * @brief Test the uninitialized repulsion class.
 */
TEST (UnitTestVectorization, testUninitialized)
{
    repulsion rep;

    EXPECT_FALSE(rep.calc());
    EXPECT_FALSE(rep.sp_pos());
    EXPECT_FALSE(rep.sp_vel());

    EXPECT_FLOAT_EQ(rep.get_dir().header.seq, 0);
    EXPECT_FLOAT_EQ(rep.get_dir().header.stamp.toSec(), Time().toSec());
    EXPECT_EQ(rep.get_dir().header.frame_id, "");
    EXPECT_FLOAT_EQ(rep.get_dir().pose.position.x, 0);
    EXPECT_FLOAT_EQ(rep.get_dir().pose.position.y, 0);
    EXPECT_FLOAT_EQ(rep.get_dir().pose.position.z, 0);
    EXPECT_FLOAT_EQ(rep.get_dir().pose.orientation.x, 0);
    EXPECT_FLOAT_EQ(rep.get_dir().pose.orientation.y, 0);
    EXPECT_FLOAT_EQ(rep.get_dir().pose.orientation.z, 0);
    EXPECT_FLOAT_EQ(rep.get_dir().pose.orientation.w, 1); // no rotation

    EXPECT_FLOAT_EQ(rep.get_pos().header.seq, 0);
    EXPECT_FLOAT_EQ(rep.get_pos().header.stamp.toSec(), Time().toSec());
    EXPECT_EQ(rep.get_pos().header.frame_id, "");
    EXPECT_FLOAT_EQ(rep.get_pos().pose.position.x, 0);
    EXPECT_FLOAT_EQ(rep.get_pos().pose.position.y, 0);
    EXPECT_FLOAT_EQ(rep.get_pos().pose.position.z, 0);
    EXPECT_FLOAT_EQ(rep.get_pos().pose.orientation.x, 0);
    EXPECT_FLOAT_EQ(rep.get_pos().pose.orientation.y, 0);
    EXPECT_FLOAT_EQ(rep.get_pos().pose.orientation.z, 0);
    EXPECT_FLOAT_EQ(rep.get_pos().pose.orientation.w, 0);

    EXPECT_FLOAT_EQ(rep.get_vel().linear.x, 0);
    EXPECT_FLOAT_EQ(rep.get_vel().linear.y, 0);
    EXPECT_FLOAT_EQ(rep.get_vel().linear.z, 0);
    EXPECT_FLOAT_EQ(rep.get_vel().angular.x, 0);
    EXPECT_FLOAT_EQ(rep.get_vel().angular.y, 0);
    EXPECT_FLOAT_EQ(rep.get_vel().angular.z, 0);
}

/**
 * @brief Test changing setpoint.
 */
TEST (UnitTestVectorization, testSetpoint)
{
    repulsion rep;

    // set position setpoint
    geometry_msgs::PoseStamped pos;
    boost::shared_ptr<geometry_msgs::PoseStamped> pos_p = boost::make_shared<geometry_msgs::PoseStamped>(pos);
    rep.set_sp_pos(pos_p);
    EXPECT_TRUE(rep.sp_pos());
    EXPECT_FALSE(rep.sp_vel());

    // set velocity setpoint
    geometry_msgs::Twist vel;
    boost::shared_ptr<geometry_msgs::Twist> vel_p = boost::make_shared<geometry_msgs::Twist>(vel);
    rep.set_sp_vel(vel_p);
    EXPECT_FALSE(rep.sp_pos());
    EXPECT_TRUE(rep.sp_vel());
}

/**
 * @brief Test invalid usage of repulsion class.
 */
TEST (UnitTestVectorization, testSetPoint)
{
    repulsion rep;

    // create single agent "swarm"
    cpswarm_msgs::ArrayOfVectors one;
    for (int i=0; i<1; ++i) {
        cpswarm_msgs::VectorStamped agent;
        agent.swarmio.node = to_string(i);
        one.vectors.push_back(agent);
    }
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> one_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(one);

    // create two agent "swarm"
    cpswarm_msgs::ArrayOfVectors two;
    for (int i=0; i<2; ++i) {
        cpswarm_msgs::VectorStamped agent;
        agent.swarmio.node = to_string(i);
        two.vectors.push_back(agent);
    }
    boost::shared_ptr<cpswarm_msgs::ArrayOfVectors> two_p = boost::make_shared<cpswarm_msgs::ArrayOfVectors>(two);

    // rep.set_swarm(swarm_p);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
