#ifndef REPULSION_H
#define REPULSION_H

#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <cpswarm_msgs/ArrayOfVectors.h>
#include <cpswarm_msgs/VectorStamped.h>

using namespace std;

/**
 * @brief An enumeration for the type of spatial CPS control.
 */
typedef enum {
    CONTROL_UNDEFINED = 0, // setpoint type not defined
    CONTROL_POSITION,      // cps is controlled through position setpoints
    CONTROL_VELOCITY       // cps is controlled through velocity setpoints
} control_t;

/**
 * @brief A class that calculates the repulsive forces between CPSs.
 */
class repulsion
{
public:
    /**
     * @brief Constructor.
     */
    repulsion ();

    /**
     * @brief Configure the repulsion behavior through parameters.
     * @param dist_critical Distance between CPSs below which the collision avoidance will work maximally.
     * @param dist_avoid Distance between CPSs below which collision avoidance is active.
     * @param repulsion_shape The shape of the repulsion function.
     */
    void init (double dist_critical, double dist_avoid, string repulsion_shape);

    /**
     * @brief Check whether collision avoidance is necessary and calculate respective position or velocity.
     * @return Whether collision avoidance is necessary.
     */
    bool calc ();

    /**
     * @brief Check whether position setpoint is used.
     * @return True when the CPS is controlled with position setpoints.
     */
    bool sp_pos ();

    /**
     * @brief Check whether position setpoint is used.
     * @return True when the CPS is controlled with position setpoints.
     */
    bool sp_vel ();

    /**
     * @brief Set the original goal position for this CPS.
     * @param pos The desired goal position of this CPS in case of no collision avoidance.
     */
    void set_sp_pos (const geometry_msgs::PoseStamped::ConstPtr& pos);

    /**
     * @brief Set the original target velocity for this CPS.
     * @param vel The desired target velocity of this CPS in case of no collision avoidance.
     */
    void set_sp_vel (const geometry_msgs::Twist::ConstPtr& vel);

    /**
     * @brief Set the current position of this CPS.
     * @param pos The current position of this CPS.
     */
    void set_pos (const geometry_msgs::PoseStamped::ConstPtr& pos);

    /**
     * @brief Set the relative positions received from other CPSs in the swarm.
     * @param swarm An array of distance and bearing of the other CPSs.
     */
    void set_swarm (const cpswarm_msgs::ArrayOfVectors::ConstPtr& swarm);

    /**
     * @brief Get the direction in which to move for collision avoidance.
     * @return The direction away from the other CPSs.
     */
    geometry_msgs::PoseStamped get_dir ();

    /**
     * @brief Get the intermediate goal position during collision avoidance.
     * @return The pose where the CPS shall move to for avoiding a collision.
     */
    geometry_msgs::PoseStamped get_pos ();

    /**
     * @brief Get the intermediate target velocity during collision avoidance.
     * @return The velocity at which the CPS shall move for avoiding a collision.
     */
    geometry_msgs::Twist get_vel ();

private:
    /**
     * @brief Calculate repulsion from close by CPSs as pair potentials.
     * @param repulsion Returns the repulsion vector.
     * @param neighbors Returns the number of neighbors that create repulsion.
     */
    void repulse (geometry_msgs::Vector3& repulsion, int& neighbors);

    /**
     * Calculate the direction towards the original goal during collision avoidance.
     * @return A normalized vector pointing into the direction of the goal.
     */
    geometry_msgs::Vector3 target_direction ();

    /**
     * @brief The type of CPS control.
     */
    control_t setpoint;

    /**
     * @brief The current position of this CPS.
     */
    geometry_msgs::PoseStamped pos;

    /**
     * @brief The originally desired goal position of this CPS in case of no collision avoidance.
     */
    geometry_msgs::PoseStamped goal_pos;

    /**
     * @brief The originally desired target velocity of this CPS in case of no collision avoidance.
     */
    geometry_msgs::Twist target_vel;

    /**
     * @brief The relative positions of the other swarm members (distance and bearing).
     */
    vector<cpswarm_msgs::VectorStamped> swarm;

    /**
     * @brief The direction of avoidance.
     */
    geometry_msgs::Vector3 direction;

    /**
     * @brief The intermediate goal position where the CPS shall move to during collision avoidance.
     */
    geometry_msgs::PoseStamped int_pos;

    /**
     * @brief The intermediate target velocity which the CPS shall move at during collision avoidance.
     */
    geometry_msgs::Twist int_vel;

    /**
     * @brief Distance between CPSs below which the collision avoidance will work maximally.
     */
    double dist_critical;

    /**
     * @brief Distance between CPSs below which collision avoidance is active.
     */
    double dist_avoid;

    /**
     * @brief The shape of the repulsion function.
     */
    string repulsion_shape;
};

#endif // REPULSION_H
