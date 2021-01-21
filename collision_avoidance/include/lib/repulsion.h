#ifndef REPULSION_H
#define REPULSION_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
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
 * @brief A class that generates a minimum-spanning-tree (MST) graph for a given grid map.
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
     */
    init (double equi_dist, double repulse_spring, double repulse_max, double accel_time);

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
    void set_goal_pos (const geometry_msgs::PoseStamped::ConstPtr& pos);

    /**
     * @brief Set the original target velocity for this CPS.
     * @param vel The desired target velocity of this CPS in case of no collision avoidance.
     */
    void set_target_vel (const geometry_msgs::Twist::ConstPtr& vel);

    /**
     * @brief Set the relative positions received from other CPSs in the swarm.
     * @param swarm An array of distance and bearing of the other CPSs.
     */
    void set_swarm (const cpswarm_msgs::ArrayOfVectors::ConstPtr& swarm);

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
     * @brief Calculate repulsive force from close by CPSs as pair potentials.
     * @return The required acceleration to reach the equilibrium distance to all close by neighbors.
     */
    geometry_msgs::Vector3 repulse ();

    /**
     * @brief The type of CPS control.
     */
    control_t setpoint;

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
     * @brief The intermediate goal position where the CPS shall move to during collision avoidance.
     */
    geometry_msgs::PoseStamped int_pos;

    /**
     * @brief The intermediate target velocity which the CPS shall move at during collision avoidance.
     */
    geometry_msgs::Twist int_vel;

    /**
     * @brief Desired equilibrium distance between CPSs.
     */
    double equi_dist;

    /**
     * @brief Repulsion spring constant of half-spring.
     */
    double repulse_spring;

    /**
     * @brief Maximum repulsion between CPSs.
     */
    double repulse_max;

    /**
     * @brief Characteristic time needed by the CPS to reach the target velocity.
     */
    double accel_time;
};

#endif // REPULSION_H
