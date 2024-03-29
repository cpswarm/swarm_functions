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
     * @param dist_critical Distance between CPSs in meter below which the collision avoidance will work maximally, i.e., maximum repulsion, no attraction.
     * @param dist_attract Distance between CPSs in meter below which attraction starts to decrease. Must be greater than dist_critical.
     * @param dist_repulse Distance between CPSs in meter below which repulsion starts to increase. Must be greater than dist_critical.
     * @param attraction_shape The shape of the attraction function.
     * @param repulsion_shape The shape of the repulsion function.
     */
    void init (double dist_critical, double dist_attract, double dist_repulse, string attraction_shape, string repulsion_shape);

    /**
     * @brief Check whether collision avoidance is necessary and calculate respective position or velocity.
     * @return The number of CPSs that this CPS is avoiding collisions with. Returns zero if the class is not properly initialized.
     */
    int calc ();

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
     * @return A pose with the current position of the CPS and the direction away from the other CPSs.
     */
    geometry_msgs::Vector3 get_dir ();

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
     * Calculate the direction towards the original goal during collision avoidance.
     * @param attraction Returns the attraction vector, i.e., the direction of the goal with the magnitude calculated from the closest CPS distance.
     * @param closest The distance to the closest CPS.
     */
    void attract (geometry_msgs::Vector3& attraction, double closest);

    /**
     * @brief Calculate repulsion from close by CPSs.
     * @param repulsion Returns the repulsion vector, i.e., the sum of the vectors pointing away from the neighbors.
     * @param neighbors Returns the number of neighbors that create repulsion.
     * @param closest Returns the distance to the closest neighbor.
     */
    void repulse (geometry_msgs::Vector3& repulsion, int& neighbors, double& closest);

    /**
     * @brief The type of CPS control.
     */
    control_t setpoint;

    /**
     * @brief The current position of this CPS.
     */
    geometry_msgs::PoseStamped pos;

    /**
     * @brief Whether a position has been provided.
     */
    bool pos_valid;

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
     * @brief Distance between CPSs in meter below which the collision avoidance will work maximally, i.e., maximum repulsion, no attraction.
     */
    double dist_critical;

    /**
     * @brief Distance between CPSs in meter below which attraction starts to decrease. Must be greater than dist_critical.
     */
    double dist_attract;

    /**
     * @brief Distance between CPSs in meter below which repulsion starts to increase. Must be greater than dist_critical.
     */
    double dist_repulse;

    /**
     * @brief The shape of the attraction function.
     */
    string attraction_shape;

    /**
     * @brief The shape of the repulsion function.
     */
    string repulsion_shape;
};

#endif // REPULSION_H
