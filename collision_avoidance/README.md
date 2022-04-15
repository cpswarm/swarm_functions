# collision_avoidance
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__collision_avoidance__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__collision_avoidance__ubuntu_xenial__source/)

This package enables cyber physical systems (CPSs) in a swarm to avoid collisions with each other.

## Dependencies
This package depends on the following message definitions:
* [std_msgs](https://wiki.ros.org/std_msgs)
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The currently running behavior is determined from [FlexBE](https://wiki.ros.org/flexbe/). It requires the patch described in [this pull request](https://github.com/team-vigir/flexbe_behavior_engine/pull/161).

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions) are required:
* kinematics_exchanger

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* *_pos_provider
* *_pos_controller (if position setpoints are used by the behavior algorithms)
* *_vel_controller (if velocity setpoints are used by the behavior algorithms)

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)
* [tf2](https://wiki.ros.org/tf2/)

## Execution
Run the launch file
```
roslaunch collision_avoidance collision_avoidance.launch
```
to launch the `collision_avoidance` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `collision_avoidance.yaml` that allows to configure the behavior of the `collision_avoidance` node.

## Nodes

### collision_avoidance
The `collision_avoidance` node constantly monitors for close by CPSs using position information received through the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio). In case there are CPSs closer than `dist_repulse`, it calculates the repulsion from each CPS as a vector pointing away from the other CPS. There are five different repulsion functions to calculate the vector magnitude in the range `[0,1]`:
* `lin`: The repulsion increases linearly, inverse to the distance between the CPSs.
* `li2`: The repulsion increases linearly, inverse to the distance between the CPSs, twice as fast as the linear function above.
* `sin`: The repulsion increases following the sine function with decreasing CPS distance.
* `log`: The repulsion increases following the natural logarithm with decreasing CPS distance.
* `exp`: The repulsion increases following the exponential function with decreasing CPS distance.

The repulsion magnitude is maximal, i.e. `1`, for CPSs closer than `dist_critical`. The overall repulsion is the sum of the repulsion from each CPS yielding a repulsion magnitude in the range `[0,number of neighbors closer than dist_repulse]`.

To calculate the resulting avoidance direction, the goal attraction of the CPS is added to the repulsion vector. It is a vector pointing to the navigation goal with the magnitude decreasing as function of the closest CPS when the distance to the closest CPS is below `dist_attract`. There are five different attraction functions to calculate the vector magnitude in the range `[0,1]`:
* `lin`: The attraction decreases linearly with the distance to the closest CPS.
* `li2`: The attraction decreases linearly with the distance to the closest CPS twice as fast as the linear function above.
* `sin`: The attraction decreases following the sine function with decreasing CPS distance.
* `log`: The attraction decreases following the natural logarithm with decreasing CPS distance.
* `exp`: The attraction decreases following the exponential function with decreasing CPS distance.

The repulsion magnitude is minimal, i.e. `0`, if the closest CPS is closer than `dist_critical`.

To make CPSs move slower when close to each other, the magnitude of the avoidance vector follows a linear function that increases with the distance to the closest CPS: It is minimal with `dist_critical/2` at distance `dist_critical` and maximal with `dist_repulse/2` at distance `dist_repulse`.

For velocity setpoints, the generated avoidance vector represents the velocity. For position setpoints, the vector is added to the current CPS's position to get the new position. The avoidance velocity and position setpoints are published to the respective controllers of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation).

A limitation of this approach is that there can be deadlocks when repulsion and attraction cancel each other.

#### Subscribed Topics
* `pos_controller/goal_position` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The originally desired goal position of this CPS in case of no collision avoidance.
* `vel_controller/target_velocity` ([geometry_msgs/Twist](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
  The originally desired target velocity of this CPS in case of no collision avoidance.
* `pos_provider/pose` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The current position of this CPS.
* `swarm_position_rel` ([cpswarm_msgs/ArrayOfVectors](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfVectors.html))
  The relative positions of the other swarm members (distance and bearing).
* `flexbe/behavior_update` ([std_msgs/String](https://docs.ros.org/api/std_msgs/html/msg/String.html))
  The current behavior state of this CPS.

#### Published Topics
* `pos_controller/ca_goal_position` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The intermediate goal position where the CPS shall move to during collision avoidance. Only published if position setpoints are used by the behavior algorithm.
* `vel_controller/ca_target_velocity` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
  The intermediate target velocity which the CPS shall move at during collision avoidance. Only published if velocity setpoints are used by the behavior algorithm.
* `collision_avoidance/direction` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  Direction of collision avoidance. Current position with orientation into avoidance direction. Only published if `visualize` is `true`.

#### Parameters
* `~loop_rate` (real, default: `1.5`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~visualize` (boolean, default: `false`)
  Whether to visualize the avoidance direction by publishing on a ROS topic.
* `~dist_critical` (real, default: `3.0`)
   Distance between CPSs in meter below which the collision avoidance will work maximally, i.e., maximum repulsion, no attraction.
* `~dist_attract` (real, default: `6.0`)
  Distance between CPSs in meter below which attraction starts to decrease. Must be greater than dist_critical.
* `~dist_repulse` (real, default: `12.0`)
  Distance between CPSs in meter below which repulsion starts to increase. Must be greater than dist_critical.
* `~attraction_shape` (string)
  The shape of the attraction function. Function shapes can be lin, li2, sin, log, or exp. Default is a constant value.
* `~repulsion_shape` (string)
  The shape of the repulsion function. Function shapes can be lin, li2, sin, log, or exp. Default is a constant value.
* `~excluded` (string list, default: `[]`)
  No collision avoidance in these states.

## Code API
[collision_avoidance package code API documentation](https://cpswarm.github.io/swarm_functions/collision_avoidance/docs/html/files.html)
