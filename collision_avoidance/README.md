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
The `collision_avoidance` node constantly monitors for close by CPSs using position information received through the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio). In case there are CPSs closer than `dist_avoid`, it calculates the repulsion from each CPS as a vector pointing away from the other CPS. There are three different repulsion functions to calculate the vector magnitude in the range `[0,1]`:
* `linear`: The repulsion increases linearly, inverse to the distance between the CPSs.
* `sine`: The repulsion increases following the sine function with decreasing CPS distance.
* `exp`: The repulsion increases following the exponential function with decreasing CPS distance.

The repulsion magnitude is maximal for CPSs closer than `dist_critical`. The overall repulsion is the sum of the repulsion from each CPS. To calculate the resulting avoidance direction, the desired target direction of the CPS is added to the repulsion vector. To make CPSs move slower when close to each other, the magnitude of the avoidance vector follows a linear function that increases with the distance to the closest CPS: It is minimal with `dist_critical/2` at distance `dist_critical` and maximal with `dist_avoid/2` at distance `dist_avoid`.

For velocity setpoints, the generated avoidance vector represents the velocity. For position setpoints, the vector is added to the current CPS's position to get the new position. The avoidance velocity and position setpoints are published to the respective controllers of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation).

A limitation of this approach is that there can be deadlocks when repulsion and goal directions cancel each other.

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
* `~dist_critical` (real, default: `1.0`)
   Distance between CPSs in meter below which the collision avoidance will work maximally.
* `~dist_avoid` (real, default: `3.0`)
  Distance between CPSs in meter below which collision avoidance is active.
* `~visualize` (boolean, default: `false`)
  Whether to visualize the avoidance direction by publishing on a ROS topic.
* `~repulsion_shape` (string, default: `linear`)
  The shape of the repulsion function, can be sine, exp, or linear.
* `~excluded` (string list, default: `[]`)
  No collision avoidance in these states.

## Code API
[collision_avoidance package code API documentation](https://cpswarm.github.io/swarm_functions/collision_avoidance/docs/html/files.html)
