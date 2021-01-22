# collision_avoidance
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__collision_avoidance__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__collision_avoidance__ubuntu_xenial__source/)

This package enables cyber physical systems (CPSs) in a swarm to avoid collisions with each other.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions) are required:
* kinematics_exchanger

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* *_pos_provider (if position setpoints are used by the behavior algorithms)
* *_vel_provider
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
The `collision_avoidance` node constantly monitors for close by CPSs using position information received through the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio). In case there are CPSs closer than `equi_dist`, it calculates the repulsion from them as the sum of the pair potentials in terms of inverse distance. It then calculates a velocity with reduced magnitude into a direction that lets the CPS repulse from the other CPSs while maintaining its desired direction. In case that position setpoints are used, it converts the velocity into an appropriate position. The avoidance velocity and position setpoints are published to the respective controllers of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation).

#### Subscribed Topics
* `pos_controller/goal_position` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The originally desired goal position of this CPS in case of no collision avoidance.
* `vel_controller/target_velocity` ([geometry_msgs/Twist](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
  The originally desired target velocity of this CPS in case of no collision avoidance.
* `pos_provider/pose` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The current position of this CPS.
* `vel_provider/velocity` ([geometry_msgs/TwistStamped](https://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))
  The current velocity of this CPS.
* `swarm_position_rel` ([cpswarm_msgs/ArrayOfVectors](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfVectors.html))
  The relative positions of the other swarm members (distance and bearing).

#### Published Topics
* `pos_controller/ca_goal_position` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The intermediate goal position where the CPS shall move to during collision avoidance. Only published if position setpoints are used by the behavior algorithm.
* `vel_controller/ca_target_velocity` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
  The intermediate target velocity which the CPS shall move at during collision avoidance. Only published if velocity setpoints are used by the behavior algorithm.

#### Parameters
* `~loop_rate` (real, default: `1.5`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~repulsion/equi_dist` (real, default: `10.0`)
  Desired equilibrium distance between CPSs in meter.
* `~repulsion/repulse_spring` (real, default: `1.0`)
  Repulsion spring constant of half-spring per square second.
* `~repulsion/repulse_max` (real, default: `1.0`)
  Maximum repulsion between CPSs in meter in order to avoid over excitation.
* `~repulsion/avoid_vel` (real, default: `1.0`)
  Target velocity during collision avoidance in meter per second.
* `~repulsion/accel_time` (real, default: `1.0`)
  Characteristic time needed by the CPS to reach the target velocity in seconds.

## Code API
[collision_avoidance package code API documentation](https://cpswarm.github.io/swarm_functions/collision_avoidance/docs/html/files.html)
