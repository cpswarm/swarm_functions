# kinematics_exchanger

This package exchanges kinematic properties such as velocity or position between multiple cyber physical systems (CPSs) in a swarm.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The following packages of the [sensing and actuation](https://github.com/cpswarm/sensing_actuation) library are required:
* *_pos_provider
* *_vel_provider

Further required packages are:
* [tf2](https://wiki.ros.org/tf2/)

## Execution
Run the launch file
```
roslaunch kinematics_exchanger kinematics_exchanger.launch
```
to launch the `kinematics_exchanger` node.

### Launch File Parameters
The launch file can be configured with following parameters:
* `id` (integer, default: 1)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: screen)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

### Parameter Files
In the `param` subdirectory there is the parameter file `kinematics_exchanger.yaml` that allows to configure the behavior of the `kinematics_exchanger` node. It contains the following parameters:
* `~loop_rate` (real, default: 1.5)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: 10)
  The size of the message queue used for publishing and subscribing to topics.
* `~timeout` (real, default: 20.0)
  The time in seconds after which another CPS is considered to have left the swarm.
* `~sample_size` (integer, default: 5)
  The number of data samples to average over for reliable results.
* `~init` (integer, default: 30)
  The number of messages to ignore during initialization. This is because the first messages are inaccurate.

## Nodes

### kinematics_exchanger
The `kinematics_exchanger` node publishes position and velocity of this CPS to the rest of the swarm and publishes the position and velocity received from the other swarm members locally. It publishes the swarm position both as relative and absolute coordinates. The swarm velocity is published only as relative coordinates.

#### Subscribed Topics
* `pos_provider/pose` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The current position of this CPS.
* `vel_provider/velocity` ([geometry_msgs/TwistStamped](https://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))
  The current velocity of this CPS.
* `bridge/events/position` ([cpswarm_msgs/Position](https://cpswarm.github.io/cpswarm_msgs/html/msg/Position.html))
  The current position of another CPS. Messages are exchanged between CPSs using the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).
* `bridge/events/velocity` ([cpswarm_msgs/Velocity](https://cpswarm.github.io/cpswarm_msgs/html/msg/Velocity.html))
  The current velocity of another CPS. Messages are exchanged between CPSs using the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

#### Published Topics
* `position` ([cpswarm_msgs/Position](https://cpswarm.github.io/cpswarm_msgs/html/msg/Position.html))
  The current position of this CPS that is forwarded by the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio) to the other swarm members.
* `velocity` ([cpswarm_msgs/Velocity](https://cpswarm.github.io/cpswarm_msgs/html/msg/Velocity.html))
  The current velocity of this CPS that is forwarded by the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio) to the other swarm members.
* `swarm_position` ([cpswarm_msgs/ArrayOfPositions](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfPositions.html))
  The positions of the other swarm members received through the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).
* `swarm_position_rel` ([cpswarm_msgs/ArrayOfVectors](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfVectors.html))
  The positions of the other swarm members received through the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio) relative to the position of this CPS.
* `swarm_velocity_rel` ([cpswarm_msgs/ArrayOfVectors](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfVectors.html))
  The velocities of the other swarm members received through the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio) relative to the velocity of this CPS.

## Code API
[kinematics_exchanger package code API documentation](https://cpswarm.github.io/swarm_functions/kinematics_exchanger/docs/html/files.html)
