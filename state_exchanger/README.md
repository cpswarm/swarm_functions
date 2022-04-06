# state_exchanger
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__state_exchanger__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__state_exchanger__ubuntu_xenial__source/)

This package exchanges behavioral states between multiple cyber physical systems (CPSs) in a swarm.

## Dependencies
This package depends on the following message definitions:
* [flexbe_msgs](https://wiki.ros.org/flexbe_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The state that is exchanged is read from a [FlexBE](https://wiki.ros.org/flexbe) state machine.

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)

## Execution
Run the launch file
```
roslaunch state_exchanger state_exchanger.launch
```
to launch the `state_exchanger` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `state_exchanger.yaml` that allows to configure the behavior of the `state_exchanger` node.

## Nodes

### state_exchanger
The `state_exchanger` node publishes behavioral states of this CPS to the rest of the swarm and publishes the state received from the other swarm members locally. The state is taken from a locally running FlexBE state machine. Each state is represented by a behavior ID, a checksum representing a specific version of a behavior.

#### Subscribed Topics
* `flexbe/status` ([flexbe_msgs/BEStatus](http://docs.ros.org/en/api/flexbe_msgs/html/msg/BEStatus.html))
  The checksum of the current state of this CPS.
* `bridge/events/state` ([cpswarm_msgs/StateEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/StateEvent.html))
  The current state of another CPS. Messages are exchanged between CPSs using the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

#### Published Topics
* `state` ([cpswarm_msgs/StateEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/StateEvent.html))
  The current state of this CPS that is forwarded by the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio) to the other swarm members.
* `swarm_state` ([cpswarm_msgs/ArrayOfStates](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfStates.html))
  The states of the other swarm members received through the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

#### Parameters
* `~loop_rate` (real, default: `1.5`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `10`)
  The size of the message queue used for publishing and subscribing to topics.
* `~timeout` (real, default: `20.0`)
  The time in seconds after which another CPS is considered to have left the swarm.

## Code API
[state_exchanger package code API documentation](https://cpswarm.github.io/swarm_functions/state_exchanger/docs/html/files.html)
