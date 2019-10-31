# state_exchanger

This package exchanges behavioral states between multiple cyber physical systems (CPSs) in a swarm.

## Dependencies
This package depends on the following message definitions:
* [smach_msgs](https://wiki.ros.org/smach_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The state that is exchanged is read from a [SMACH](https://wiki.ros.org/smach) state machine.

## Execution
Run the launch file
```
roslaunch state_exchanger state_exchanger.launch
```
to launch the `state_exchanger` node.

### Launch File Parameters
The launch file can be configured with following parameters:
* `id` (integer, default: 1)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: screen)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

### Parameter Files
In the `param` subdirectory there is the parameter file `state_exchanger.yaml` that allows to configure the behavior of the `state_exchanger` node. It contains the following parameters:
* `~loop_rate` (real, default: 1.5)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: 10)
  The size of the message queue used for publishing and subscribing to topics.
* `~timeout` (real, default: 20.0)
  The time in seconds after which another CPS is considered to have left the swarm.
* `~sm_path` (string, default: /SM_TOP)
  The path of the smach state machine whose state shall be exchanged.

## Nodes

### state_exchanger
The `state_exchanger` node publishes behavioral states of this CPS to the rest of the swarm and publishes the state received from the other swarm members locally. The state is taken from a locally running SMACH state machine. If the CPS is in multiple states, only the first one is considered.

#### Subscribed Topics
* `smach_server/smach/container_status` ([smach_msgs/SmachContainerStatus](http://docs.ros.org/api/smach_msgs/html/msg/SmachContainerStatus.html))
  The current state of this CPS.
* `bridge/events/state` ([cpswarm_msgs/StateEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/StateEvent.html))
  The current state of another CPS. Messages are exchanged between CPSs using the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

#### Published Topics
* `state` ([cpswarm_msgs/StateEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/StateEvent.html))
  The current state of this CPS that is forwarded by the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio) to the other swarm members.
* `swarm_state` ([cpswarm_msgs/ArrayOfStates](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfStates.html))
  The states of the other swarm members received through the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

## Code API
[state_exchanger package code API documentation](https://cpswarm.github.io/swarm_functions/state_exchanger/docs/html/files.html)
