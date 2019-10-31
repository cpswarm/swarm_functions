# coverage_path

This package generates an optimal path to cover a given area with a cyber physical system (CPS).

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [nav_msgs](https://wiki.ros.org/nav_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The following packages of the [swarm functions](https://github.com/cpswarm/swarm_functions) library are required:
* area_division
* state_exchanger

The following packages of the [sensing and actuation](https://github.com/cpswarm/sensing_actuation) library are required:
* *_pos_provider

Further required packages are:
* [tf2](https://wiki.ros.org/tf2/)

## Execution
Run the launch file
```
roslaunch coverage_path coverage_path.launch
```
to launch the `coverage_path` node.

### Launch File Parameters
The launch file can be configured with following parameters:
* `id` (integer, default: 1)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: screen)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

### Parameter Files
In the `param` subdirectory there is the parameter file `coverage_path.yaml` that allows to configure the behavior of the `coverage_path` node. It contains the following parameters:
* `~loop_rate` (real, default: 1.5)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: 1)
  The size of the message queue used for publishing and subscribing to topics.
* `~swarm_timeout` (real, default: 5.0)
  Time in seconds after which it is assumed that a swarm member has left the swarm if no position update has been received.
* `~visualize` (boolean, default: false)
  Whether to publish the coverage path on a topic for visualization.

## Nodes

### coverage_path
The `coverage_path` node generates a path that allows a CPS to cover a given area. It uses the area assigned by the area_division package. The generated coverage path is based on a [minimum spanning tree](https://en.wikipedia.org/wiki/Minimum_spanning_tree) to optimally sweep the area. The path is regenerated when CPSs with the same behavior state join or leave the swarm. A regeneration of the path also triggers a new area division among the CPSs. The path can be retrieved either as a whole or waypoint after waypoint. In the latter case, the current waypoint is returned, based on the current position of the CPS.

#### Subscribed Topics
* `pos_provider/pose` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The current position of this CPS.
* `state` ([cpswarm_msgs/StateEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/StateEvent.html))
  The current behavior state of this CPS.
* `swarm_state` ([cpswarm_msgs/ArrayOfStates](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfStates.html))
  The current behavior states of other CPSs in the swarm.

#### Published Topics
* `coverage_path/path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
  The generated path for visualization purposes. Only published if the parameter `visualize` is set to true.
* `coverage_path/waypoint` ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))
  The current waypoint of the generated path for visualization purposes. Only published if the parameter `visualize` is set to true.

#### Services
* `coverage_path/path` ([nav_msgs/GetPlan](http://docs.ros.org/api/nav_msgs/html/srv/GetPlan.html))
  Provides the complete generated path.
* `coverage_path/waypoint` ([cpswarm_msgs/GetWaypoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/path/GetWaypoint.html))
  Provides the current waypoint of the generated path.

#### Services Called
* `area/assigned` ([nav_msgs/GetMap](http://docs.ros.org/api/nav_msgs/html/srv/GetMap.html))
  Get the map of the area assigned to this CPS.

## Code API
[coverage_path package code API documentation](https://cpswarm.github.io/swarm_functions/coverage_path/docs/html/files.html)
