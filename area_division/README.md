# area_division
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__area_division__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__area_division__ubuntu_xenial__source/)

This package divides the available environment area among multiple cyber physical systems (CPSs) in a swarm. It is part of the swarm functions library.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [nav_msgs](https://wiki.ros.org/nav_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* area_provider
* *_pos_provider
* *_pos_controller

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)

## Execution
Run the launch file
```
roslaunch area_division area_division.launch
```
to launch the `area_division` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `area_division.yaml` that allows to configure the behavior of the `area_division` node.

## Nodes

### area_division
The `area_division` divides the environment area among multiple CPSs. When this node is running, it listens to area division requests by other CPSs to perform the area division. The division is also triggered when CPS join or leave the swarm or the behavior state of this CPS changes. Once the area division starts, all CPSs that participate in the area division, stop moving, and synchronize in order to achieve the same area division result. The division algorithm is based on the [DARP algorithm](https://github.com/athakapo/DARP) which tries to divide the area optimally. Each CPS is assigned an equal share of the environment that includes its current position.

#### Subscribed Topics
* `state` ([cpswarm_msgs/StateEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/StateEvent.html))
  The behavior state of this CPS.
* `swarm_state` ([cpswarm_msgs/ArrayOfStates](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfStates.html))
  The behaior states of the other CPSs.
* `pos_provider/pose` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The current position of this CPS.
* `area/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
  The map to be divided.
* `bridge/uuid` ([swarmros/String](https://cpswarm.github.io/swarmio/swarmros/msg/String.html))
  The UUID of this CPS.
* `bridge/events/area_division` ([cpswarm_msgs/AreaDivisionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/AreaDivisionEvent.html))
  The area division requests from other CPSs. Messages are exchanged between CPSs using the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

#### Published Topics
* `pos_controller/goal_position` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The topic for stoping the CPS.
* `area_division` ([cpswarm_msgs/AreaDivisionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/AreaDivisionEvent.html))
  The topic for requesting area division among the available CPSs in the swarm. The request is forwarded by the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio) to the other swarm members.
* `area/assigned` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
  The area assigned to this CPS.
* `area/rotated` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
  The area to be divided, rotated so the lower boundary is horizontal. For visualization purposes, only published if the parameter `visualize` is set to true.
* `area/downsampled` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
  The area to be divided, downsampled to a lower resolution. For visualization purposes, only published if the parameter `visualize` is set to true.

#### Services Called
* `area/get_rotation` ([cpswarm_msgs/GetDouble](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetDouble.html))
  Get the rotation required to align the lower boundary of the area horizontally.

#### Parameters
* `~loop_rate` (real, default: `1.5`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `10`)
  The size of the message queue used for publishing and subscribing to topics.
* `resolution` (real, default: `1.0`
  The grid map underlying the area division will be downsampled to this resolution in meter / cell.
* `~swarm_timeout` (real, default: `5.0`)
  The time in seconds communication in the swarm can be delayed at most. Used to wait after an area division event before starting the area division or time after which it is assumed that a swarm member has left the swarm if no position update has been received.
* `~visualize` (boolean, default: `false`)
  Whether to publish the area division on a topic for visualization.
* `~states` (string list, default: `[]`)
  Only CPSs in these states divide the area among each other.
* `~/optimizer/iterations` (integer, default: `10`)
  Maximum number of iterations of the optimization algorithm.
* `~/optimizer/variate_weight` (real, default: `0.01`)
  Maximum variate weight of connected components.
* `~/optimizer/discrepancy` (integer, default: `30`)
  Maximum difference between number of assigned grid cells to each CPS.

## Code API
[area_division package code API documentation](https://cpswarm.github.io/swarm_functions/area_division/docs/html/files.html)
