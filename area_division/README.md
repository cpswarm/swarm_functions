# area_division

This package allows to divide the available environment area among multiple cyber physical systems (CPSs) in a swarm. It is part of the swarm functions library.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [nav_msgs](https://wiki.ros.org/nav_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The following packages of the [sensing and actuation](https://github.com/cpswarm/sensing_actuation) library are required:
* area_provider
* *_pos_provider
* *_pos_controller

## Execution
Run the launch file
```
roslaunch area_division area_division.launch
```
to launch the `area_division` node.

### Launch File Parameters
The launch file can be configured with following parameters:
* `id` (integer, default: 1)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: screen)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

### Parameter Files
In the `param` subdirectory there is the parameter file `area_division.yaml` that allows to configure the behavior of the `area_division` node. It contains the following parameters:
* `~loop_rate` (real, default: 1.5)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: 10)
  The size of the message queue used for publishing and subscribing to topics.
* `~swarm_timeout` (real, default: 5.0)
  The time in seconds to wait after an area division event before starting the area division.
* `~visualize` (boolean, default: false)
  Whether to publish the area division on a topic for visualization.
* `~/optimizer/iterations` (integer, default: 10)
  Maximum number of iterations of the optimization algorithm.
* `~/optimizer/variate_weight` (real, default: 0.01)
  Maximum variate weight of connected components.
* `~/optimizer/discrepancy` (integer, default: 30)
  Maximum difference between number of assigned grid cells to each CPS.

## Nodes

### area_division
The `area_division` divides the environment area among multiple CPSs. When this node is running, it listens to area division requests by other CPSs to perform the area division. The division can also be triggered locally, by calling the `area/assigned` service. Once the area division starts, all CPSs that participate in the area division, stop moving, and synchronize in order to achieve the same area division result. The division algorithm is based on the ([DARP algorithm](https://github.com/athakapo/DARP)) which tries to divide the area optimally. Each CPS is assigned an equal share of the environment that includes its current position.

#### Subscribed Topics
* `pos_provider/pose` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The current position of this CPS.
* `area_provider/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
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
* `assigned_map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
  The area assigned to this CPS for visualization purposes. Only published if the parameter `visualize` is set to true.

#### Services
* `area/assigned` ([nav_msgs/GetMap](http://docs.ros.org/api/nav_msgs/html/srv/GetMap.html))
  Provides the map of the area assigned to this CPS.

## Code API
[area_division package code API documentation](https://cpswarm.github.io/swarm_functions/area_division/docs/html/files.html)
