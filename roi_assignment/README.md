# roi_assignment

This package centrally assigns regions of interest (ROIs) to multiple cyber physical system (CPSs) without central instance in distributed manner. It is part of the swarm functions library.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication to CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* area_provider

The following packages of the central functions library are required:
* roi_division_cent

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions) are required:
* kinematics_exchanger
* state_exchanger

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)
* [actionlib](https://wiki.ros.org/actionlib/)

## Execution
Run the launch file
```
roslaunch roi_assignment roi_assignment.launch
```
to launch the `roi_assignment` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `log`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

  In the `param` subdirectory there is the parameter file `roi_assignment.yaml` that allows to configure the behavior of the ROI assignment with further parameters.

## Nodes

### roi_assignment
The `roi_assignment` node provides an action server that assigns all available ROIs to a set of CPSs. The ROIs are retrieved from the `area_provider` package. The set of CPSs is determined by communication through the CPSwarm communication library, specifically using the `kinematics_exchanger` and `state_exchanger` packages. The assignment procedure can follow different criteria, currently implemented are the following:
* Simple assignment: Each CPS is assigned to the spatially closest ROI. If multiple CPSs are assigned to the same ROI, the ROI is divided among them.

#### Subscribed Topics
* `swarm_position` ([cpswarm_msgs/ArrayOfPositions](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfPositions.html))
  The positions of all CPSs in the swarm. They allow assignment based on spatial metrics.
* `swarm_state` ([cpswarm_msgs/ArrayOfStates](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfStates.html))
  The behavior states of all CPSs in the swarm. They allow assignment based on the behavior executed by the different CPSs.

#### Published Topics
* `rois/assignment` ([cpswarm_msgs/PointArrayEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/PointArrayEvent.html))
  The division result. It is an event message that can be forward through the CPSwarm communication library. For each CPS, a separate message with a vector of coordinates is published.

#### Services Called
* `rois/get_all` ([cpswarm_msgs/GetMultiPoints](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetMultiPoints.html))
  Retrieve all ROIs for assignment.

#### Action Servers
* `rois/assign` ([cpswarm_msgs/RoiAssignmentAction](https://cpswarm.github.io/cpswarm_msgs/html/action/RoiAssignmentAction.html))
  The action server that implements a [simple action client](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionClient.html). Both goal and result are empty. The CPSs and ROIs are retrieved independently by the `roi_assignment` node. The result is published through a topic so it can be forwarded through the CPSwarm communication library. The progress of the division is provided as feedback of the action server.

#### Actions Clients
* `rois/divide` ([cpswarm_msgs/RoiDivisionAction](https://cpswarm.github.io/cpswarm_msgs/html/action/RoiDivisionAction.html))
  Divide ROIs among multiple CPSs.

#### Parameters
* `~loop_rate` (real, default: `1.5`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~states` (string list, default: `[]`)
  A list that allows to specify the behavior states in which CPSs are considered for the assignment process.

## Code API
[roi_assignment package code API documentation](https://cpswarm.github.io/central_functions/roi_assignment/docs/html/files.html)
