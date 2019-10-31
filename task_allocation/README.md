# task_allocation

This package offers action servers for assigning tasks between cyber physical system (CPS). It uses a market-inspired approach by running a single-round, single-item auction. The package offers two action servers that perform the auction, one that acts as auctioneer and one that acts as bidder. The auctioneer opens the auction for a specific duration in which interested bidders can place a single bid. After the timeout the winner, i.e., the bidder with the highest bid is announced.

## Dependencies
This package depends on the following message definitions:
* [actionlib_msgs](https://wiki.ros.org/actionlib_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The following packages of the [sensing and actuation](https://github.com/cpswarm/sensing_actuation) library are required:
* *_pos_provider

Further required packages are:
* [actionlib](https://wiki.ros.org/actionlib/)

## Execution
To start the action servers, run the launch file
```
roslaunch task_allocation task_allocation.launch
```
which launches both action servers which listen for incoming requests.

### Launch File Parameters
The launch file can be configured with following parameters:
* `id` (integer, default: 1)
  The identifier (ID) of the CPS that is running the servers.
* `output` (string, default: screen)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

### Parameter Files
In the `param` subdirectory there is the parameter file `task_allocation.yaml` that allows to configure the behavior of the auction process. It contains the following parameters:
* `~loop_rate` (real, default: 5.0)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: 10)
  The size of the message queue used for publishing and subscribing to topics.
* `~timeout` (real, default: 10.0)
  The time in seconds to listen for incoming bids from other CPSs after the auction has been opened, i.e., auction duration.

## Nodes

### auction_action
The `auction_action` node offers the `task_allocation_auction` action server that acts as auctioneer in the task allocation auction. When the action is called, it opens an auction and announces the task with ID and location. It then waits a specific time for the bids of other CPS. Once the auction timeout expires, it broadcasts the ID of the winning CPS, i.e., the one with the highest bid, to which the task is assigned. If no CPS participated in the auction, the action server aborts the auction goal.

#### Action API
The `auction_action` node provides an implementation of the SimpleActionServer to provide the task allocation auction process. It takes in goals containing cpswarm_msgs/TaskAllocation messages.

##### Action Subscribed Topics
* `cmd/task_allocation_auction/goal` ([cpswarm_msgs/TaskAllocationActionGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/TaskAllocation.html))
  A goal to start an auction containing the universally unique ID (UUID) of the auctioneer together with the ID and the position of the task that is auctioned.
* `cmd/task_allocation_auction/cancel` ([actionlib_msgs/GoalID](https://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))
  A request to cancel a specific auction goal.

##### Action Published Topics
* `cmd/task_allocation_auction/feedback` ([cpswarm_msgs/TaskAllocationActionFeedback](https://cpswarm.github.io/cpswarm_msgs/html/action/TaskAllocation.html))
  The feedback is empty for the `task_allocation_auction` action.
* `cmd/task_allocation_auction/status` ([actionlib_msgs/GoalStatusArray](https://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))
  Provides status information on the goals that are sent to the `task_allocation_auction` action.
* `cmd/task_allocation_auction/result` ([cpswarm_msgs/TaskAllocationActionResult](https://cpswarm.github.io/cpswarm_msgs/html/action/TaskAllocation.html))
  The result of the auction contains the UUID of the winning CPS, together with ID and position of the task that has been auctioned.

#### Subscribed Topics
* `bridge/events/cps_selection` ([cpswarm_msgs/TaskAllocationEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TaskAllocationEvent.html))
  The the bids of the auction from other CPSs. Messages are exchanged between CPSs using the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

#### Published Topics
* `cps_selected` ([cpswarm_msgs/TaskAllocatedEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TaskAllocatedEvent.html))
  The UUID of the CPS that won the auction to the other CPSs. Messages are exchanged between CPSs using the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

### bid_action
The `bid_action` node offers the `task_allocation_bid` action server that acts as bidder in the task allocation auction opened by another CPS. When the action is called, it computes a bid for the task based on its location. The bid value is inversely proportional to the distance between the CPS and the task. It publishes the bid and waits until the auction ends. If the CPS has won the auction, the action server goal succeeds, otherwise it is aborted.

#### Action API
The `bid_action` node provides an implementation of the SimpleActionServer to provide the task allocation auction process. It takes in goals containing cpswarm_msgs/TaskAllocation messages.

##### Action Subscribed Topics
* `cmd/task_allocation_bid/goal` ([cpswarm_msgs/TaskAllocationActionGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/TaskAllocation.html))
  A goal to compute a bid containing the universally unique ID (UUID) of the auctioneer together with the ID and the position of the task that is auctioned.
* `cmd/task_allocation_bid/cancel` ([actionlib_msgs/GoalID](https://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))
  A request to cancel a specific bidding goal.

##### Action Published Topics
* `cmd/task_allocation_bid/feedback` ([cpswarm_msgs/TaskAllocationActionFeedback](https://cpswarm.github.io/cpswarm_msgs/html/action/TaskAllocation.html))
  The feedback is empty for the `task_allocation_bid` action.
* `cmd/task_allocation_bid/status` ([actionlib_msgs/GoalStatusArray](https://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))
  Provides status information on the goals that are sent to the `task_allocation_bid` action.
* `cmd/task_allocation_bid/result` ([cpswarm_msgs/TaskAllocationActionResult](https://cpswarm.github.io/cpswarm_msgs/html/action/TaskAllocation.html))
  The result of the auction contains ID and position of the task that has been auctioned.

#### Subscribed Topics
* `pos_provider/pose` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The current position of the CPS.
* `bridge/uuid` ([swarmros/String](https://cpswarm.github.io/swarmio/swarmros/msg/String.html))
  The UUID of this CPS. It is published by the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).
* `bridge/events/cps_selected` ([cpswarm_msgs/TaskAllocatedEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TaskAllocatedEvent.html))
  The UUID of the CPS that won the auction. Messages are exchanged between CPSs using the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

#### Published Topics
* `cps_selection` ([cpswarm_msgs/TaskAllocationEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TaskAllocationEvent.html))
  The auction bid submitted to the auctioneer CPS. Messages are exchanged between CPSs using the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

## Code API
[task_allocation package code API documentation](https://cpswarm.github.io/swarm_functions/task_allocation/docs/html/files.html)
