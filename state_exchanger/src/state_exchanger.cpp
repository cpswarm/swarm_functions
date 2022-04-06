#include "state_exchanger.h"

/**
 * @brief Callback function for state updates.
 * @param msg State received from the CPS state machine.
 */
void state_callback (const std_msgs::String::ConstPtr& msg)
{
    // state received
    state_valid = true;

    // store state name
    state = msg->data;

    ROS_DEBUG("Started behavior %s", state.c_str());
}

/**
 * @brief Callback function for state updates from other swarm members.
 * @param msg The state received from another CPS.
 */
void swarm_state_callback (cpswarm_msgs::StateEvent msg) {

    // uuid of the sending swarm member
    string uuid = msg.swarmio.node;

    // add new swarm member
    if (swarm_state.count(uuid) <= 0) {
        state_t member_state;
        member_state.uuid = uuid;
        swarm_state.emplace(uuid, member_state);
    }

    // update swarm member
    swarm_state[uuid].state = msg.state.c_str();
    swarm_state[uuid].stamp = Time::now();
}

/**
 * @brief A ROS node that exchanges the behavioral state between CPSs in a swarm.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char **argv)
{
    // init ros node
    init(argc, argv, "state_exchanger");
    NodeHandle nh;

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    double timeout;
    nh.param(this_node::getName() + "/timeout", timeout, 20.0);
    nh.param(this_node::getName() + "/read_only", read_only, false);

    // init topic flags
    state_valid = false;

    // publishers and subscribers
    Subscriber state_subscriber;
    if (read_only == false)
        state_subscriber = nh.subscribe("flexbe/behavior_update", queue_size, state_callback);
    Subscriber incoming_state_subscriber = nh.subscribe("bridge/events/state", queue_size, swarm_state_callback);
    Publisher outgoing_state_publisher;
    if (read_only == false)
        outgoing_state_publisher = nh.advertise<cpswarm_msgs::StateEvent>("state", queue_size);
    Publisher incoming_state_publisher = nh.advertise<cpswarm_msgs::ArrayOfStates>("swarm_state", queue_size);

    // init loop rate
    Rate rate(loop_rate);

    // init behavior state
    if (read_only == false) {
        while (ok() && state_valid == false) {
            ROS_DEBUG_ONCE("Waiting for valid state...");
            rate.sleep();
            spinOnce();
        }
    }

    // init swarm state message
    cpswarm_msgs::ArrayOfStates swarm_state_msg;

    // continuously exchange state between swarm members
    while (ok()) {
        // reset swarm state message
        swarm_state_msg.states.clear();

        // update swarm state
        for (auto member=swarm_state.begin(); member!=swarm_state.end();) {
            // delete members that haven't updated their state lately
            if ((Time::now() - member->second.stamp) > Duration(timeout)) {
                member = swarm_state.erase(member);
                continue;
            }

            // store state of swarm member
            cpswarm_msgs::StateEvent state_event;
            state_event.header.stamp = Time::now();
            state_event.swarmio.node = member->first;
            state_event.state = member->second.state;
            swarm_state_msg.states.push_back(state_event);

            // next member
            ++member;
        }

        // publish swarm state locally
        incoming_state_publisher.publish(swarm_state_msg);

        // publish local state to swarm
        if (read_only == false) {
            cpswarm_msgs::StateEvent state_event;
            state_event.header.stamp = Time::now();
            state_event.swarmio.name = "state";
            state_event.state = state;
            outgoing_state_publisher.publish(state_event);
        }

        rate.sleep();
        spinOnce();
    }
}
