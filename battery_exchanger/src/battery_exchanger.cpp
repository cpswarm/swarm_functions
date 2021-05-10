#include "battery_exchanger.h"

/**
 * @brief Callback function for battery state updates.
 * @param msg Battery state received from the battery monitor.
 */
void battery_callback (const cpswarm_msgs::BatteryState::ConstPtr& msg)
{
    // battery state received
    battery_valid = true;

    // store battery state in class variables
    battery = *msg;
}

/**
 * @brief Callback function for battery state updates from other swarm members.
 * @param msg The battery state received from another CPS.
 */
void swarm_battery_callback (cpswarm_msgs::BatteryStateEvent msg) {

    // uuid of the sending swarm member
    string uuid = msg.swarmio.node;

    // add new swarm member
    if (swarm_battery.count(uuid) <= 0) {
        battery_t member_battery;
        member_battery.uuid = uuid;
        swarm_battery.emplace(uuid, member_battery);
    }

    // update swarm member
    swarm_battery[uuid].state = msg.state;
    swarm_battery[uuid].stamp = Time::now();
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

    // init topic flags
    battery_valid = false;

    // publishers and subscribers
    Subscriber battery_subscriber = nh.subscribe("battery", queue_size, battery_callback);
    Subscriber incoming_battery_subscriber = nh.subscribe("bridge/events/battery", queue_size, swarm_battery_callback);
    Publisher outgoing_battery_publisher = nh.advertise<cpswarm_msgs::BatteryStateEvent>("battery_event", queue_size);
    Publisher incoming_battery_publisher = nh.advertise<cpswarm_msgs::ArrayOfBatteries>("swarm_battery", queue_size);

    // init loop rate
    Rate rate(loop_rate);

    // init battery state
    while (ok() && battery_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid state...");
        rate.sleep();
        spinOnce();
    }

    // init swarm battery message
    cpswarm_msgs::ArrayOfBatteries swarm_battery_msg;

    // continuously exchange state between swarm members
    while (ok()) {
        // reset swarm state message
        swarm_battery_msg.states.clear();

        // update swarm battery state
        for (auto member=swarm_battery.begin(); member!=swarm_battery.end();) {
            // delete members that haven't updated their battery state lately
            if ((Time::now() - member->second.stamp) > Duration(timeout)) {
                member = swarm_battery.erase(member);
                continue;
            }

            // store battery state of swarm member
            cpswarm_msgs::BatteryStateEvent battery_event;
            battery_event.swarmio.node = member->first;
            battery_event.state = member->second.state;
            battery_event.state.header.stamp = Time::now();
            swarm_battery_msg.states.push_back(battery_event);

            // next member
            ++member;
        }

        // publish swarm battery state locally
        incoming_battery_publisher.publish(swarm_battery_msg);

        // publish local battery state to swarm
        cpswarm_msgs::BatteryStateEvent battery_event;
        battery_event.swarmio.name = "battery";
        battery_event.state = battery;
        battery_event.state.header.stamp = Time::now();
        outgoing_battery_publisher.publish(battery_event);

        rate.sleep();
        spinOnce();
    }
}
