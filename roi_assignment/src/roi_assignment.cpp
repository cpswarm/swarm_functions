#include "roi_assignment.h"

/**
 * @brief Create a two-dimensional vector from a vector that has been flattened for transmission purposes and calculate the distance from the current position to the coordinates.
 * @param layout The layout of the original structure.
 * @param array_flat The flattened vector.
 * @throws invalid_argument if the given array layout does not specify two dimensions.
 * @throws runtime_error if the distance to a ROIs could not be retrieved.
 * @return A two-dimensional vector of geometry_msgs/Point messages together with the distance.
 */
vector<pair<double,vector<geometry_msgs::Point>>> expand (std_msgs::MultiArrayLayout layout, vector<geometry_msgs::Point> array_flat)
{
    NodeHandle nh;

    // check if given array is two-dimensional
    if (layout.dim.size() != 2)
        throw invalid_argument("Invalid array dimensions");

    // service client to get distance
    ServiceClient get_dist_client = nh.serviceClient<cpswarm_msgs::GetDist>("rois/get_distance");
    ROS_DEBUG("Waiting for ROI distance service...");
    get_dist_client.waitForExistence();
    ROS_DEBUG("ROI distance service available");
    cpswarm_msgs::GetDist gd;

    // empty two-dimensional array
    vector<pair<double,vector<geometry_msgs::Point>>> array_2d;

    // iterate first dimension
    for (int i=0; i<layout.dim[0].size; ++i) {
        // empty one-dimensional array as part of the two-dimensional array
        vector<geometry_msgs::Point> array_1d;

        // iterate second dimension
        for (int j=0; j<layout.dim[1].size; ++j) {
            // current coordinate
            geometry_msgs::Point coord = array_flat[layout.data_offset + i*layout.dim[1].stride + j];

            // skip empty padding elements at end
            if (array_1d.size() > 0 && coord.x == 0 && coord.y == 0)
                break;

            // add coordinate to one-dimensional array
            array_1d.push_back(coord);
        }

        // get distance
        gd.request.coords = array_1d;
        gd.request.point = position.pose.position;
        if (get_dist_client.call(gd)) {
            // add one-dimensional array two two-dimensional array
            array_2d.emplace_back(gd.response.distance, array_1d);
        }

        // exit if rois could not be retrieved
        else
            throw runtime_error("Failed to call service 'rois/get_distance'");
        }

    // return restored two-dimensional array
    return array_2d;
}

/**
 * @brief Inform the other CPSs about the beginning of an auction.
 */
void broadcast_auction ()
{
    // create auction message
    cpswarm_msgs::TaskAllocationEvent auction;
    auction.header.stamp = Time::now();
    auction.swarmio.name = "roi_assignment_auction";
    auction.id = auct->get_running().roi;
    auction.bid = auct->get_running().bid;

    // publish auction
    auction_pub.publish(auction);
}

/**
 * @brief Inform the other CPSs about the ROI that this CPS selected.
 */
void broadcast_assignment ()
{
    // create assignment message
    cpswarm_msgs::TaskAllocatedEvent assignment;
    assignment.header.stamp = Time::now();
    assignment.swarmio.name = "roi_assignment_result";
    assignment.task_id = auct->get_assigned();
    assignment.cps_id = uuid;

    // publish assignment
    result_pub.publish(assignment);
}

/**
 * @brief Send a bid to another CPS in order to participate in an auction.
 * @param roi The ID of the ROI being assigned/auctioned.
 * @param auctioneer The UUID of the CPS acting as auctioneer.
 * @param bid_value The value of the bid placed by this CPS.
 */
void send_bid (string roi, string auctioneer, double bid_value)
{
    // create bid message
    cpswarm_msgs::TaskAllocationEvent bid;
    bid.header.stamp = Time::now();
    bid.swarmio.name = "roi_assignment_bid";
    bid.swarmio.node = auctioneer;
    bid.id = roi;
    bid.bid = bid_value;

    // publish bid
    bid_pub.publish(bid);
}

/**
 * @brief Callback function to receive an auction initiated by another CPS.
 * @param msg The auction consisting of ROI ID and bid.
 */
void auction_cb (const cpswarm_msgs::TaskAllocationEvent::ConstPtr& msg)
{
    // ignore my own messages
    if (msg->swarmio.node == "")
        return;

    try {
        // calculate bid
        double bid = rois.bid(msg->id);

        // only send bid if it is higher
        if (bid > msg->bid) {
            send_bid(msg->id, msg->swarmio.node, bid);

            ROS_DEBUG("Place bid %.2f in auction for ROI %s initiated by %s", bid, msg->id.c_str(), msg->swarmio.node.c_str());
        }
    }
    catch (const exception& e) {
        ROS_ERROR("ROI assignment error: Could not participate in auction: %s", e.what());
    }
}

/**
 * @brief Callback function to receive bids for the auction held by this CPS.
 * @param msg The bid together with some meta data.
 */
void bid_cb (const cpswarm_msgs::TaskAllocationEvent::ConstPtr& msg)
{
    ROS_ERROR("Received bid %.2f in auction for ROI %s from %s", msg->bid, msg->id.c_str(), msg->swarmio.node.c_str());

    // ignore my own messages
    if (msg->swarmio.node == "")
        return;

    try {
        auct->participant(msg->id, msg->swarmio.node, msg->bid);

        ROS_DEBUG("Received bid %.2f in auction for ROI %s from %s", msg->bid, msg->id.c_str(), msg->swarmio.node.c_str());
    }
    catch (const exception& e) {
        ROS_WARN("ROI assignment warning: Invalid bid received: %s", e.what());
    }
}

/**
 * @brief Callback function to receive the result of an auction held by another CPS.
 * @param msg The assigned CPS together with some meta data.
 */
void result_cb (const cpswarm_msgs::TaskAllocatedEvent::ConstPtr& msg)
{
    // ignore my own messages
    if (msg->swarmio.node == "")
        return;

    try {
        // store result of assignment (cpss don't necessarily choose a roi when they win an auction)
        if (msg->swarmio.node == msg->cps_id) // only results assigned by cpss to themselves mean they will go to the roi
            rois.add(msg->task_id, msg->cps_id);

        ROS_DEBUG("ROI %s assigned to %s", msg->task_id.c_str(), msg->cps_id.c_str());
    }
    catch (const exception& e) {
        ROS_ERROR("ROI assignment error: Invalid assignment result received: %s", e.what());
    }
}

/**
 * @brief Callback function to receive the position of this CPS.
 * @param msg A pose with meta data.
 */
void pos_cb (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    position = *msg;
}

/**
 * @brief Callback function to receive the UUID of this CPS.
 * @param msg A string containing the UUID.
 */
void uuid_cb (const swarmros::String::ConstPtr& msg)
{
    uuid = msg->value;
}

/**
 * @brief Callback to execute the action server that assigns ROIs.
 * @param goal An empty action server goal.
 * @param as The action server object.
 */
void roi_assignment (const cpswarm_msgs::RoiAssignmentGoal::ConstPtr& goal, AssignmentAction* assignment_server)
{
    NodeHandle nh;

    // read parameters
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    double timeout;
    nh.param(this_node::getName() + "/timeout", timeout, 5.0);
    Rate rate(timeout * 10);

    // TODO: updated rois without overwriting previous assignment results

    // ros communication
    auction_pub = nh.advertise<cpswarm_msgs::TaskAllocationEvent>("rois/assignment/auction", queue_size); // disseminate auctions
    result_pub = nh.advertise<cpswarm_msgs::TaskAllocatedEvent>("rois/assignment/result", queue_size); // disseminate auction result
    Subscriber bid_sub = nh.subscribe("rois/assignment/bid", queue_size, bid_cb); // listen to bids

    // try acquiring a roi until succeeded
    do {
        // select a roi
        auction_roi selected;
        try {
            selected = rois.select(); // TODO: select next cheapest each iteration
        }
        catch (const exception& e) {
            ROS_FATAL("ROI assignment failure: Could not select a ROI: %s", e.what());
            assignment_server->setAborted();
            return;
        }

        // start auction
        try {
            auct->initiate(selected.get_id(), rois.bid(selected.get_id()), Duration(timeout));

            ROS_DEBUG("Start auction for ROI %s with bid %.2f", selected.get_id().c_str(), auct->get_running().bid);

            // inform swarm about auction
            try {
                broadcast_auction();
            }
            catch (const exception& e) {
                ROS_ERROR("ROI assignment error: Failed to broadcast auction for ROI %s: %s", selected.get_id().c_str(), e.what());
                continue;
            }
        }
        catch (const exception& e) {
            ROS_ERROR("ROI assignment error: Failed to start auction for ROI %s: %s", selected.get_id().c_str(), e.what());
            continue;
        }

        // wait for bids
        while (ok() && auct->is_running() && assignment_server->isPreemptRequested() == false) {
            rate.sleep();
            spinOnce();
        }

        // assignment interrupted
        if (assignment_server->isPreemptRequested()) {
            assignment_server->setPreempted();
            ROS_ERROR("ROI assignment preempted");
            return;
        }
    }
    while (auct->won() == false);

    // completed assignment action
    try {
        // get assigned roi coordinates
        cpswarm_msgs::RoiAssignmentResult result;
        result.roi = rois.get_coords(auct->get_assigned());

        // inform others
        broadcast_assignment();

        ROS_DEBUG("Successfully acquired ROI %s ", auct->get_assigned().c_str());

        // successfully completed assignment action
        assignment_server->setSucceeded(result);
    }
    catch (const exception& e) {
        // failed to complete assignment action
        ROS_FATAL("ROI assignment failure: Failed to get result: %s", e.what());
        assignment_server->setAborted();
    }
}

/**
 * @brief A ROS node that assigns ROIs between CPSs.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char **argv)
{
    // init ros node
    init(argc, argv, "roi_assignment");
    NodeHandle nh;

    // retrieve rois
    ServiceClient get_rois_client = nh.serviceClient<cpswarm_msgs::GetMultiPoints>("rois/get_all");
    ROS_DEBUG("Waiting for ROI service...");
    get_rois_client.waitForExistence();
    ROS_DEBUG("ROI service available");
    cpswarm_msgs::GetMultiPoints gmp;
    if (get_rois_client.call(gmp)) {
        // initialize roi data
        rois.init(expand(gmp.response.layout, gmp.response.points));
    }
    // rois could not be retrieved
    else {
        ROS_FATAL("ROI assignment failure: Failed to get ROIs: Failed to call service 'rois/get_all");
        return 1;
    }

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // get position
    ROS_DEBUG("Wait for valid position...");
    Subscriber pos_sub = nh.subscribe("position_provider/pose", queue_size, pos_cb);
    while (ok() && position.header.stamp.isZero()) {
        spinOnce();
        rate.sleep();
    }
    ROS_DEBUG("Valid position received: (%.2f,%.2f)", position.pose.position.x, position.pose.position.y);

    // get uuid
    ROS_DEBUG("Wait for valid UUID...");
    Subscriber uuid_sub = nh.subscribe("bridge/uuid", queue_size, uuid_cb);
    while (ok() && uuid.compare("") == 0) {
        spinOnce();
        rate.sleep();
    }
    ROS_DEBUG("Valid UUID received: %s", uuid.c_str());

    // create auctioning object
    auct = new auctioning(uuid);

    // init random number generator
    int seed;
    nh.param<int>("/rng_seed", seed, 0);
    if (seed != 0)
        rng = new random_numbers::RandomNumberGenerator(seed);
    else
        rng = new random_numbers::RandomNumberGenerator();

    // publisher for auction bids
    bid_pub = nh.advertise<cpswarm_msgs::TaskAllocationEvent>("rois/assignment/bid", queue_size);

    // listen to auctions in the background
    Subscriber auction_sub = nh.subscribe("rois/assignment/auction", queue_size, auction_cb); // listen to auction openings
    Subscriber result_sub = nh.subscribe("rois/assignment/result", queue_size, result_cb); // listen to auction closings

    // provide action server
    AssignmentAction assignment_action(nh, "rois/assign", boost::bind(&roi_assignment, _1, &assignment_action), false); // no autostart
    assignment_action.start();

    ROS_INFO("ROI assignment action server available");

    // wait for assignment requests
    spin();

    // clean up
    delete rng;
    delete auct;

    return 0;
}
