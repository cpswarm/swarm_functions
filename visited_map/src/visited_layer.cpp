#include "visited_layer.h"

PLUGINLIB_EXPORT_CLASS(visited_layer::VisitedLayer, costmap_2d::Layer)

namespace visited_layer
{
    VisitedLayer::VisitedLayer ()
    {
    }

    void VisitedLayer::onInitialize ()
    {
        // initialize layer
        ros::NodeHandle nh("~/" + name_);
        current_ = true;
        default_value_ = NO_INFORMATION;
        matchSize();

        dsrv_ = new dynamic_reconfigure::Server<visited_map::VisitedLayerConfig>(nh);
        dynamic_reconfigure::Server<visited_map::VisitedLayerConfig>::CallbackType cb = boost::bind(&VisitedLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void VisitedLayer::updateBounds (double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        if (!enabled_)
            return;

        // mark area around robot as visited
        unsigned int mx;
        unsigned int my;
        for (double r=0; r<=fov; r+=0.1) {
            for (double a=0; a<2*M_PI; a+=0.1) { // TODO: define resolution costmap_2d::Costmap2D::getResolution()
                if (worldToMap(robot_x + r * cos(a), robot_y + r * sin(a), mx, my)){
                    setCost(mx, my, LETHAL_OBSTACLE); // TODO: is it useful to mark it as obstacle?
                }
            }
        }

        // return size to be updated
        *min_x = std::min(*min_x, robot_x-fov);
        *min_y = std::min(*min_y, robot_y-fov);
        *max_x = std::max(*max_x, robot_x+fov);
        *max_y = std::max(*max_y, robot_y+fov);
    }

    void VisitedLayer::updateCosts (Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
            return;

        // copy information from this layer to master grid
        for (int j=min_j; j<max_j; ++j) {
            for (int i=min_i; i<max_i; ++i) {
                int index = getIndex(i, j);
                if (costmap_[index] == NO_INFORMATION)
                    continue;
                master_grid.setCost(i, j, costmap_[index]);
            }
        }
    }

    void VisitedLayer::reconfigureCB (visited_map::VisitedLayerConfig &config, uint32_t level)
    {
        fov = config.fov;

        if (enabled_ != config.enabled)
        {
            enabled_ = config.enabled;
            current_ = false;
        }
    }
}
