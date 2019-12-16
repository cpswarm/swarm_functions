#ifndef VISITED_LAYER_H
#define VISITED_LAYER_H

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <visited_map/VisitedLayerConfig.h>

using namespace std;
using namespace ros;
using namespace costmap_2d;

namespace visited_layer
{
    /**
     * @brief TODO
     */
    class VisitedLayer : public CostmapLayer
    {
    public:
        /**
         * @brief Constructor.
         */
        VisitedLayer ();

        /**
         * @brief TODO
         */
        virtual void onInitialize ();

        /**
         * @brief TODO
         */
        virtual void updateBounds (double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y);

        /**
         * @brief TODO
         */
        virtual void updateCosts (Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    private:
        /**
         * @brief TODO
         */
        void reconfigureCB (visited_map::VisitedLayerConfig &config, uint32_t level);

        /**
         * @brief TODO
         */
        dynamic_reconfigure::Server<visited_map::VisitedLayerConfig> *dsrv_;

        /**
         * @brief Radius in meters around the CPS which is marked as visited.
         */
        double fov;
    };
}

#endif // VISITED_LAYER_H
