#ifndef AREA_DIVISION_LIB_H
#define AREA_DIVISION_LIB_H

#include <vector>
#include <valarray>
#include <map>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "lib/connected_components.h"

using namespace std;
using namespace ros;

/**
 * @brief A class to divide the environment optimally among multiple cyber physical systems (CPSs).
 */
class area_division
{
public:
    /**
     * @brief Constructor.
     */
    area_division();

    /**
     * @brief Perform the area division.
     */
    void divide ();

    /**
     * @brief Get the region assigned to a CPS.
     * @param map Original grid map.
     * @param cps UUID of the CPS.
     * @return Grid map which contains obstacles in cells assigned to the CPS.
     */
    nav_msgs::OccupancyGrid get_grid (nav_msgs::OccupancyGrid map, string cps);

    /**
     * @brief Define the CPS positions.
     * @param cpss Mapping from UUIDs to positions of the CPSs.
     */
    void initialize_cps (map<string, vector<int>> cpss);

    /**
     * @brief Define the grid map.
     * @param r Number of rows in the grid map.
     * @param c Number of columns in the grid map.
     * @param src Grid map describing the environment in the ROS format.
     */
    void initialize_map (int r, int c, vector<signed char> src);

private:
    /**
     * @brief Define the assignment matrix based on a given distance metric matrix.
     * @param matrix A matrix of distance metrics.
     */
    void assign (vector<valarray<double>> matrix);

    /**
     * @brief Calculate the connected multiplier.
     * @param dist1
     * @param dist2
     * @return The connected multiplier array.
     */
    valarray<float> CalcConnectedMultiplier (valarray<float> dist1, valarray<float> dist2);

    /**
     * @brief Update the metric matrix.
     * @param CM Correction multiplier.
     * @param curentONe Current metric matrix.
     * @param CC Connected multiplier array.
     * @return The updated metric matrix.
     */
    valarray<double> FinalUpdateOnMetricMatrix (double CM, valarray<double> curentONe, valarray<float> CC);

    /**
     * @brief Check if the areas assigned to each robot are of similar size.
     * @param thres The maximum number of grid cells that the areas assigned to different robots is allowed to differ.
     * @return True, if the maximum assigned area to any robot is at most thresh larger than the minimum assigned area to any robot, false otherwise.
     */
    bool isThisAGoalState (int thres);

    /**
     * @brief Maximum variate weight of connected components.
     */
    double variate_weight;

    /**
     * @brief Number of rows in the grid map.
     */
    int rows;

    /**
     * @brief Number of columns in the grid map.
     */
    int cols;

    /**
     * @brief Number of robots.
     */
    int nr;

    /**
     * @brief Number of obstacles.
     */
    int ob;

    /**
     * @brief Maximum number of iterations of the optimization process.
     */
    int max_iter;

    /**
     * @brief The environment grid map. Robots are represented by 2, obstacles by 1/-2, empty cells by -1.
     */
    vector<signed char> gridmap;

    /**
     * @brief Positions of the CPSs.
     */
    vector<vector<int>> cps;

    /**
     * @brief UUID mapping of CPSs.
     */
    map<string, int> uuid_map;

    /**
     * @brief A binary array for each CPS which indicates free space.
     */
    vector<valarray<int>> BWlist;

    /**
     * @brief Area assignment matrix.
     */
    valarray<int> A;

    /**
     * @brief Grid cells not assigned to the robots.
     */
    vector<int> ArrayOfElements;

    /**
     * @brief Whether the regions of the robots are connected.
     */
    vector<bool> regions;

    /**
     * @brief Whether the assignment succeeded.
     */
    bool success;

    /**
     * @brief Maximum difference between number of assigned grid cells to each CPS.
     */
    int discr;
};

#endif // AREA_DIVISION_LIB_H

