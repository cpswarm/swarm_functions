#ifndef AREA_DIVISION_LIB_H
#define AREA_DIVISION_LIB_H

#include <vector>
#include <valarray>
#include <map>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random_numbers/random_numbers.h>
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
     * @brief Define the CPSs.
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

    /**
     * @brief Define the optimization parameters.
     * @param iters Maximum number of iterations of the optimization algorithm.
     * @param vWeight TODO
     * @param rLevel TODO
     * @param discr TODO
     */
    void setup (int iters, double vWeight, double rLevel, int discr);

private:
    /**
     * @brief TODO
     */
    void assign (vector<valarray<double>> q);

    /**
     * @brief TODO
     */
    valarray<float> CalcConnectedMultiplier (valarray<float> dist1, valarray<float> dist2);

    /**
     * @brief TODO
     */
    void calculateRobotBinaryArrays ();

    /**
     * @brief TODO
     */
    valarray<double> FinalUpdateOnMetricMatrix (valarray<double> CM, valarray<double> curentONe, valarray<float> CC);

    /**
     * @brief TODO
     */
    bool isThisAGoalState (int thres);

    /**
     * @brief TODO
     */
    double variate_weight;

    /**
     * @brief TODO
     */
    double random_level;

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
     * @brief TODO
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
     * @brief TODO
     */
    int discr;
};

#endif // AREA_DIVISION_LIB_H

