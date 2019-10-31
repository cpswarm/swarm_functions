#ifndef CONNECTED_COMPONENTS_H
#define CONNECTED_COMPONENTS_H

#include <vector>
#include <valarray>
#include <ros/ros.h>

using namespace std;

/**
 * @brief Use row-by-row labeling algorithm to label connected components. The algorithm makes two passes over the image: one pass to record equivalences and assign temporary labels and the second to replace each temporary label by the label of its equivalence class.
 * Based on Linda G. Shapiro, Computer Vision: Theory and Applications.  (3.4 Connected Components Labeling), Rosenfeld and Pfaltz (1966).
 */
class connected_components
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param m The matrix containing the data to label.
     * @param rows Number of rows in the matrix.
     * @param cols Number of columns in the matrix.
     * @param zeroAsBg Whether 0 represents background and is ignored.
     */
    connected_components (valarray<int> m, int rows, int cols, bool zeroAsBg);

    /**
     * @brief Rearrange the labels to make the label numbers consecutive.
     * @return An array of labels.
     */
    valarray<int> compactLabeling ();

    /**
     * @brief Get the max label of the labeling process which is in the range [0,max_label].
     * @return The maximum label.
     */
    int getMaxLabel ();


    /**
     * @brief Label the connected components.
     * @return An array of labels.
     */
    vector<int> labeling ();

    /**
     * @brief Combine two labels.
     * @param x The first label.
     * @param y The second label.
     * @param parent The label hierarchy.
     */
    void uf_union (int x, int y, vector<int>& parent);

    /**
     * @brief Return the root label (starts from 1 because label array is initialized to 0 at first). The label array records the new label for every root.
     * @param x The label to start the search from.
     * @param parent The label hierarchy.
     * @param label The vector of labels.
     * @return The root label.
     */
    int uf_find (int x, vector<int> parent, vector<int>& label);

    /**
     * @brief Create binary arrays of connected labels.
     * @param robotsLabel The label to create the binary array for.
     */
    void constructBinaryImages (int robotsLabel);

    /**
     * @brief Calculate the normalized euclidean distance transform of a binary image with foreground pixels set to 1 and background set to 0.
     * @param RobotR Whether to compute the distances to the robot.
     * @return The distance array.
     */
    valarray<float> NormalizedEuclideanDistanceBinary (bool RobotR);

private:
    /**
     * @brief DT1D.
     * @param f
     * @param d
     * @param v
     * @param z
     */
    void DT1D (vector<float> f, vector<float>& d, vector<int>& v , vector<float>& z);

    /**
     * @brief Get a slice of an array.
     * @param A The array to take the slice from.
     * @param row The index of the slice to take.
     * @return The selected slice.
     */
    vector<float> getVector (valarray<float> A, int row);

    /**
     * @brief Array of labels.
     */
    valarray<int> label2d;

    /**
     * @brief Binary array of the label assigned to a robot.
     */
    valarray<int> BinaryRobot;

    /**
     * @brief Binary array of the labels not assigned to a robot.
     */
    valarray<int> BinaryNonRobot;

    /**
     * @brief Number of rows of the image.
     */
    int rows;

    /**
     * @brief Number of columns of the image.
     */
    int cols;

    /**
     * @brief The current label ID.
     */
    int next_label = 1;

    /**
     * @brief The image to perform the labeling on.
     */
    vector<int> image;

    /**
     * @brief Whether 0 should be considered or not.
     */
    bool zeroAsBg;
};

#endif // CONNECTED_COMPONENTS_H
