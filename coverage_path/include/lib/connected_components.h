#ifndef CONNECTED_COMPONENTS_H
#define CONNECTED_COMPONENTS_H

#include <vector>
#include <valarray>

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
    connected_components(valarray<int> m, int rows, int cols, bool zeroAsBg);

    /**
     * @brief Rearrange the labels to make the label numbers consecutive.
     * @return TODO
     */
    valarray<int> compactLabeling ();

    /**
     * @brief Get the max label of the labeling process which is in the range [0,max_label].
     * @return The maximum label.
     */
    int getMaxLabel ();


    /**
     * @brief Label the connected components.
     * @param image The data to label.
     * @param zeroAsBg Label 0 is treated as background, i.e., ignored.
     */
    vector<int> labeling ();

    /**
     * @brief TODO
     */
    void uf_union (int x, int y, vector<int> parent);

    /**
     * @brief Return the root label (starts from 1 because label array is initialized to 0 at first). The label array records the new label for every root.
     * @param TODO
     * @return TODO
     */
    int uf_find (int x, vector<int> parent, vector<int> label);

    /**
     * @brief TODO
     */
    void constructBinaryImages (int robotsLabel);

    /**
     * @brief Calculate the normalized euclidean distance transform of a binary image with foreground pixels set to 1 and background set to 0.
     * @param TODO
     * @return TODO
     */
    valarray<float> NormalizedEuclideanDistanceBinary (bool RobotR);

private:
    void DT1D (vector<float> f, vector<float> d, vector<int> v , vector<float> z);

    vector<float> getVector(valarray<float> A, int row);

    valarray<int> label2d, BinaryRobot, BinaryNonRobot;

    int rows, cols;

    int next_label = 1;

    vector<int> image;

    bool zeroAsBg;
};

#endif // CONNECTED_COMPONENTS_H
