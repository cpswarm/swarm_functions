#include "lib/connected_components.h"

connected_components::connected_components (valarray<int> m, int rows, int cols, bool zeroAsBg) : rows(rows), cols(cols), zeroAsBg(zeroAsBg)
{
    image.resize(rows*cols);
    image.assign(begin(m), end(m));

    label2d.resize(rows*cols);
}

valarray<int> connected_components::compactLabeling ()
{
    // label first
    vector<int> label = labeling();
    vector<int> stat(next_label+1);
    for (int i=0; i<image.size(); i++) {
        stat[label[i]]++;
    }

    // label 0 will be mapped to 0
    stat[0] = 0;

    // whether 0 is background or not
    int j = 1;
    for (int i=1; i<stat.size(); i++) {
        if (stat[i] != 0)
            stat[i] = j++;
    }

    next_label= j-1;
    int locIDX=0;
    for (int i=0; i<label.size(); ++i) {
        label[i] = stat[label[i]];
    }

    copy(label.begin(), label.end(), begin(label2d));

    return label2d;
}

int connected_components::getMaxLabel ()
{
    return next_label;
}

vector<int> connected_components::labeling ()
{
    vector<int> rst(cols*rows);
    vector<int> parent(cols*rows);
    vector<int> labels(cols*rows);

    // region label starts from 1;
    // this is required as union-find data structure
    int next_region = 1;
    for (int y = 0; y < rows; ++y ) {
        for (int x = 0; x < cols; ++x ) {
            if (image[y*cols+x] == 0 && zeroAsBg)
                continue;

            int k = 0;
            bool connected = false;

            // if connected to the left
            if (x > 0 && image[y*cols+x-1] == image[y*cols+x]) {
                k = rst[y*cols+x-1];
                connected = true;
            }

            // if connected upwards
            if (y > 0 && image[(y-1)*cols+x] == image[y*cols+x] && (!connected || image[(y-1)*cols+x] < k)) {
                k = rst[(y-1)*cols+x];
                connected = true;
            }

            // not connected
            if (!connected) {
                k = next_region;
                next_region++;
            }

            // set label
            rst[y*cols+x] = k;

            // if connected, but with different label, then do union
            if (x > 0 && image[y*cols+x-1] == image[y*cols+x] && rst[y*cols+x-1] != k)
                uf_union(k, rst[y*cols+x-1], parent);
            if (y > 0 && image[(y-1)*cols+x] == image[y*cols+x] && rst[(y-1)*cols+x] != k)
                uf_union(k, rst[(y-1)*cols+x], parent);
        }
    }

    // begin the second pass, assign the new labels
    // if 0 is reserved for background, then the first available label is 1
    next_label = 1;
    for (int i=0; i<cols*rows; i++) {
        if (image[i] != 0 || !zeroAsBg) {
            rst[i] = uf_find(rst[i], parent, labels);
            // The labels are from 1, if label 0 should be considered, then
            // all the label should minus 1
            if (!zeroAsBg)
                rst[i]--;
        }
    }

    // record the max label
    next_label--;
    if (!zeroAsBg)
        next_label--;

    return rst;
}

void connected_components::uf_union (int x, int y, vector<int>& parent)
{
    while ( parent[x]>0 )
        x = parent[x];
    while ( parent[y]>0 )
        y = parent[y];
    if ( x != y ) {
        if (x<y)
            parent[x] = y;
        else parent[y] = x;
    }
}

int connected_components::uf_find (int x, vector<int> parent, vector<int>& label)
{
    while (parent[x] > 0) {
        x = parent[x];
    }
    if (label[x] == 0) {
        label[x] = next_label;
        ++next_label;
    }
    return label[x];
}

void connected_components::constructBinaryImages (int robotsLabel)
{
    BinaryRobot = label2d;
    BinaryNonRobot = label2d;

    for (int i=0; i<label2d.size(); ++i) {
        if (label2d[i] == robotsLabel) {
            BinaryRobot[i] = 1;
            BinaryNonRobot[i] = 0;
        }
        else if (label2d[i] != 0) {
            BinaryRobot[i] = 0;
            BinaryNonRobot[i] = 1;
        }
    }
}

valarray<float> connected_components::NormalizedEuclideanDistanceBinary (bool RobotR)
{
    valarray<float> Region(rows*cols);

    vector<float> f(max(rows, cols));
    vector<float> d(f.size());
    vector<int> v(f.size());
    vector<float> z(f.size() + 1);

    for (int x=0; x<cols; x++) {
        for (int y=0; y<rows; y++) {
            if (RobotR)
                f[y] = BinaryRobot[y*cols+x] == 0 ? numeric_limits<float>::max() : 0;
            else
                f[y] = BinaryNonRobot[y*cols+x] == 0 ? numeric_limits<float>::max() : 0;
        }

        DT1D(f, d, v, z);
        for (int y = 0; y < rows; y++) {
            Region[y*cols+x] = d[y];
        }
    }

    float maxV=0, minV=numeric_limits<float>::max();
    for (int y = 0; y < rows; y++) {
        DT1D(getVector(Region,y), d, v,  z);

        for (int x = 0; x < cols; x++) {
            Region[y*cols+x] = (float) sqrt(d[x]);
            if (maxV < Region[y*cols+x]) {maxV = Region[y*cols+x];}
            if (minV > Region[y*cols+x]) {minV = Region[y*cols+x];}
        }
    }


    //Normalization
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            if (RobotR)
                Region[y*cols+x] = (Region[y*cols+x] - minV) * (1/(maxV-minV)) +1;
            else
                Region[y*cols+x] = (Region[y*cols+x] - minV) * (1/(maxV-minV));
        }
    }

    return Region;
}

void connected_components::DT1D(vector<float> f, vector<float>& d, vector<int>& v , vector<float>& z)
{
    int k = 0;
    v[0] = 0;
    z[0] = -numeric_limits<float>::max();
    z[1] = numeric_limits<float>::max();

    for (int q = 1; q < f.size(); q++) {
        float s  = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);

        while (s <= z[k]) {
            k--;
            s  = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = numeric_limits<float>::max();
    }

    k = 0;
    for (int q = 0; q < f.size(); q++) {
        while (z[k + 1] < q) {k++;}

        d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
    }
}

vector<float> connected_components::getVector (valarray<float> A, int row)
{
    valarray<float> col = A[slice(row*cols, cols, 1)];
    return vector<float>(begin(col), end(col));
}
