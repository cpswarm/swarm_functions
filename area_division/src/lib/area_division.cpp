#include "lib/area_division.h"

area_division::area_division ()
{
    // read parameters
    NodeHandle nh;
    nh.param(this_node::getName() + "/optimizer/iterations", max_iter, 1);
    nh.param(this_node::getName() + "/optimizer/variate_weight", variate_weight, 0.01);
    nh.param(this_node::getName() + "/optimizer/random_level", random_level, 1e-4);
    nh.param(this_node::getName() + "/optimizer/discrepancy", discr, 30);
}

void area_division::divide ()
{
    // initializations
    int NoTiles = rows*cols;
    double fairDivision = 1.0 / nr;
    int effectiveSize = NoTiles - nr - ob;
    int termThr;
    if (effectiveSize % nr !=0) {
        termThr=1;
    }
    else {
        termThr=0;
    }

    // initialize distances of cells to cps
    vector<valarray<double>> AllDistances(nr, valarray<double>(rows*cols));
    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols;j++) {
            for (int r=0; r<nr; r++) {
                AllDistances[r][i*cols+j] = hypot(cps[r][1]-i, cps[r][0]-j);
            }
        }
    }


    vector<valarray<double>> MetricMatrix = AllDistances;

    // perform area division
    success = false;
    valarray<double> criterionMatrix;
    while (termThr<=discr && !success) {
        // initializations
        double downThres = ((double)NoTiles-(double)termThr*(nr-1)) / (double)(NoTiles*nr);
        double upperThres = ((double)NoTiles+termThr) / (double)(NoTiles*nr);
        success = true;

        // main optimization loop
        int iter = 0;
        while (iter <= max_iter) {
            assign(MetricMatrix);

            // find connected areas
            vector<valarray<float>> ConnectedMultiplierList(nr);
            double plainErrors[nr];
            double divFairError[nr];
            for (int r=0; r<nr; r++) {
                valarray<float> ConnectedMultiplier(1, rows*cols);
                regions[r] = true;

                connected_components cc(BWlist[r], rows, cols, true);
                valarray<int> Ilabel = cc.compactLabeling();
                // at least one unconnected regions among r-robot's regions is found
                if (cc.getMaxLabel() > 1) {
                    regions[r] = false;

                    // find robot's sub-region and construct robot and non-robot binary regions
                    cc.constructBinaryImages(Ilabel[cps[r][1] * cols + cps[r][0]]);

                    // construct the final connected component multiplier
                    ConnectedMultiplier = CalcConnectedMultiplier(cc.NormalizedEuclideanDistanceBinary(true), cc.NormalizedEuclideanDistanceBinary(false));
                }
                ConnectedMultiplierList.push_back(ConnectedMultiplier);

                // calculate the deviation from the the optimal assignment
                plainErrors[r] = ArrayOfElements[r] / (double) effectiveSize;
                if (plainErrors[r] < downThres) {
                    divFairError[r] = downThres - plainErrors[r];
                }
                else if (plainErrors[r] > upperThres) {
                    divFairError[r] = upperThres - plainErrors[r];
                }
            }

            // exit conditions
            if (isThisAGoalState(termThr)) {
                break;
            }

            // check fairness among different partitions
            double TotalNegPerc = 0.0, totalNegPlainErrors = 0.0;
            double correctionMult[nr];
            for (int r=0; r<nr; r++) {
                if (divFairError[r] < 0) {
                    TotalNegPerc += abs(divFairError[r]);
                    totalNegPlainErrors += plainErrors[r];
                }
                correctionMult[r] = 1.0;
            }

            // restore fairness
            for (int r=0; r<nr; r++) {
                if (totalNegPlainErrors != 0.0) {
                    if (divFairError[r]<0.0) {
                        correctionMult[r] = 1.0 + (plainErrors[r] / totalNegPlainErrors) * (TotalNegPerc / 2.0);
                    }
                    else {
                        correctionMult[r] = 1.0 - (plainErrors[r] / totalNegPlainErrors) * (TotalNegPerc / 2.0);
                    }

                    valarray<double> criterionMatrix(correctionMult[r], rows*cols);
                }
                MetricMatrix[r] = FinalUpdateOnMetricMatrix(criterionMatrix, MetricMatrix[r], ConnectedMultiplierList[r]);
            }

            iter++;
        }

        if (iter >= max_iter) {
            max_iter = max_iter/2;
            success = false;
            termThr++;
        }
    }
}

nav_msgs::OccupancyGrid area_division::get_grid (nav_msgs::OccupancyGrid map, string cps)
{
    nav_msgs::OccupancyGrid assigned;
    assigned = map;

    for (int i=0; i<A.size(); ++i) {
        // mark assigned cells as free
        if (A[i] == uuid_map[cps]) {
            assigned.data[i] = 0;
        }
        // mark cells assigned to other cpss as obstacles
        else {
            assigned.data[i] = 100;
        }
    }

    assigned.header.stamp = Time::now();
    assigned.info.map_load_time == Time::now();

    return assigned;
}

void area_division::initialize_cps (map<string, vector<int>> cpss)
{
    // initialize
    nr = 0;
    cps.clear();
    uuid_map.clear();
    A.resize(rows*cols);
    regions.clear();
    regions.resize(cpss.size());

    // place cpss in the map
    for (auto c : cpss) {
        // index of position in grid map
        int idx = c.second[1] * cols + c.second[0];

        // place cps in data structures
        gridmap[idx] = numeric_limits<signed char>::max();
        A[idx] = nr;

        // store cps position and mapping of uuid
        cps.push_back(initializer_list<int>{c.second[0],c.second[1]});
        uuid_map[c.first] = nr;

        // count number of cpss
        nr++;
    }
}

void area_division::initialize_map (int r, int c, vector<signed char> src)
{
    // initialization
    rows = r;
    cols = c;
    ob = 0;
    gridmap = src;
}

void area_division::assign (vector<valarray<double>> matrix)
{
    BWlist.resize(nr);
    for (int r=0; r<nr; r++) {
        BWlist[r].resize(rows*cols);
        BWlist[r][cps[r][1] * cols + cps[r][0]] = 1;
    }

    ArrayOfElements.clear();
    ArrayOfElements.resize(nr);
    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols; j++) {
            // unknown grid cell (treat as free)
            if (gridmap[i*cols+j] == -1) {
                double minV = matrix[0][i*cols+j];
                int indMin = 0;
                for (int r=1; r<nr; r++) {
                    if (matrix[r][i*cols+j] < minV) {
                        minV = matrix[r][i*cols+j];
                        indMin = r;
                    }
                }
                A[i*cols+j] = indMin;
                BWlist[indMin][i*cols+j] = 1;
                ArrayOfElements[indMin]++;
            }

            // obstacle
            else if (gridmap[i*cols+j] < numeric_limits<signed char>::max()) {
                A[i*cols+j] = nr;
            }
        }
    }
}

valarray<double> area_division::FinalUpdateOnMetricMatrix(valarray<double> CM, valarray<double> curentONe, valarray<float> CC)
{
    random_numbers::RandomNumberGenerator* rng = new random_numbers::RandomNumberGenerator();

    valarray<double> MMnew(rows*cols);

    for (int i=0; i<MMnew.size(); ++i) {
        double random = 2.0*random_level * rng->uniform01() + 1.0 - random_level;
        MMnew[i] = curentONe[i] * CM[i] * random * CC[i];
    }

    delete rng;

    return MMnew;
}


bool area_division::isThisAGoalState(int thres)
{
    int maxCellsAss = 0;
    int minCellsAss = numeric_limits<int>::max();


    for (int r=0; r<nr; r++) {
        if (maxCellsAss < ArrayOfElements[r]) {
            maxCellsAss = ArrayOfElements[r];
        }
        if (minCellsAss > ArrayOfElements[r]) {
            minCellsAss = ArrayOfElements[r];
        }

        if (!regions[r]) {
            return false;
        }
    }

    return (maxCellsAss - minCellsAss) <= thres;
}

valarray<float> area_division::CalcConnectedMultiplier(valarray<float> dist1, valarray<float> dist2)
{
    valarray<float> returnM(rows*cols);
    float MaxV = 0;
    float MinV = numeric_limits<float>::max();
    for (int i=0;i<rows;i++){
        for (int j=0;j<cols;j++){
            returnM[i*cols+j] =dist1[i*cols+j] - dist2[i*cols+j];
            if (MaxV < returnM[i*cols+j]) {MaxV = returnM[i*cols+j];}
            if (MinV > returnM[i*cols+j]) {MinV = returnM[i*cols+j];}
        }
    }

    for (int i=0;i<rows;i++){
        for (int j=0;j<cols;j++){
            returnM[i*cols+j] =(returnM[i*cols+j] - MinV)*((2*(float)variate_weight)/(MaxV-MinV))+(1-(float)variate_weight);
        }
    }

    return  returnM;
}