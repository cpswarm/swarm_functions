#include "lib/area_division.h"

area_division::area_division()
{
}

void area_division::setup(int iters, double vWeight, double rLevel, int discr, bool imp)
{
    maxIter = iters;
    variateWeight = vWeight;
    randomLevel = rLevel;
    discr = discr;
    UseImportance = imp;
}

void area_division::initialize_map(int r, int c, vector<signed char> src)
{
    // initialization
    rows = r;
    cols = c;
    ob = 0;
    GridEnv = src;
}

void area_division::initialize_cps(map<string, vector<int>> cpss)
{
    // initialize
    nr = 0;
    RobotsInit.clear();
    uuid_map.clear();
    robotBinary.resize(rows*cols);
    A.resize(rows*cols);
    ConnectedRobotRegions.clear();
    ConnectedRobotRegions.resize(cpss.size());

    // place cpss in the map
    for (auto cps : cpss) {
        // index of position in grid map
        int idx = cps.second[1] * cols + cps.second[0];

        // place cps in data structures
        robotBinary[idx] = true;
        GridEnv[idx] = numeric_limits<signed char>::max();
        A[idx] = nr;

        // store cps position and mapping of uuid
        RobotsInit.push_back(initializer_list<int>{cps.second[0],cps.second[1]});
        uuid_map[cps.first] = nr;

        // count number of cpss
        nr++;
    }
}

void area_division::divide()
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

    // initialize distance to cps and importance of all tiles
    vector<valarray<double>> AllDistances(nr, valarray<double>(rows*cols));
    vector<valarray<double>> TilesImportance(nr, valarray<double>(rows*cols));
    double MaximumDist[nr];
    double MaximumImportance[nr];
    double MinimumImportance[nr];
    for (int r=0; r<nr; r++) {
        MinimumImportance[r] = numeric_limits<double>::max();
    }
    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols;j++) {
            double tempSum=0.0;
            for (int r=0; r<nr; r++) {
                AllDistances[r][i*cols+j] = hypot(RobotsInit[r][1]-i, RobotsInit[r][0]-j);
                if (AllDistances[r][i*cols+j] > MaximumDist[r]) {
                    MaximumDist[r] = AllDistances[r][i*cols+j];
                }
                tempSum += AllDistances[r][i*cols+j];
            }
            for (int r=0; r<nr; r++) {
                TilesImportance[r][i*cols+j] = 1.0 / (tempSum - AllDistances[r][i*cols+j]);
                if (TilesImportance[r][i*cols+j] > MaximumImportance[r]) {
                    MaximumImportance[r]=TilesImportance[r][i*cols+j];
                }
                if (TilesImportance[r][i*cols+j] < MinimumImportance[r]) {
                    MinimumImportance[r]=TilesImportance[r][i*cols+j];
                }
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
        while (iter <= maxIter) {
            assign(MetricMatrix);

            // find connected areas
            vector<valarray<float>> ConnectedMultiplierList(nr);
            double plainErrors[nr];
            double divFairError[nr];
            for (int r=0; r<nr; r++) {
                valarray<float> ConnectedMultiplier(1, rows*cols);
                ConnectedRobotRegions[r] = true;

                connected_components cc(BWlist[r], rows, cols, true);
                valarray<int> Ilabel = cc.compactLabeling();
                // at least one unconnected regions among r-robot's regions is found
                if (cc.getMaxLabel() > 1) {
                    ConnectedRobotRegions[r] = false;

                    // find robot's sub-region and construct robot and non-robot binary regions
                    cc.constructBinaryImages(Ilabel[RobotsInit[r][1] * cols + RobotsInit[r][0]]);

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

                    valarray<double> criterionMatrix = calculateCriterionMatrix(TilesImportance[r], MinimumImportance[r], MaximumImportance[r], correctionMult[r], (divFairError[r] < 0));
                }
                MetricMatrix[r] = FinalUpdateOnMetricMatrix(criterionMatrix, generateRandomMatrix(), MetricMatrix[r], ConnectedMultiplierList[r]);
            }

            iter++;
        }

        if (iter >= maxIter) {
            maxIter = maxIter/2;
            success = false;
            termThr++;
        }
    }

    // convert robot assignment to binary matrix
    calculateRobotBinaryArrays();
}

valarray<bool> area_division::get_region(string cps)
{
    return BinaryRobotRegions[uuid_map[cps]];
}

nav_msgs::OccupancyGrid area_division::get_grid(nav_msgs::OccupancyGrid map, string cps)
{
    nav_msgs::OccupancyGrid assigned;
    assigned = map;

    // mark assigned cells as obstacles
    for (int i=0; i<BinaryRobotRegions[uuid_map[cps]].size(); ++i) {
        if (BinaryRobotRegions[uuid_map[cps]][i])
            assigned.data[i] = 0;
        else
            assigned.data[i] = 100;
    }

    assigned.header.stamp = Time::now();
    assigned.info.map_load_time == Time::now();

    return assigned;
}

bool area_division::getSuccess()
{
    return success;
}

int area_division::getNr()
{
    return nr;
}

int area_division::getNumOB()
{
    return ob;
}

valarray<int> area_division::getAssignmentMatrix()
{
    return A;
}

valarray<bool> area_division::getRobotBinary()
{
    return robotBinary;
}

vector<valarray<bool>> area_division::getBinaryRobotRegions()
{
    return BinaryRobotRegions;
}

vector<vector<int>> area_division::getRobotsInit()
{
    return RobotsInit;
}

int area_division::getEffectiveSize()
{
    return 4*(rows*cols-ob);
}

int area_division::getMaxCellsAss()
{
    return 4*(maxCellsAss+1);
}

int area_division::getMinCellsAss()
{
    return 4*(minCellsAss+1);
}

int area_division::getDiscr()
{
    return discr;
}

int area_division::getMaxIter()
{
    return  maxIter;
}

int area_division::getAchievedDiscr()
{
    return maxCellsAss-minCellsAss;
}

void area_division::calculateRobotBinaryArrays()
{
    BinaryRobotRegions.resize(nr);
    for (int r=0; r<nr; r++) {
        BinaryRobotRegions[r].resize(rows*cols);
    }
    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols; j++) {
            if (A[i*cols+j] < nr) {
                BinaryRobotRegions[A[i*cols+j]][i*cols+j] = true;
            }
        }
    }
}

valarray<double> area_division::FinalUpdateOnMetricMatrix(valarray<double> CM, valarray<double> RM, valarray<double> curentONe, valarray<float> CC)
{
    valarray<double> MMnew(rows*cols);

    for (int i=0; i<rows; i++){
        for (int j=0; j<cols; j++) {
            MMnew[i*cols+j] = curentONe[i*cols+j]*CM[i*cols+j]*RM[i*cols+j]*CC[i*cols+j];
        }
    }

    return MMnew;
}

valarray<double> area_division::generateRandomMatrix()
{

    valarray<double> RandomMa(rows*cols);
    random_numbers::RandomNumberGenerator* rng = new random_numbers::RandomNumberGenerator();

    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols; j++) {
            RandomMa[i*cols+j] = 2.0*randomLevel*rng->uniform01()+1.0-randomLevel;
        }
    }

    delete rng;

    return  RandomMa;
}

valarray<double> area_division::calculateCriterionMatrix(valarray<double> TilesImp, double minImp, double maxImp, double corMult, bool SmallerThan0)
{
    valarray<double> retrunCriter(rows*cols);

    for (int i=0; i<rows; i++){
        for (int j=0; j<cols; j++){
            if (UseImportance) {
                if (SmallerThan0) {
                    retrunCriter[i*cols+j] = (TilesImp[i*cols+j] - minImp)*((corMult-1)/(maxImp-minImp))+1;
                }
                else {
                    retrunCriter[i*cols+j] = (TilesImp[i*cols+j] - minImp)*((1-corMult)/(maxImp-minImp))+corMult;
                }
            }
            else{
                retrunCriter[i*cols+j] = corMult;
            }
        }
    }

    return retrunCriter;
}


bool area_division::isThisAGoalState(int thres)
{
    maxCellsAss = 0;
    minCellsAss = numeric_limits<int>::max();


    for (int r=0; r<nr; r++) {
        if (maxCellsAss < ArrayOfElements[r]) {
            maxCellsAss = ArrayOfElements[r];
        }
        if (minCellsAss > ArrayOfElements[r]) {
            minCellsAss = ArrayOfElements[r];
        }

        if (!ConnectedRobotRegions[r]) {
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
            returnM[i*cols+j] =(returnM[i*cols+j] - MinV)*((2*(float)variateWeight)/(MaxV-MinV))+(1-(float)variateWeight);
        }
    }

    return  returnM;
}

void area_division::assign(vector<valarray<double>> Q)
{
    BWlist.resize(nr);
    for (int r=0; r<nr; r++) {
        BWlist[r].resize(rows*cols);
        BWlist[r][RobotsInit[r][1] * cols + RobotsInit[r][0]] = 1;
    }

    ArrayOfElements.clear();
    ArrayOfElements.resize(nr);
    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols; j++) {
            // unknown grid cell (treat as free)
            if (GridEnv[i*cols+j] == -1) {
                double minV = Q[0][i*cols+j];
                int indMin = 0;
                for (int r=1; r<nr; r++) {
                    if (Q[r][i*cols+j] < minV) {
                        minV = Q[r][i*cols+j];
                        indMin = r;
                    }
                }
                A[i*cols+j] = indMin;
                BWlist[indMin][i*cols+j] = 1;
                ArrayOfElements[indMin]++;
            }

            // obstacle
            else if (GridEnv[i*cols+j] < numeric_limits<signed char>::max()) {
                A[i*cols+j] = nr;
            }
        }
    }
}
