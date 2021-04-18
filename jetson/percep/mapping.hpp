#ifndef PERCEP_MAPPING
#define PERCEP_MAPPING

#include <vector>
#include "rover_msgs/Odometry.hpp"
#include "mapping.hpp"

using namespace rover_msgs;

const std::size_t DEFAULT_OCCUPANCY_MAP_HEIGHT = 10000;
const std::size_t DEFAULT_OCCUPANCY_MAP_WIDTH = 10000;
const float LOG_ODDS_OCCUPIED = 0.65;
const float LOG_ODDS_UNOCCUPIED = 1 - LOG_ODDS_OCCUPIED;

const double DEFAULT_FOV = 90.0;

const double CELL_DISTANCE = 0.4; //0.4 meters
const int MAX_FILTER_LENGTH = 7; //7 meters

class OccupancyMap {
private:
    //Vector of vectors that stores the log odds of occupacy for each cell as a float.
    std::vector<std::vector<float> > occupancyMap;
    
public:
    //Default Constructor for OccupancyMap Class, resizes the vector of vectors to the default map height and width.
    OccupancyMap();

    //Overloaded Constructor for OccupancyMap Class, resizes the vector of vectors to the inputed height and width.
    OccupancyMap(int length, int width);
};

class MapSegmentation {
private:
    enum class Quadrant { QuadrantOne, QuadrantTwo, QuadrantThree, QuadrantFour };

    double heading, upperFOV, lowerFOV;
    /* [quadrant] -> rowStart, colStart, rowEnd, colEnd... This vector can have either 8 or 12 elements depending on how
    many quadrants  the total FOV would take up */
    std::vector<int> indexCorners;
    Quadrant headingQuadrant, upperFOVQuadrant, lowerFOVQuadrant;

    //Given an angle, this function determines what Quadrant a cell is in
    Quadrant getQuadrant(double &angleIn);

    /*//Quandrant one index calculations
    void calcQuadrantOne(double &angle, char &key);

    //Quandrant two index calculations
    void calcQuadrantTwo(double &angle, char &key);

    //Quandrant three index calculations
    void calcQuadrantThree(double &angle, char &key);

    //Quandrant four index calculations
    void calcQuadrantFour(double &angle, char &key);*/

    //Quadrant calculator (optimized, not implemented yet)
    void calculateQuadrantCorners(double &angleInDegrees, char &key, Quadrant &quadrant);

    //calculates the segmentation for each quadrant and fills the indexCorners vector
    void fillIndexCorners();
public:
    //Default Constructor for MapSegmentation Class.
    MapSegmentation(double &headingAngle);

    //returns the indexCorners vector
    std::vector<int> getIndexCorners();

};

class Mapping {
private:
    OccupancyMap map;

    Odometry odometry;

    int roverXCoordsInOccupancyMap, roverYCoordsInOccupancyMap;

    double orientationAngle;

    void updatePositionInOccupancyMap(Odometry &currentOdometry);
    
    void updateOrientation (double &orientationAngle);

    void getMapArea();

    void updateOccupancyValues (size_t xIndex, size_t yIndex, bool &occupied);

public:
    Mapping();

    double getHeadingAngle();

    void updateOccupancyGrid(std::vector<int> &obstacleData);
};


class MappingMath {
public: 
    //custom math function for mapping
    //returns angle in degrees
    double cos(double &angleInRadians);
    double sin(double &angleInRadians);
};
#endif
