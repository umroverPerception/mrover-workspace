#ifndef PERCEP_MAPPING
#define PERCEP_MAPPING

#include <vector>
#include "rover_msgs/Odometry.hpp"

const std::size_t DEFAULT_OCCUPANCY_MAP_HEIGHT = 10000;
const std::size_t DEFAULT_OCCUPANCY_MAP_WIDTH = 10000;

const double DEFAULT_FOV = 90.0;

const double CELL_DISTANCE = 0.4 //0.4 meters
const int MAX_FILTER_LENGTH = 7 //7 meters

class OccupancyMap {
private:
    //Vector of vectors that stores the log odds of occupacy for each cell as a float.
    std::vector<std::vector<float> > occupancyMap;
    
public:
    //Default Constructor for OccupancyMap Class, resizes the vector of vectors to the default map height and width.
    OccupancyMap();

    //Overloaded Constructor for OccupancyMap Class, resizes the vector of vectors to the inputed height and width.
    OccupancyMap(int length, int width);
}

class MapSegmentation {
private:
    enum class Quadrant {
        QuadrantOne,
        QuadrantTwo,
        QuadrantThree,
        QuadrantFour
    };

    double heading, upperFOV, lowerFOV;
    /* [quadrant] -> rowStart, colStart, rowEnd, colEnd... This vector can have either 8 or 12 elements depending on how
    many quadrants  the total FOV would take up */
    std::vector<int> indexCorners;
    Quadrant headingQuadrant, upperFOVQuadrant, lowerFOVQuadrant;

    //Given an angle, this function determines what Quadrant a cell is in
    Quadrant getQuadrant(double &angleIn);

    //Quandrant one index calculations
    void calcQuadrantOne(double &angle, char &key);

    //Quandrant two index calculations
    void calcQuadrantTwo(double &angle, char &key);

    //Quandrant three index calculations
    void calcQuadrantThree(double &angle, char &key);

    //Quandrant four index calculations
    void calcQuadrantFour(double &angle, char &key);

    //calculates the segmentation for each quadrant and fills the indexCorners vector
    void fillIndexCorners();
public:
    //Default Constructor for MapSegmentation Class.
    MapSegmentation();

    //returns the indexCorners vector
    std::vector<int> getIndexCorners();
}

class Mapping {
private:
    OccupancyMap map;
    Odometry previousOdometry;
    int roverXCoordsInOccupancyMap, roverYCoordsInOccupancyMap;
    double orientationAngle;
    double cellDistance;
    bool occupied;

public:
    Mapping() : occupied(false);

    void updatePositionInOccupancyMap(Odometry &currentOdometry);
    
    void updateOrientation (double &orientationAngle);

    void getMapArea();

    void getMapArea(double &angle, double &FOV);

    void updateOccupancy (size_t xIndex, size_t yIndex);
}

/*class OccupancyMap {
    private:
        double CONVERSION_FACTOR = 0.004;
        const double INITIALIZE_VECTOR_PROBABILITY = 0.5;
        std::vector<std::vector<char> > occupancyMap;
        double occupancyMapHeight;
        double occupancyMapWidth;
        double previousLatitude, previousLongitude;
        const double DEGREES_TO_METERS_CONVERSION_FACTOR = 111317.0997;
        double sizeOfOneCellInMeters = 0.3;
        currentPositionInOccupancyMapHeight, currentPositionInOccupancyMapLength;

        
    public:
        //default constructor for occupancy map
        OccupancyMap();

        //converts double to char
        char doubleToChar(double &input);
        
        //converts char to double
        double charToDouble(char &input);

        //gets height of occupancyMap
        int getMapHeight();

        //gets width of occupancyMap
        int getMapWidth();

        //fills occupancyMap with a given char "toFillWith"
        void fillOccupanyMap(char &toFillWith);

        //converts deg min into degrees
        double convertDegreesToMinutes(int &degreeMin);

        //converts degrees to meteres
        //based on NASA's diameter of earth 
        //https://imagine.gsfc.nasa.gov/features/cosmic/earth_info.html
        //and then math
        const double convertDegreesToMeters(double degrees) {
            return degrees * 111317.0997;
        }

        //gets the change in latitude in meters
        double changeInLatitudeMeters(double currentLatitude);
    

        //gets the change in latitude in meters
        double changeInLongitudeMeters(double currentLongitude);

        //updates robot position
        void updatePosition()

};*/
#endif
