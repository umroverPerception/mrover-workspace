#ifndef PERCEP_MAPPING
#define PERCEP_MAPPING

#include <vector>
#include "rover_msgs/Odometry.hpp"

const std::size_t DEFAULT_OCCUPANCY_MAP_HEIGHT = 10000;
const std::size_t DEFAULT_OCCUPANCY_MAP_WIDTH = 10000;
const double CONVERSION_FACTOR = 0.004;

class OccupancyMap {
private:
    std::vector<std::vector<char> > occupancyMap;
    size_t occupancyMapHeight, occupancyMapWidth;
    char charToFill;
    
    void fillOccupancyMap();
public:
    //default constructor for occupancy map
    OccupancyMap();

    //constructor with given length and width
    OccupancyMap(int length, int width, double doubleToFill);

    double &operator()(int height, int width);

    double &operator=(double &value);
}

class Mapping {
private:
    OccupancyMap map;
    Odometry previousOdometry;
    int roverXCoordsInOccupancyMap, roverYCoordsInOccupancyMap;
    double cellDistance;

public:
    Mapping();

    void updatePositionInOccupancyMap(Odometry &currentOdometry);
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
