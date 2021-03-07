#include "mapping.hpp"
#include "utilities.hpp"
#include "rover_msgs/Odometry.hpp"

#include <iostream>
#include <vector>
#include <cmath>

OccupancyMap::OccupancyMap() {
    occupancyMapHeight = DEFAULT_OCCUPANCY_MAP_HEIGHT;
    occupancyMapWidth = DEFAULT_OCCUPANCY_MAP_WIDTH;
    occupancyMap.resize(occupancyMapHeight);
    for (std::size_t i = 0; i < occupancyMap.size(); ++i) {
        occupancyMap[i].resize(occupancyMapWidth);
    }
}

OccupancyMap::OccupancyMap(int height, int width, char charToFill) : occupancyMapHeight(height), occupancyMapWidth(width), charToFill(static_cast<char>(128)) {
    occupancyMap.resize(occupancyMapHeight);
    for (std::size_t i = 0; i < occupancyMap.size(); ++i) {
        occupancyMap[i].resize(occupancyMapWidth);
    } 
}

void OccupancyMap::fillOccupanyMap() {
    for(std::size_t height = 0; width < occupancyMapHeight; height++) {
        for(std::size_t width = 0; width < occupancyMapWidth; width++) {
            occupancyMap[height][width] = charToFill;
        }
    }
}

double& OccupancyMap::operator()(int row, int col) {
    return occupancyMap[row][col] * CONVERSION_FACTOR;
}

double& OccupancyMap::operator=(double &value) {
    return ceil(value / CONVERSION_FACTOR);
}

Mapping::Mapping() {
    //defaultCellDistanceRepresentation
    cellDistance = 0.3; //0.3 meters
    occupancyMap = new OccupancyMap();
    previousOdometry = new Odometry();
    roverXCoordsInOccupancyMap = 4999;
    roverXCoordsInOccupancyMap = 4999;
}

void Mapping::updatePositionInOccupancyMap(Odometry &currentOdometry) {
    double changeInDistanceFromPreviousPosition = 
            estimateNoneuclid(currentOdometry, previousOdometry);
        double changeInBearingFromPreviousPosition = 
            calcBearing(currentOdometry, previousOdometry);
        
        while(changeInBearingFromPreviousPosition < 0) {
            changeInBearingFromPreviousPosition += 360.0;
        }

        //if the bearing is between 0 and 90
        if(changeInBearingFromPreviousPosition =< 90.0) {
            changeInPositionWidth = changeInEuclideanDistanceFromPreviousPosition * 
                cos(90.0 - changeInBearingFromPreviousPosition);
            changeInPositionHeight = changeInEuclideanDistanceFromPreviousPosition * 
                sin(90.0 - changeInBearingFromPreviousPosition);
        }
        //if bearing is between 90 - 180 
        else if(changeInBearingFromPreviousPosition > 90.0 && changeInBearingFromPreviousPosition =< 180.0) {
            changeInPositionWidth = cos(changeInBearingFromPreviousPosition - 90.0);
            changeInPositionHeight = sin(changeInBearingFromPreviousPosition - 90.0);
        }
        else if(changeInBearingFromPreviousPosition > 180.0 && changeInBearingFromPreviousPosition =< 270.0) {
            changeInPositionWidth = cos(270.0 - changeInBearingFromPreviousPosition); 
            changeInPositionHeight = sin(270.0 - changeInBearingFromPreviousPosition);               
        }
        else {
            changeInPositionWidth = cos(changeInBearingFromPreviousPosition - 270.0);
            changeInPositionHeight = sin(hangeInBearingFromPreviousPosition - 270.0);
        }

        roverXCoordsInOccupancyMap += ceil(changeInPositionWidth/cellDistance);
        roverYCoordsInOccupancyMap += ceil(changeInPositionHeight/cellDistance);
        
        //then update odometry
        previousOdometry = currentOdometry;
}

/*char OccupancyMap::doubleToChar(double &input) {
    return ceil(input / CONVERSION_FACTOR);
}
        
double OccupancyMap::charToDouble(char &input) {
    return input * CONVERSION_FACTOR;
}

int OccupancyMap::getMapHeight() {
    return occupancyMapHeight;
}

int OccupancyMap::getMapWidth() {
    return occupancyMapWidth;
}

void OccupancyMap::fillOccupanyMap(char &toFillWith) {
    for(std::size_t row = 0; row < occupancyMap.size(); ++row) {
        for(std::size_t col = 0; col < occupancyMap[row].size(); ++col) {
            occupancyMap[row][col] = toFillWith;
        }
    }
}

double OccupancyMap::convertDegreesToMeters(double degrees) {
    return degrees * DEGREES_TO_METERS_CONVERSION_FACTOR;
}
double OccupancyMap::changeInLatitudeMeters(double currentLatitude) {
    return convertDegreesToMeters(previousLatitude - currentLatitude);
}

double OccupancyMap::changeInLongitudeMeters(double currentLongitude); {
    return convertDegreesToMeters(previousLongitude - currentLongitude);
}

void updatePosition() {
    currentPositionInOccupancyMapHeight =+ round(changeInLatitudeMeters(currentLatitude));
    currentPositionInOccupancyMapLength =+ round(changeInLongitudeMeters(currentLongitude));
}

int getCurrentPositionInOccupancyMapHeight() return currentPositionInOccupancyMapHeight;
int getCurrentPositionInOccupancyMapLength() return currentPositionInOccupancyMapLength;
*/