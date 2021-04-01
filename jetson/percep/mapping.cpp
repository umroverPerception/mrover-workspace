#include "mapping.hpp"
#include "utilities.hpp"
#include "rover_msgs/Odometry.hpp"

#include <iostream>
#include <vector>
#include <cmath>

OccupancyMap::OccupancyMap(int height, int width) : occupancyMapHeight(height), occupancyMapWidth(width) {
    /*occupancyMap.resize(occupancyMapHeight);
    for (std::size_t i = 0; i < occupancyMap.size(); ++i) {
        occupancyMap[i].resize(occupancyMapWidth);
    } 
    
    charToFill = static_cast<char>(doubleToFill);*/
    occupancyMap = occupancyMap(occupancyMapHeight, std::vector(occupancyMapWidth, 0));
}

OccupancyMap::OccupancyMap() {
    //OcccupancyMap(DEFAULT_OCCUPANCY_MAP_HEIGHT, DEFAULT_OCCUPANCY_MAP_WIDTH, 0.5);
    occupancyMap = occupancyMap(DEFAULT_OCCUPANCY_MAP_HEIGHT, std::vector(DEFAULT_OCCUPANCY_MAP_WIDTH, 0));
}

/*void OccupancyMap::fillOccupanyMap() {
    for(std::size_t height = 0; width < occupancyMapHeight; height++) {
        for(std::size_t width = 0; width < occupancyMapWidth; width++) {
            occupancyMap[height][width] = charToFill;
        }
    }
}*/
/*
double& OccupancyMap::operator()(int row, int col) {
    return occupancyMap[row][col] * CONVERSION_FACTOR;
}

double& OccupancyMap::operator=(double &value) {
    return ceil(value / CONVERSION_FACTOR);
}*/

Mapping::Mapping() {
    //defaultCellDistanceRepresentation
    //cellDistance = 0.3; //0.3 meters
    OccupancyMap occupanceMap;
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

        roverXCoordsInOccupancyMap += ceil(changeInPositionWidth/CELL_DISTANCE);
        roverYCoordsInOccupancyMap += ceil(changeInPositionHeight/CELL_DISTANCE);
        
        //then update odometry
        previousOdometry = currentOdometry;
}

void Mapping::updateOrientation(double &currentAngle) {
    orientationAngle = currentAngle;
}

void Mapping::getMapArea(double &angle, double &FOV) {

    if (angle  < 0) {
        angle = 360.0 - (90.0 - angle);
    }
    else {
        angle = 90.0 - angle;
    }

    
    
    /*if (orientationAngle >= 0.0 && orientationAngle < 45.0) {
        rowStart = roverYCoordsInOccupancyMap + ceil(cos(orientationAngle - ())/CELL_DISTANCE);
        rowEnd = roverYCoordsInOccupancyMap - ceil(cos(135.0 - orientationAngle)/CELL_DISTANCE);

        colStart = roverXCoordsInOccupancyMap;
        colEnd = roverXCoordsInOccupancyMap + ceil(MAX_FILTER_LENGTH/CELL_DISTANCE);
    }
    else if (orientationAngle == 45.0) {
        rowStart = roverYCoordsInOccupancyMap + ceil(MAX_FILTER_LENGTH/CELL_DISTANCE);
        rowEnd = overYCoordsInOccupancyMap;

        colStart = roverXCoordsInOccupancyMap;
        colEnd = roverXCoordsInOccupancyMap + ceil(MAX_FILTER_LENGTH/CELL_DISTANCE);
    }
    else if (orientationAngle > 45.0  orientationAngle < 135.0) {
        rowStart = roverYCoordsInOccupancyMap + ceil(MAX_FILTER_LENGTH/CELL_DISTANCE);
        rowEnd = overYCoordsInOccupancyMap;

        colStart = roverXCoordsInOccupancyMap - ceil(cos(135.0 - orientationAngle)/CELL_DISTANCE);
        colEnd = roverXCoordsInOccupancyMap + ceil(cos(45.0 - orientationAngle)/CELL_DISTANCE)
    }*/
    
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