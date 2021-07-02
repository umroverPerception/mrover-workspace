#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <fstream>
#include "rover_msgs/Odometry.hpp"

using namespace rover_msgs;

MappingMath::cos(double &angleInDegrees) {
    return radianToDegree(cos(degreeToRadian(angleInDegrees, 0)));
}

MappingMath::sin(double &angleInDegrees) {
    return radianToDegree(sin(degreeToRadian(angleInDegrees, 0)));
}

OccupancyMap::OccupancyMap() {
    occupancyMap = occupancyMap(DEFAULT_OCCUPANCY_MAP_HEIGHT, std::vector(DEFAULT_OCCUPANCY_MAP_WIDTH, 0.5));
}

OccupancyMap::OccupancyMap(int height, int width) {
    occupancyMap = occupancyMap(height, std::vector(width, 0.5));
}

MapSegmentation::Quadrant MapSegmentation::getQuadrant(double &angleIn) {
    if (angleIn >= 0.0 && angleIn < 90.0) {
        return QuadrantOne;
    }
    else if (angleIn >= 90.0 && angleIn < 180.0) {
        return QuadrantTwo;
    }
    else if (angleIn >= 180.0 && angleIn < 270.0) {
        return QuadrantThree;
    }
    else if (angleIn >= 270.0 && angleIn < 360.0) {
        return QuadrantFour;
    }
}

MapSegmentation::MapSegmentation(double &headingAngle) : heading(headinAngle) {
    headingQuadrant = getQuadrant(heading);

    upperFOV = heading + (DEFAULT_FOV/2.0);
    if (upperFOV >= 360.0) {
        upperFOV = upperFOV - 360.0;
    }
    upperFOVQuadrant = getQuadrant(upperFOV);


    lowerFOV = heading - (DEFAULT_FOV/2.0);
    if (upperFOV < 0.0) {
        upperFOV = upperFOV + 360.0;
    }
    lowerFOVQuadrant = getQuadrant(lowerFOV);
}

void MapSegmentation::calculateQuadrantCorners(double &angleInDegrees, char &key, Quadrant &quadrant) {
    for(int i = 0; i < 4; i++) {
        if(quadrant == QuadrantOne) {
            if(i == 1 || i == 2) indexCorners[i] = 0;
            else if(key == 'u' && i == 0) {
                indexCorners[i] = ceil(MAX_FILTER_LENGTH * MappingMath::cos(90.0 - angle));
            }
            else if(key== 'l' && i == 3) {
                indexCorners[i] = ceil(MAX_FILTER_LENGTH * MappingMath::cos(angle));
            }
            else {
                indexCorners[i] = MAX_FILTER_LENGTH;
            }
        }
        else if(quadrant == QuadrantTwo) {
            if(i == 2 || i == 3) indexCorners[i] = 0;
            else if(key == 'u' && i == 2) {
                indexCorners[i] = ((-1) * ceil(MAX_FILTER_LENGTH * MappingMath::cos(180.0 - angle)));
            }
            else if( (key == 'h' && i == 2) || (key == 'l' && i == 2)) {
                indexCorners[i] = ((-1) * MAX_FILTER_LENGTH);
            }
            else {
                indexCorners[i] = MAX_FILTER_LENGTH;
            }
        }
        else if(quadrant == QuadrantThree) {
            if(i == 0 || i == 3) indexCorners[i] = 0;
            else if(key == 'u' && i == 2) {
                indexCorners[i] = ((-1) * ceil(MAX_FILTER_LENGTH * MappingMath::cos(360.0 - angle));
            }
            else if(key == 'l' && i == 1) {
                indexCorners[i] = ((-1) * ceil(MAX_FILTER_LENGTH * MappingMath::cos(angle - 270.0));
            }
            else {
                indexCorners[i] = ((-1) * MAX_FILTER_LENGTH);
            }
        }
        else {
            if(i == 0 || i == 1) indexCorners[i] = 0;
            else if(key =='l' && i == 2) {
                indexCorners[i] = (-1) * ceil(MAX_FILTER_LENGTH * MappingMath::cos(angle - 270.0);
            }
            else if(key == 'u' && i == 3) {
                indexCorners[i] = (ceil(MAX_FILTER_LENGTH * MappingMath::cos(360.0 - angle));
            }
            else if(key == 'u' && i == 2) { 
                indexCorners[i] = ((-1) * MAX_FILTER_LENGTH);
            }
            else {
                indexCorners[i] = MAX_FILTER_LENGTH;
            }
        }
    }
}

void MapSegmentation::fillIndexCorners() {
    indexCorners.resize(8);

    //upper FOV
    calculateQuadrantCorners(upperFOV, 'h', upperFOVQuadrant);

    //lowerFOV
    calculateQuadrantCorners(lowerFOV, 'l', lowerFOVQuadrant);

    //heading
    if ((lowerFOVQuadrant != headingQuadrant) && (upperQuadrant != headingQuadrant)) {
        calculateQuadrantCorners(heading, 'h', headingQuadrant);
        indexCorners.resize(12);
    }
}

std::vector<int> MapSegmentation::getIndexCorners() {
    fillIndexCorners();
    return indexCorners;
}

Mapping::Mapping() {
    map();
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
                MappingMath::cos(90.0 - changeInBearingFromPreviousPosition);
            changeInPositionHeight = changeInEuclideanDistanceFromPreviousPosition * 
                MappingMath::sin(90.0 - changeInBearingFromPreviousPosition);
        }
        //if bearing is between 90 - 180 
        else if(changeInBearingFromPreviousPosition > 90.0 && changeInBearingFromPreviousPosition =< 180.0) {
            changeInPositionWidth = MappingMath::cos(changeInBearingFromPreviousPosition - 90.0);
            changeInPositionHeight = MappingMath::sin(changeInBearingFromPreviousPosition - 90.0);
        }
        else if(changeInBearingFromPreviousPosition > 180.0 && changeInBearingFromPreviousPosition =< 270.0) {
            changeInPositionWidth = MappingMath::cos(270.0 - changeInBearingFromPreviousPosition); 
            changeInPositionHeight = MappingMath::sin(270.0 - changeInBearingFromPreviousPosition);               
        }
        else {
            changeInPositionWidth = MappingMath::cos(changeInBearingFromPreviousPosition - 270.0);
            changeInPositionHeight = MappingMath::sin(hangeInBearingFromPreviousPosition - 270.0);
        }

        roverXCoordsInOccupancyMap += ceil(changeInPositionWidth/CELL_DISTANCE);
        roverYCoordsInOccupancyMap += ceil(changeInPositionHeight/CELL_DISTANCE);
        
        //then update odometry
        previousOdometry = currentOdometry;
}

void Mapping::updatePositionWithTXT() {
    ifstream odom("odom.txt");
    if (odom.is_open()) {
        double lat, latMin, lon, lonMin;
        while (std::getline(odom, lat)) {
            
        }
    }
}

void Mapping::updateOrientation(double &currentAngle) {
    if (currentAngle >= 0.0 && currentAngle < 90.0) {
        orientationAngle = 90.0 - currentAngle;
    }
    else if (currentAngle >= 90.0 && currentAngle < 180.0) {
        orientationAngle = (90.0 - currentAngle) + 360.0;
    }
    else if (currentAngle < 0.0 && currentAngle >= -90.0) {
        orientationAngle = (-1 * currentAngle) + 90.0;
    }
    else if (currentAngle < -90.0 && currentAngle >= -180.0) {
        orientationAngle = (-1 * currentAngle) + 90.0;
    }
}

void Mapping::getMapArea(){

    updatePositionInOccupancyMap(odometry);
    updateOrientation(odometry.bearing_in);

    MapSegmentation segment(orientationAngle);
    std::vector<int> mapArea = segment.getIndexCorners();
}

double Mapping::getHeadingAngle() {
    return orientationAngle;
}

void Mapping::updateOccupancyGrid(std::vector<int> &obstacleData) {
    std::ifstream records;
    std::ifstream odom;
    records.open ("records.txt");
    std::string compleleted;
    int odomTime;
    std::string odomOther;
    int time;
    while (!records.eof && !odom.eof) {
        getline(records, completed);
        records >> time;

        odom >> odomOther >> odomOther >> odomOther >> odomOther >> odomOther >> odomOther >> odomTime;
        /*
        int rowLowerBound = obstacleData[0];
        int columnLowerBound = obstacleData[1];
        int rowUpperBound = obstacleData[2];
        int columnUpperBound = obstacleData[3];
        */
        if (time >= odomTime) {
            map.getMapArea();
            for (std::size_t i = 0; i < obstacleData.size();  i += 4) {
                for (int k = obstacleData[i]; k < obstacleData[i+2]; ++k) {
                    for (int j = obstacleData[i+1]; j < obstacleData[i+3]; ++j) {
                        updateOccupancyValues(k,j,true);
                    }
                }
            }
        }
        else {
            odom >> odomOther >> odomOther >> odomOther >> odomOther >> odomOther >> odomOther >> odomTime;
        }
    }
}

void Mapping::updateOccupancyValues(size_t xIndex, size_t yIndex, bool &occupied) {
    if (occupied) {
        map[xIndex][yIndex] = map[xIndex][yIndex] + (LOG_ODDS_OCCUPIED/LOG_ODDS_UNOCCUPIED);
    }
    else {
        map[xIndex][yIndex] = map[xIndex][yIndex] + (LOG_ODDS_UNOCCUPIED/LOG_ODDS_OCCUPIED); 
    }
}

/*void MapSegmentation::caclulateQuadOne(double &angle, char &key) {
    if (key == 'u') {
        indexCorners.push_back(ceil(MAX_FILTER_LENGTH * MappingMath::cos(90.0 - angle)));
        indexCorners.push_back(0);
        indexCorners.push_back(0);
        indexCorners.push_back(MAX_FILTER_LENGTH);
    }
    else if (key == 'h') {
        indexCorners.push_back(MAX_FILTER_LENGTH);
        indexCorners.push_back(0);
        indexCorners.push_back(0);
        indexCorners.push_back(MAX_FILTER_LENGTH);
    }
    else if (key == 'l') {
        indexCorners.push_back(MAX_FILTER_LENGTH);
        indexCorners.push_back(0);
        indexCorners.push_back(0);
        indexCorners.push_back(ceil(MAX_FILTER_LENGTH * MappingMath::cos(angle)));
    }
}

void MapSegmentation::caclulateQuadTwo(double &angle, char &key) {
    if (key == 'u') {
        indexCorners.push_back(MAX_FILTER_LENGTH);
        indexCorners.push_back((-1) * ceil(MAX_FILTER_LENGTH * MappingMath::cos(180.0 - angle)));
        indexCorners.push_back(0);
        indexCorners.push_back(0);
    }
    else if (key == 'h') {
        indexCorners.push_back(MAX_FILTER_LENGTH);
        indexCorners.push_back((-1) * MAX_FILTER_LENGTH);
        indexCorners.push_back(0);
        indexCorners.push_back(0);
    }
    else if (key == 'l') {
        indexCorners.push_back(ceil(MAX_FILTER_LENGTH * MappingMath::cos(angle - 90.0)));
        indexCorners.push_back((-1) * MAX_FILTER_LENGTH);
        indexCorners.push_back(0);
        indexCorners.push_back(0);
    }
}

void MappingSegmentation::calulateQuadThree(double &angle, char &key) {
    if (key == 'u') {
        indexCorners.push_back(0);
        indexCorners.push_back((-1) * MAX_FILTER_LENGTH);
        indexCorners.push_back((-1) * ceil(MAX_FILTER_LENGTH * MappingMath::cos(360.0 - angle));
        indexCorners.push_back(0);
    }
    else if (key == 'h') {
        indexCorners.push_back(0);
        indexCorners.push_back((-1) * MAX_FILTER_LENGTH);
        indexCorners.push_back((-1) * MAX_FILTER_LENGTH);
        indexCorners.push_back(0);
    }
    else if (key == 'l') {
        indexCorners.push_back(0);
        indexCorners.push_back((-1) * ceil(MAX_FILTER_LENGTH * MappingMath::cos(angle - 270.0));
        indexCorners.push_back((-1) * MAX_FILTER_LENGTH);
        indexCorners.push_back(0);
    }
}

void MappingSegmentation::caclulateQuadFour(double &angle, char &key) {
    if (key == 'u') {
        indexCorners.push_back(0);
        indexCorners.push_back(0);
        indexCorners.push_back((-1) * MAX_FILTER_LENGTH);
        indexCorners.push_back(ceil(MAX_FILTER_LENGTH * MappingMath::cos(360.0 - angle));
    }
    else if (key == 'h') {
        indexCorners.push_back(0);
        indexCorners.push_back(0);
        indexCorners.push_back((-1) * MAX_FILTER_LENGTH);
        indexCorners.push_back(MAX_FILTER_LENGTH);
    }
    else if (key == 'l') {
        indexCorners.push_back(0);
        indexCorners.push_back(0);
        indexCorners.push_back((-1) * ceil(MAX_FILTER_LENGTH * MappingMath::cos(angle - 270.0);
        indexCorners.push_back(MAX_FILTER_LENGTH);
    }
}

void MapSegmentation::fillIndexCorners() {
    //upperFOV
    if (upperFOVQuadrant = QuadrantOne) {
        caclulateQuadOne(upperFOV, 'u');
    }
    else if (upperFOVQuadrant = QuadrantTwo) {
        caclulateQuadTwo(upperFOV, 'u');
    }
    else if (upperFOVQuadrant = QuadrantThree) {
        caclulateQuadThree(upperFOV, 'u');
    }
    else if (upperFOVQuadrant = QuadrantFour) {
        caclulateQuadFour(upperFOV, 'u');
    }

    //lowerFOV
    if (lowerFOVQuadrant = QuadrantOne) {
        caclulateQuadOne(lowerFOV, 'l');
    }
    else if (lowerFOVQuadrant = QuadrantTwo) {
        caclulateQuadTwo(lowerFOV, 'l');
    }
    else if (lowerFOVQuadrant = QuadrantThree) {
        caclulateQuadThree(lowerFOV, 'l');
    }
    else if (lowerFOVQuadrant = QuadrantFour) {
        caclulateQuadFour(lowerFOV, 'l');
    }

    //heading
    if ((lowerFOVQuadrant != headingQuadrant) && (upperQuadrant != headingQuadrant)) {
        if (headingQuadrant = QuadrantOne) {
        caclulateQuadOne(heading, 'h');
    }
    else if (headingQuadrant = QuadrantTwo) {
        caclulateQuadTwo(heading, 'h');
    }
    else if (headingQuadrant = QuadrantThree) {
        caclulateQuadThree(heading, 'h');
    }
    else if (headingQuadrant = QuadrantFour) {
        caclulateQuadFour(heading, 'h');
    }
    }
}*/