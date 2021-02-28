#include 'mapping.hpp'

#define DEFAULT_OCCUPANCY_MAP_HEIGHT 10000
#define DEFAULT_OCCUPANCY_MAP_WIDTH 10000

OccupancyMap::OccupancyMap() {
    occupancyMapHeight = DEFAULT_OCCUPANCY_MAP_HEIGHT;
    occupancyMapWidth = DEFAULT_OCCUPANCY_MAP_WIDTH;
    fillOccupanyMap(occupancyMap, doubleToChar(0.5));
}
        
char OccupancyMap::doubleToChar(double input) {
    return ceil(input / CONVERSION_FACTOR);
}
        
double OccupancyMap::charToDouble(char input) {
    return input * CONVERSION_FACTOR;
}

int OccupancyMap::getHeightOfMap() {
    return occupancyMap.size();
}

int OccupancyMap::getWidthOfOccupancyMap() {
    return occupancyMap.at(0).size();
}

void OccupancyMap::fillOccupanyMap(std::vector<std::vector<char> > &occupancyMap, char toFillWith) {
    for(int row = 0; row < getHeightOfMap(); row++) {
        for(int col = 0; col < getWidthOfOccupancyMap(row).size(); col++) {
            occupancyMap.at(row).at(col).pushback(toFillWith);
        }
    }
}