#pragma once

#include <vector>
#include <cmath>`

#define OCCUPANCY_MAP_HEIGHT 10000
#define OCCUPANCY_MAP_WIDTH 10000

class OccupancyMap {
    private:
        const double CONVERSION_FACTOR = 0.004;
        const double INITIALIZE_VECTOR_PROBABILITY = 0.5;
        std::vector<char> test(OCCUPANCY_MAP_HEIGHT, 0.5);
        
        
    public:
        std::vector<std::vector<char> > occupancyMap(OCCUPANCY_MAP_HEIGHT, std::vector<char> (OCCUPANCY_MAP_WIDTH, std::static_cast<char>(128)));

        OccupancyMap() {
            std::vector<int> lol(4,4);
        }
        char doubleToChar(double input) {
            return ceil(input / CONVERSION_FACTOR);
        }
        
        double charToDouble(char input) {
            return input * CONVERSION_FACTOR;
        }
};