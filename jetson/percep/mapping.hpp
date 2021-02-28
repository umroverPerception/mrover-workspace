#pragma once

#include <vector>
#include <cmath>

class OccupancyMap {
    private:
        double CONVERSION_FACTOR = 0.004;
        const double INITIALIZE_VECTOR_PROBABILITY = 0.5;
        std::vector<std::vector<char> > occupancyMap;
        double occupancyMapHeight;
        double occupancyMapWidth;

        
    public:
        //default constructor for occupancy map
        OccupancyMap();

        //converts double to char
        char doubleToChar(double input);
        
        //converts char to double
        double charToDouble(char input);

        //gets height of occupancyMap
        int getHeightOfMap();

        //gets width of occupancyMap
        int getWidthOfOccupancyMap();

        //fills occupancyMap with a given char "toFillWith"
        void fillOccupanyMap(std::vector<std::vector<char> > &occupancyMap, char toFillWith);
};