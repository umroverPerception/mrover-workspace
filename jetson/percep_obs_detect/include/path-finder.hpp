#include "common.hpp"

/**
 * \class CompareLine
 * \brief GPU Functor that finds out where a point is relative to a line
 */
class CompareLine {
public:
    int xIntercept;
    float slope;
    
    /**
     * \brief CompareLine Constructor for Device
     * \param angle_in Angle of the line the constructor is modeling
     * \param xInt_in X intercept of the line
     */ 
    __device__ CompareLine(float angle_in, int xInt_in) : xIntercept{xInt_in}, 
                        slope{tan(angle_in*PI/180)} {
                            if(slope != 0) {
                                slope = 1/slope;
                            }
                        }

    /**
     * 
     */
    //Returns 1 if point is right of line, 0 if on, -1 if left of line
    __device__ int operator()(float x, float y) {
        
        //Make sure don't divide by 0
        float xc = xIntercept; //x calculated
        if(slope != 0) {
            xc = y/slope+xIntercept; //Find x value on line with same y value as input point
        }
            
        //Point is right of the line
        if(x > xc) {
            return 1;
        }
        //Point is on the line
        else if (x == xc) {
            return 0;
        }
        //Point is left of the line
        else {
            return -1;
        } 
    }

    //Assumes x1 < x2
    __device__ bool operator()(float x1, float y1, float x2, float y2) {
        if(x1 != x2){
            if(slope != 0){
                float slopeSeg = (y2-y1)/(x2-x1);
                float xIntersect = (-slopeSeg*x1+y1-xIntercept)/(slope-slopeSeg);
                return (xIntersect < x2 && xIntersect > x1);
            }
            //Check if left of line and right of line if slope is undefined
            else if(this->operator()(x1,y1) < 1 && this->operator()(x2,y2) > -1) return true; 
        }
        return false;
        
    }
};

class PathFinder {
    public:

        // Member Variables
        float leftBearing; // Left bearing of a clear path
        float rightBearing; // Right bearing of a clear path

        int findNextLargestSquare(int num);

        /**
         * \brief Device function to find leftmost and rightmost paths 
         */
        __global__ void findAngleOffCenterKernel(float* minXG, float* maxXG, float* minZG, 
                                                 float* maxZG, int numClusters, int* bearing, int direction);

        /**
         * \brief Device function to check if center path is clear or not
         */
        __global__ void findClearPathKernel(float* minXG, float* maxXG, float* minZG, 
                                            float* maxZG, int numClusters, int* leftBearing, int* rightBearin;

        

}