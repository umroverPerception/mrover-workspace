#if PERCEPTION_DEBUG
#include "perception.hpp"

using namespace cv;
using namespace std;

// Constructor
Display:: Display(string in_windowName) : windowName(in_windowName) {
    namedWindow(windowName);
    imshow(windowName, img);
    waitKey(30);
}

// Private member functions
void Display::clearDisplay(){
    string inputText, statsText;
    int yValue = 20;

    for(auto &stats : inputStats){
      statsText = to_string(stats.second);
      inputText = stats.first + statsText;
      putText(img, inputText, Point(5, yValue), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
      yValue += 20;
    }
}

// Public member functions
void Display::updateDisplay(map<string, double> inputs){
    clearDisplay();

    inputStats = inputs;
    string inputText, statsText;
    int yValue = 20;

    for(auto &stats : inputStats){
    statsText = to_string(stats.second);
    inputText = stats.first + statsText;
    putText(img, inputText, Point(5, yValue), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);
    yValue += 20;
    }

    imshow(windowName, img);
    waitKey(1);
}

// Destructor
Display::~Display(){
    destroyWindow(windowName);
}
#endif
