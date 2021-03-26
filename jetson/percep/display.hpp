#if PERCEPTION_DEBUG
#include "perception.hpp"

using namespace cv;
using namespace std;

class Display{
  private:
  Mat img{Mat(250, 400, CV_8UC3, Scalar(0,0,0))};
  string windowName;
  map<string, double> inputStats;

  void clearDisplay();

  public:
    Display(string in_windowName);
  
    void updateDisplay(map<string, double> inputs);

    ~Display();
};

#endif