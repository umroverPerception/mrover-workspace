#include "perception.hpp"
#include <chrono>
#include <iostream>
#include <string>
#include <map>
#include <unordered_map>
#include <ctime>
#include <fstream>

using namespace cv;
using namespace std;
using namespace std::chrono;


// enum states for type of insertion
enum class InsertType{ Text, Time};

// enum states for timer 
enum class Timer{ Current, Duration};

class Display{
  private:
  Mat img{Mat(250, 400, CV_8UC3, Scalar(0,0,0))};
  string windowName;
  map<string, double> allData;
  unordered_map<string, steady_clock::time_point> timerData;
  fstream timeLogsFile;

  void clearDisplay();

  public:
    InsertType text = InsertType::Text;
    InsertType time = InsertType::Time;
    Timer current = Timer::Current;
    Timer duration = Timer::Duration;

    Display(string in_windowName);
  
    void show();

    void insert(string displayName, InsertType dataType, double outputVal);

    void insert(string displayName, InsertType dataType, Timer timerType);

    void clear();

    ~Display();
};