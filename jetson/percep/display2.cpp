#include "display2.hpp"

using namespace cv;
using namespace std;


// Constructor
Display::Display(string in_windowName, bool outputSrc_in) 
    : windowName(in_windowName), outputSrc(outputSrc_in) {
    namedWindow(windowName);
    imshow(windowName, img);
    waitKey(30);

  
    
}

// Private member functions
void Display::clearDisplay(){
    string inputText, statsText;
    int yValue = 20;

    for(auto &stats : allData){
      statsText = to_string(stats.second);
      inputText = stats.first + ": " + statsText;
      putText(img, inputText, Point(5, yValue), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
      yValue += 20;
    }
}

// Public member functions
void Display::show(){
    //clearDisplay();
    string inputText, statsText;
    int yValue = 20;

    for(auto &stats : allData){
        statsText = to_string(stats.second);
        inputText = stats.first + ": " + statsText;
        putText(img, inputText, Point(5, yValue), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);
        yValue += 20;
    }

    imshow(windowName, img);
    waitKey(1);
}

void Display::insert(string displayName, InsertType dataType, double outputVal){
    if(dataType == InsertType::Text){
        allData.insert({displayName, outputVal});
    }
    else if(dataType == InsertType::Time){
        cout << "Timer cannot be called with decimal value. Either use "
            << "current or duration as 3rd parameter\n";
    }
}

void Display::insert(string displayName, InsertType dataType, Timer timerType){
    if(dataType == InsertType::Time){
        if(timerType == Timer::Current){
            auto current = steady_clock::now();

            timerData.insert({displayName, current});
        }
        else if(timerType == Timer::Duration){
            auto end = steady_clock::now();

            auto it = timerData.find(displayName);
            if(it == timerData.end()){
                cout << "Timer never started for " << displayName 
                    << ". Cannot find duration\n";
                return;
            }

            auto start = it->second;
            double time_diff = std::chrono::duration<double, std::ratio<1,1000>>(end - start).count();
            
            if(outputSrc){
                //timeLogsFile << it->first << ": " << time_diff << " ms" << endl;
                cout << it->first << ": " << time_diff << " ms" << endl;
            }
          
            

            allData.insert({displayName, time_diff});
        }
    }
    else if(dataType == InsertType::Text){
        cout << "Text cannot insert Timer::current or Timer::duration\n";
    }
}

void Display::clear(){
    clearDisplay();
    allData.clear();
    timerData.clear();
}

// Destructor
Display::~Display(){
    destroyWindow(windowName);
}

