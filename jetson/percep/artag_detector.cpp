#include "perception.hpp"

static Mat HSV;
static Mat DEPTH;
/* For debug use: print the HSV values at mouseclick locations */
void onMouse(int event, int x, int y, int flags, void *userdata) {
    if (event == EVENT_LBUTTONUP) {
        Vec3b p = HSV.at<Vec3b>(y, x);
        float d = DEPTH.at<float>(y, x);
        printf(
            "Get mouse click at (%d, %d), HSV value is H: %d, S: %d, V:%d, "
            "depth is %.2f meters \n",
            y, x, p.val[0], p.val[1], p.val[2], d);
    }
}

TagDetector::TagDetector() {  //initializes detector object with pre-generated dictionary of tags

    cv::FileStorage fsr("jetson/percep/alvar_dict.yml", cv::FileStorage::READ);
    if (!fsr.isOpened()) {  //throw error if dictionary file does not exist
        std::cerr << "ERR: \"alvar_dict.yml\" does not exist! Create it before running main\n";
        throw Exception();
    }

    // read dictionary from file
    int mSize, mCBits;
    cv::Mat bits;
    fsr["MarkerSize"] >> mSize;
    fsr["MaxCorrectionBits"] >> mCBits;
    fsr["ByteList"] >> bits;
    fsr.release();
    alvarDict = new cv::aruco::Dictionary(bits, mSize, mCBits);

    // initialize other special parameters that we need to properly detect the URC (Alvar) tags
    alvarParams = new cv::aruco::DetectorParameters();
    alvarParams->markerBorderBits = 2;
    alvarParams->doCornerRefinement = false;
    alvarParams->polygonalApproxAccuracyRate = 0.08;
}

Point2f TagDetector::getAverageTagCoordinateFromCorners(const vector<Point2f> &corners) {  //gets coordinate of center of tag
    // RETURN:
    // Point2f object containing the average location of the 4 corners
    // of the passed-in tag
    Point2f avgCoord;
    for (auto &corner : corners) {
        avgCoord.x += corner.x;
        avgCoord.y += corner.y;
    }
    avgCoord.x /= corners.size();
    avgCoord.y /= corners.size();
    return avgCoord;
}

pair<Tag, Tag> TagDetector::findARTags(Mat &src, Mat &depth_src, Mat &rgb) {  //detects AR tags in source Mat and outputs Tag objects for use in LCM
    // RETURN:
    // pair of target objects- each object has an x and y for the center,
    // and the tag ID number return them such that the "leftmost" (x
    // coordinate) tag is at index 0
    cvtColor(src, rgb, COLOR_RGBA2RGB);
    // clear ids and corners vectors for each detection
    ids.clear();
    corners.clear();
<<<<<<< HEAD

    Mat gray;
    cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    /*------------------------------------------CODE WALKTHROUGH------------------------------------------
    For a walkthrough of how to test any code that you add, scroll down to the TESTING WALKTHROUGH section.
    Here are the different methods that we have tried in order to detect AR-Tags on a Dark/Varied Background:
    1. Image Thresholding -> The general idea behind image thresholding is that if a pixel is lighter than 
    some color (in our case pretty dark) then it should be set to white. What that means for the purposes of 
    our image is that everything should be white except pixels that are as dark or darker than the black squares
    of an AR Tag. This code had some issues, but another set of eyes might be able to get it to work.
    
    2. Image Saturation/Sharpening -> The idea here is to increase to difference between various colors in order to make
    the edges of the AR Tag sharper and thus give the detectMarkers function a better chance of picking up the AR Tag.
    This solution worked reasonably well, but I'm concerned that it's use case is limited or that its dependent on the amount
    of light already in the image.

    3. Contouring -> With this method, we are trying to detect edges ourselves using the Canny Edge Detection Algorithm.
    After doing this we impose those edges in white on top of the image essentially giving us a pseudo white background 
    for any AR Tag. Additionally there is code in this method for taking the edges and detecting squares/rectangles in order
    to only impose square or rectangular edges as opposed to all of them. This hasn't been working particularly well so far
    but their might be something that I'm missing. The Method that has worked the best so far for filtering out unnecessary edges
    was to look at the size of the contour and if it was too small, exclude it.

    4. OpenCV Rejects -> The OpenCV detectMarkers function by default returns a list of rejected candidates as well. I thought
    I would try drawing white borders around those rejected candidates, to see if the reason detectMarkers rejected them was
    because they didn't have a white background. This didn't work very well when I tried it, and it would potentially be inefficient
    as 2 calls to detectMarkers would be required.
    ------------------------------------------CODE WALKTHROUGH------------------------------------------*/
    /*------------------------------------THRESHOLDING CODE (Doesn't Work)-----------------------------------------------------*/
    //Apply Custom thresholding in order to detect AR Tags with no outline
    Mat threshold;
    const double threshold_val = 5; //If rgb is greater than threshold_val then set to white, otherwise set to black
    const double threshold_max = 255;
    const int threshold_type = 0; //Binary Thresholding (described above)
    cv::threshold(gray, threshold, threshold_val, threshold_max, threshold_type);
    /*------------------------------------THRESHOLDING (Doesn't Work)-----------------------------------------------------*/
    /*------------------------------------IMAGE SATURATION---------------------------------------------*/
    Mat sat_image = Mat::zeros(rgb.size(), rgb.type());
    double alpha = 3; /*< Simple contrast control */
    int beta = 0;

    for( int y = 0; y < rgb.rows; y++ ) {
        for( int x = 0; x < rgb.cols; x++ ) {
            for( int c = 0; c < rgb.channels(); c++ ) {
                if(rgb.channels() == 0) {cout << "Wack\n";}
                sat_image.at<Vec3b>(y,x)[c] =
                    saturate_cast<uchar>( alpha*rgb.at<Vec3b>(y,x)[c] + beta );
            }
        }
    }
    /*------------------------------------IMAGE SATURATION---------------------------------------------*/
    /*------------------------------------CONTOURING-----------------------------------------------------*/
    Mat canny_output;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> accepted;
    std::vector<Vec4i> hierarchy;
    int thresh = 15;
    Canny(gray, canny_output, thresh, thresh * 3, 3);
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    //You can only use SIZE BASED DETECTION or SQUARE DETECTION -> Use one and comment out the other
    std::vector<std::vector<Point>>poly(contours.size());
    for (int i = 0; i < contours.size(); ++i) {
        cv::approxPolyDP(Mat(contours[i]), poly[i], 0.01*arcLength(Mat(contours[i]), true), true);
        /*--------------SIZE BASED DETECTION (Comment out the if statement and closing brace but not the line 
        inside the if statement if you want to impose all edges)--------------*/
        if (poly[i].size() < 20 && contourArea(contours[i]) > 75) {
            accepted.push_back(contours[i]);
        }
        /*--------------SIZE BASED DETECTION (Comment out the if statement and closing brace but not the line 
        inside the if statement if you want to impose all edges)--------------*/
        /*--------------SQUARE DETECTION (Comment out the if statement and closing brace but not the line 
        inside the if statement if you want to impose all edges)--------------*/
        //if (poly[i].size() == 4){
            //accepted.push_back(contours[i]);
        //}
        /*--------------SQUARE DETECTION (Comment out the if statement and closing brace but not the line 
        inside the if statement if you want to impose all edges)--------------*/
    }

    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    for (int i = 0; i < accepted.size(); i++) {
        Scalar color = Scalar(255, 255, 255);
        int thickness = int(accepted[i].size() * 0.001);
        drawContours(drawing, accepted, i, color, 2, 8, hierarchy, 0);
    }
    Mat final = rgb + drawing;
    /*------------------------------------CONTOURING-----------------------------------------------------*/
    /*------------------------------------REJECTS-----------------------------------------------------*/
    std::vector<std::vector<cv::Point2f> > rejected;
    /*------------------------------------REJECTS-----------------------------------------------------*/
    /*------------------------------------TESTING WALKTHROUGH-----------------------------------------------------
    In order to test Your Code, there are three lines that you need to be aware of:
    1. The first is directly beneath this comment -> cv::aruco::detectMarkers. The first parameter in this function
    is the Matrix(Image) that the Detector Function will take in to find ARtags.

    2. The second and third are right next to one another. Under the preprocessor Directive #if PERCEPTION_DEBUG a little bit further
    down the file. They are cv::aruco::drawDetectedMarkers and cv::imshow. The first parameter in drawDetectedMarkers and the second parameter
    in imshow should be the same Matrix(Image). They should both be the image you actually want to see in the window that opens up while
    you are debugging.
    ------------------------------------TESTING WALKTHROUGH-----------------------------------------------------*/
    /*------------------------------------TAG DETECTION (Necessary for All Methods)-----------------------------------------------------*/
    cv::aruco::detectMarkers(final, alvarDict, corners, ids, alvarParams, rejected);
    /*------------------------------------TAG DETECTION (Necessary for All Methods)-----------------------------------------------------*/
    /*------------------------------------REJECTS CONTINUED-----------------------------------------------------*/
    Mat drawing2 = Mat::zeros(rgb.size(), CV_8UC3);
    for(int i = 0; i < rejected.size(); ++i){
        double distOne = cv::norm(rejected[i][0]-rejected[i][1]);
        double distTwo = cv::norm(rejected[i][0]-rejected[i][2]);
        double distThree = cv::norm(rejected[i][0]-rejected[i][3]);
        double maxDist = std::max(std::max(distOne, distTwo), distThree);
        int index = 0;
        if(maxDist == distThree){
            index = 3;
        }else if(maxDist == distTwo){
            index = 2;
        }else{ 
            index = 1;
        }
        Scalar color = Scalar(255, 255, 255);
        cv::rectangle(drawing2, rejected[i][0], rejected[i][index], color, 3);
    }

    Mat final2 = rgb + drawing2;
    /*------------------------------------REJECTS CONTINUED-----------------------------------------------------*/

    
#if AR_RECORD
cv::aruco::drawDetectedMarkers(rgb, corners, ids);
#endif
#if PERCEPTION_DEBUG
    // Draw detected tags
    cv::aruco::drawDetectedMarkers(final, corners, ids);
    cv::imshow("AR Tags", final);

    // on click debugging for color
    DEPTH = depth_src;
    cvtColor(rgb, HSV, COLOR_RGB2HSV);
    setMouseCallback("Obstacle", onMouse);
#endif

    // create Tag objects for the detected tags and return them
    pair<Tag, Tag> discoveredTags;
    if (ids.size() == 0) {
        // no tags found, return invalid objects with tag set to -1
        discoveredTags.first.id = -1;
        discoveredTags.first.loc = Point2f();
        discoveredTags.second.id = -1;
        discoveredTags.second.loc = Point2f();

    } else if (ids.size() == 1) {  // exactly one tag found
        discoveredTags.first.id = ids[0];
        discoveredTags.first.loc = getAverageTagCoordinateFromCorners(corners[0]);
        // set second tag to invalid object with tag as -1
        discoveredTags.second.id = -1;
        discoveredTags.second.loc = Point2f();
    } else if (ids.size() == 2) {  // exactly two tags found
        Tag t0, t1;
        t0.id = ids[0];
        t0.loc = getAverageTagCoordinateFromCorners(corners[0]);
        t1.id = ids[1];
        t1.loc = getAverageTagCoordinateFromCorners(corners[1]);
        if (t0.loc.x < t1.loc.x) {  //if tag 0 is left of tag 1, put t0 first
            discoveredTags.first = t0;
            discoveredTags.second = t1;
        } else {  //tag 1 is left of tag 0, put t1 first
            discoveredTags.first = t1;
            discoveredTags.second = t0;
        }
    } else {  // detected >=3 tags
        // return leftmost and rightsmost detected tags to account for potentially seeing 2 of each tag on a post
        Tag t0, t1;
        t0.id = ids[0];
        t0.loc = getAverageTagCoordinateFromCorners(corners[0]);
        t1.id = ids[ids.size() - 1];
        t1.loc = getAverageTagCoordinateFromCorners(corners[ids.size() - 1]);
        if (t0.loc.x < t1.loc.x) {  //if tag 0 is left of tag 1, put t0 first
            discoveredTags.first = t0;
            discoveredTags.second = t1;
        } else {  //tag 1 is left of tag 0, put t1 first
            discoveredTags.first = t1;
            discoveredTags.second = t0;
        }
    }
    return discoveredTags;
}