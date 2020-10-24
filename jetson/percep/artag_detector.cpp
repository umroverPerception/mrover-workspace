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
    
    /*------------------------------------THRESHOLDING CODE (Doesn't Work)-----------------------------------------------------*/
    //Apply Custom thresholding in order to detect AR Tags with no outline
    /*Mat threshold;
    const double threshold_val = 5; //If rgb is greater than threshold_val then set to white, otherwise set to black
    const double threshold_max = 255;
    const int threshold_type = 0; //Binary Thresholding (described above)
    cv::threshold(gray, threshold, threshold_val, threshold_max, threshold_type);*/
    /*------------------------------------THRESHOLDING CODE (Doesn't Work)-----------------------------------------------------*/
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

    Mat sharp;
    Mat kernel = (Mat_<double>(3, 3) << -1,-1,-1, -1, 9,-1, -1,-1,-1);
    filter2D(rgb, sharp, -1 , kernel);

    /*------------------------------------IMAGE SATURATION---------------------------------------------*/
    /*------------------------------------CONTOURING CODE-----------------------------------------------------*/
    //GrayScale Image
    Mat gray;
    cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    Mat canny_output;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> accepted;
    std::vector<Vec4i> hierarchy;
    int thresh = 15;
    Canny(gray, canny_output, thresh, thresh * 3, 3);
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    std::vector<std::vector<Point>>poly(contours.size());
    for (int i = 0; i < contours.size(); ++i) {
        cv::approxPolyDP(Mat(contours[i]), poly[i], 0.01*arcLength(Mat(contours[i]), true), true);
        //if (poly[i].size() < 20 && contourArea(contours[i]) > 75) {
            //accepted.push_back(contours[i]);
        //}

        if (poly[i].size() == 4){
            accepted.push_back(contours[i]);
        }
    }

    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    for (int i = 0; i < accepted.size(); i++) {
        Scalar color = Scalar(255, 255, 255);
        drawContours(drawing, accepted, i, color, 2, 8, hierarchy, 0, Point());
    }
    /*------------------------------------CONTOURING CODE-----------------------------------------------------*/
    /// Find tags
    Mat final = rgb + drawing;
    cv::aruco::detectMarkers(final, alvarDict, corners, ids, alvarParams);

    
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