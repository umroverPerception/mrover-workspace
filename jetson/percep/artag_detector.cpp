#include "perception.hpp"
#include <assert.h>

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

//initializes detector object with pre-generated dictionary of tags 
TagDetector::TagDetector(const rapidjson::Document &mRoverConfig) :  

   //Populate Constants from Config File
   BUFFER_ITERATIONS{mRoverConfig["ar_tag"]["buffer_iterations"].GetInt()},
   MARKER_BORDER_BITS{mRoverConfig["alvar_params"]["marker_border_bits"].GetInt()},
   DO_CORNER_REFINEMENT{!!mRoverConfig["alvar_params"]["do_corner_refinement"].GetInt()},
   POLYGONAL_APPROX_ACCURACY_RATE{mRoverConfig["alvar_params"]["polygonal_approx_accuracy_rate"].GetDouble()},
   MM_PER_M{mRoverConfig["mm_per_m"].GetInt()},
   DEFAULT_TAG_VAL{mRoverConfig["ar_tag"]["default_tag_val"].GetInt()} {

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
    alvarParams->markerBorderBits = MARKER_BORDER_BITS;
    alvarParams->doCornerRefinement = DO_CORNER_REFINEMENT;
    alvarParams->polygonalApproxAccuracyRate = POLYGONAL_APPROX_ACCURACY_RATE;
}

Point2f TagDetector::getAverageTagCoordinateFromCorners(const vector<Point2f> &corners) {  //gets coordinate of center of tag
    // RETURN:
    // Point2f object containing the average location of the 4 corners
    // of the passed-in tag
    Point2f avgCoord;
    for (auto &corner : corners) {
        cout << "corner's x: " << corner.x << ", corner's y: " << corner.y << endl;
        avgCoord.x += corner.x;
        avgCoord.y += corner.y;
    }
    avgCoord.x /= corners.size();
    avgCoord.y /= corners.size();
    //cout << "average x: " << avgCoord.x << ", average y: " << avgCoord.y << endl;
    return avgCoord;
}

vector<Point2f> TagDetector::getMoreCoordinates(const vector<Point2f> &corners) {
    vector<Point2f> coordinates;
    Point2f avgCoord = getAverageTagCoordinateFromCorners(corners);
    int xMin = max(corners[0].x,corners[3].x);
    int xMax = min(corners[1].x,corners[2].x);
    int yMin = max(corners[0].y,corners[1].y);
    int yMax = min(corners[2].y,corners[3].y);
    Point2f point1;
    point1.x = avgCoord.x;
    point1.y = (avgCoord.y + yMin)/2;
    //cout << "point1 x: " << point1.x << ", point1 y: " << point1.y << endl;
    coordinates.push_back(point1);
    Point2f point2;
    point2.x = (avgCoord.x + xMax)/2;
    point2.y = avgCoord.y;
    //cout << "point2 x: " << point2.x << ", point2 y: " << point2.y << endl;
    coordinates.push_back(point2);
    Point2f point3;
    point3.x = avgCoord.x;
    point3.y = (avgCoord.y + yMax)/2;
    //cout << "point3 x: " << point3.x << ", point3 y: " << point3.y << endl;
    coordinates.push_back(point3);
    Point2f point4;
    point4.x = (avgCoord.x + xMin)/2;
    point4.y = avgCoord.y;
    //cout << "point4 x: " << point4.x << ", point4 y: " << point4.y << endl;
    coordinates.push_back(point4);
    Point2f point5;
    point5.x = (avgCoord.x + xMin)/2;
    point5.y = (avgCoord.y + yMin)/2;
    //cout << "point5 x: " << point5.x << ", point5 y: " << point5.y << endl;
    coordinates.push_back(point5);
    Point2f point6;
    point6.x = (avgCoord.x + xMax)/2;
    point6.y = (avgCoord.y + yMin)/2;
    //cout << "point6 x: " << point6.x << ", point6 y: " << point6.y << endl;
    coordinates.push_back(point6);
    Point2f point7;
    point7.x = (avgCoord.x + xMax)/2;
    point7.y = (avgCoord.y + yMax)/2;
    //cout << "point7 x: " << point7.x << ", point7 y: " << point7.y << endl;
    coordinates.push_back(point7);
    Point2f point8;
    point8.x = (avgCoord.x + xMin)/2;
    point8.y = (avgCoord.y + yMax)/2;
    //cout << "point8 x: " << point8.x << ", point8 y: " << point8.y << endl;
    coordinates.push_back(point8);
    return coordinates;
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

    // Find tags
    cv::aruco::detectMarkers(rgb, alvarDict, corners, ids, alvarParams);
#if AR_RECORD
cv::aruco::drawDetectedMarkers(rgb, corners, ids);
#endif
#if PERCEPTION_DEBUG
    // Draw detected tags
    cv::aruco::drawDetectedMarkers(rgb, corners, ids);
    if (corners.size() > 0)
    {
        vector<Point2f> morePoints = getMoreCoordinates(corners[0]);
        for (size_t i = 0; i < morePoints[0].y; i++)
        {
            rgb.at<Vec3b>(i,morePoints[0].x)[0] = 255;
            rgb.at<Vec3b>(i,morePoints[0].x)[1] = 255;
            rgb.at<Vec3b>(i,morePoints[0].x)[2] = 255;
        }
        for (size_t i = morePoints[2].y; i < 1000; i++)
        {
            rgb.at<Vec3b>(i,morePoints[2].x)[0] = 255;
            rgb.at<Vec3b>(i,morePoints[2].x)[1] = 255;
            rgb.at<Vec3b>(i,morePoints[2].x)[2] = 255;
        }
        for (size_t i = 0; i < morePoints[3].x; i++)
        {
            rgb.at<Vec3b>(morePoints[3].y,i)[0] = 255;
            rgb.at<Vec3b>(morePoints[3].y,i)[1] = 255;
            rgb.at<Vec3b>(morePoints[3].y,i)[2] = 255;
        }
        for (size_t i = morePoints[1].x; i < 1000; i++)
        {
            rgb.at<Vec3b>(morePoints[1].y,i)[0] = 255;
            rgb.at<Vec3b>(morePoints[1].y,i)[1] = 255;
            rgb.at<Vec3b>(morePoints[1].y,i)[2] = 255;
        }
        for (size_t i = 0; i < morePoints[4].y; i++)
        {
            rgb.at<Vec3b>(i,morePoints[4].x)[0] = 255;
            rgb.at<Vec3b>(i,morePoints[4].x)[1] = 0;
            rgb.at<Vec3b>(i,morePoints[4].x)[2] = 0;
        }
        for (size_t i = 0; i < morePoints[4].x; i++)
        {
            rgb.at<Vec3b>(morePoints[4].y,i)[0] = 255;
            rgb.at<Vec3b>(morePoints[4].y,i)[1] = 0;
            rgb.at<Vec3b>(morePoints[4].y,i)[2] = 0;
        }
        for (size_t i = morePoints[6].y; i < 1000; i++)
        {
            rgb.at<Vec3b>(i,morePoints[6].x)[0] = 255;
            rgb.at<Vec3b>(i,morePoints[6].x)[1] = 0;
            rgb.at<Vec3b>(i,morePoints[6].x)[2] = 0;
        }
        for (size_t i = morePoints[6].x; i < 1000; i++)
        {
            rgb.at<Vec3b>(morePoints[6].y,i)[0] = 255;
            rgb.at<Vec3b>(morePoints[6].y,i)[1] = 0;
            rgb.at<Vec3b>(morePoints[6].y,i)[2] = 0;
        }
        for (size_t i = 0; i < morePoints[5].y; i++)
        {
            rgb.at<Vec3b>(i,morePoints[5].x)[0] = 255;
            rgb.at<Vec3b>(i,morePoints[5].x)[1] = 0;
            rgb.at<Vec3b>(i,morePoints[5].x)[2] = 0;
        }
        for (size_t i = morePoints[5].x; i < 1000; i++)
        {
            rgb.at<Vec3b>(morePoints[5].y,i)[0] = 255;
            rgb.at<Vec3b>(morePoints[5].y,i)[1] = 0;
            rgb.at<Vec3b>(morePoints[5].y,i)[2] = 0;
        }
        for (size_t i = morePoints[7].y; i < 1000; i++)
        {
            rgb.at<Vec3b>(i,morePoints[7].x)[0] = 255;
            rgb.at<Vec3b>(i,morePoints[7].x)[1] = 0;
            rgb.at<Vec3b>(i,morePoints[7].x)[2] = 0;
        }
        for (size_t i = 0; i < morePoints[7].x; i++)
        {
            rgb.at<Vec3b>(morePoints[7].y,i)[0] = 255;
            rgb.at<Vec3b>(morePoints[7].y,i)[1] = 0;
            rgb.at<Vec3b>(morePoints[7].y,i)[2] = 0;
        }
        
        cout << morePoints << endl;
        //cout << corners[0][0].x << endl;
        // for (size_t i = 0; i < corners[0][0].y; i++)
        // {
        //     rgb.at<Vec3b>(i,corners[0][0].x)[0] = 255;
        //     rgb.at<Vec3b>(i,corners[0][0].x)[1] = 255;
        //     rgb.at<Vec3b>(i,corners[0][0].x)[2] = 255;
        // }
        // for (size_t i = 0; i < corners[0][0].x; i++)
        // {
        //     rgb.at<Vec3b>(corners[0][0].y,i)[0] = 255;
        //     rgb.at<Vec3b>(corners[0][0].y,i)[1] = 255;
        //     rgb.at<Vec3b>(corners[0][0].y,i)[2] = 255;
        // }
        // for (size_t i = 0; i < corners[0][1].y; i++)
        // {
        //     rgb.at<Vec3b>(i,corners[0][1].x)[0] = 255;
        //     rgb.at<Vec3b>(i,corners[0][1].x)[1] = 0;
        //     rgb.at<Vec3b>(i,corners[0][1].x)[2] = 0;
        // }
        // for (size_t i = corners[0][1].x; i < 1000; i++)
        // {
        //     rgb.at<Vec3b>(corners[0][1].y,i)[0] = 255;
        //     rgb.at<Vec3b>(corners[0][1].y,i)[1] = 0;
        //     rgb.at<Vec3b>(corners[0][1].y,i)[2] = 0;
        // }
        // for (size_t i = corners[0][2].y; i < 1000; i++)
        // {
        //     rgb.at<Vec3b>(i,corners[0][2].x)[0] = 0;
        //     rgb.at<Vec3b>(i,corners[0][2].x)[1] = 255;
        //     rgb.at<Vec3b>(i,corners[0][2].x)[2] = 0;
        // }
        // for (size_t i = corners[0][2].x; i < 1000; i++)
        // {
        //     rgb.at<Vec3b>(corners[0][2].y,i)[0] = 0;
        //     rgb.at<Vec3b>(corners[0][2].y,i)[1] = 255;
        //     rgb.at<Vec3b>(corners[0][2].y,i)[2] = 0;
        // }
        // for (size_t i = corners[0][2].y; i < 1000; i++)
        // {
        //     rgb.at<Vec3b>(i,corners[0][3].x)[0] = 0;
        //     rgb.at<Vec3b>(i,corners[0][3].x)[1] = 0;
        //     rgb.at<Vec3b>(i,corners[0][3].x)[2] = 255;
        // }
        // for (size_t i = 0; i < corners[0][2].x; i++)
        // {
        //     rgb.at<Vec3b>(corners[0][3].y,i)[0] = 0;
        //     rgb.at<Vec3b>(corners[0][3].y,i)[1] = 0;
        //     rgb.at<Vec3b>(corners[0][3].y,i)[2] = 255;
        // }
    }
    
    
   
    cv::imshow("AR Tags", rgb);

    // on click debugging for color
    DEPTH = depth_src;
    cvtColor(rgb, HSV, COLOR_RGB2HSV);
    setMouseCallback("Obstacle", onMouse);
#endif

    // create Tag objects for the detected tags and return them
    pair<Tag, Tag> discoveredTags;
    if (ids.size() == 0) {
        // no tags found, return invalid objects with tag set to -1
        discoveredTags.first.id = DEFAULT_TAG_VAL;
        discoveredTags.first.loc = Point2f();
        discoveredTags.second.id = DEFAULT_TAG_VAL;
        discoveredTags.second.loc = Point2f();

    } else if (ids.size() == 1) {  // exactly one tag found
        discoveredTags.first.id = ids[0];
        discoveredTags.first.loc = getAverageTagCoordinateFromCorners(corners[0]);
        discoveredTags.first.extraPoints = getMoreCoordinates(corners[0]);
        // set second tag to invalid object with tag as -1
        discoveredTags.second.id = DEFAULT_TAG_VAL;
        discoveredTags.second.loc = Point2f();
    } else if (ids.size() == 2) {  // exactly two tags found
        Tag t0, t1;
        t0.id = ids[0];
        t0.loc = getAverageTagCoordinateFromCorners(corners[0]);
        t0.extraPoints = getMoreCoordinates(corners[0]);
        t1.id = ids[1];
        t1.loc = getAverageTagCoordinateFromCorners(corners[1]);
        t1.extraPoints = getMoreCoordinates(corners[1]);
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
        t0.extraPoints = getMoreCoordinates(corners[0]);
        t1.id = ids[ids.size() - 1];
        t1.loc = getAverageTagCoordinateFromCorners(corners[ids.size() - 1]);
        t1.extraPoints = getMoreCoordinates(corners[ids.size() - 1]);
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

double TagDetector::getAngle(float xPixel, float wPixel){
    double fieldofView = 110 * PI/180;
    return atan((xPixel - wPixel/2)/(wPixel/2)* tan(fieldofView/2))* 180.0 /PI;
}

void TagDetector::updateDetectedTagInfo(rover_msgs::Target *arTags, pair<Tag, Tag> &tagPair, Mat &depth_img, Mat &src){
    struct tagPairs {
        vector<int> id;
        vector<double> locx;
        vector<double> locy;
        vector<int> buffer;
    }; 
    tagPairs tags;

    tags.id.push_back(tagPair.first.id);
    tags.locx.push_back(tagPair.first.loc.x);
    tags.locy.push_back(tagPair.first.loc.y);
    for (size_t i = 0; i < tagPair.first.extraPoints.size(); i++)
    {
        tags.locx.push_back(tagPair.first.extraPoints[i].x);
        tags.locy.push_back(tagPair.first.extraPoints[i].y);
    }
    tags.id.push_back(tagPair.second.id);
    tags.locx.push_back(tagPair.second.loc.x);
    tags.locy.push_back(tagPair.second.loc.y);
    for (size_t i = 0; i < tagPair.second.extraPoints.size(); i++)
    {
        tags.locx.push_back(tagPair.second.extraPoints[i].x);
        tags.locy.push_back(tagPair.second.extraPoints[i].y);
    }
    tags.buffer.push_back(0);
    tags.buffer.push_back(0);

  for (uint i=0; i<18; i += 9) {
    if(tags.id[i/9] == DEFAULT_TAG_VAL){//no tag found
        if(tags.buffer[i/9] <= BUFFER_ITERATIONS){//send buffered tag until tag is found
            ++tags.buffer[i/2];
        } else {//if still no tag found, set all stats to -1
            arTags[i/9].distance = DEFAULT_TAG_VAL;
            arTags[i/9].bearing = DEFAULT_TAG_VAL;
            arTags[i/9].id = DEFAULT_TAG_VAL;
        }
     } else {//tag found
     vector<double> tagDistances;
     double totalDistances;
     double meanDistance;
        for(int j = 0; j < 9; ++j) {
        if(!isnan(depth_img.at<float>(tags.locy.at(i+j), tags.locx.at(i+j)))) {
            tagDistances.push_back(depth_img.at<float>(tags.locy.at(i+j), tags.locx.at(i+j)) / MM_PER_M);
            totalDistances += tagDistances[j];
            cout << "Tag " << j+1 << " distance: " << tagDistances[j] << endl;
        }
        if (tagDistances.size() == 0)
        {
            meanDistance = DEFAULT_TAG_VAL;
        }
        else {
            meanDistance = totalDistances/tagDistances.size();
        }
        
        }
        arTags[i/9].distance = meanDistance;
        arTags[i/9].bearing = getAngle((int)tags.locx.at(i/9), src.cols);
        arTags[i/9].id = tags.id.at(i/9);
        tags.buffer[i/9] = 0;
   }
  }
}
