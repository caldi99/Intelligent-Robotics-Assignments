#include <detection/wall_detector/wall_detector.h>

WallDetector::WallDetector(float angleIncrement, float angleMin, float angleMax, float rangeMax, float rangeMin){
    this->angleIncrement = angleIncrement;
    this->angleMin = angleMin + M_PI_2;
    this->angleMax = angleMax + M_PI_2;
    this->rangeMax = rangeMax;
    this->rangeMin = rangeMin;
    image = cv::Mat(getNumberRows(), getNumberCols(), CV_8UC1, cv::Scalar(0,0,0));
}//WallDetector

int WallDetector::getNumberRows(){
    return std::floor((angleMin + M_PI_2)/(angleIncrement));
}//getNumberRows

int WallDetector::getNumberCols(){
    return std::floor((M_PI)/(angleIncrement));
}//getNumberCols

float WallDetector::getWidthPixel(){
    return (2*rangeMax)/(getNumberCols());
}//getWidthPixel

float WallDetector::getHeightPixel(){
    return ((rangeMax)+(std::abs(rangeMax * std::sin(angleMin))))/(getNumberRows());
}//getHeightPixel

float WallDetector::getXRawImage(float rho, float theta){
    return rangeMax - rho * std::sin(theta);
}//getXRawImage

float WallDetector::getYRawImage(float rho, float theta){
    return rangeMax + rho * std::cos(theta);
}//getYRawImage

int WallDetector::getXImage(float rho, float theta){    
    return std::floor((getXRawImage(rho,theta)/(getHeightPixel())));
}//getXImage

int WallDetector::getYImage(float rho, float theta){
    return std::floor((getYRawImage(rho,theta)/(getWidthPixel())));
}//getYImage

void WallDetector::buildImage(const std::vector<float>& ranges){
    for(int i=0; i<ranges.size(); i++)
    {
        float rho = ranges.at(i);
        float theta = angleMin + (angleIncrement * (i));
        int x = getXImage(rho, theta);
        int y = getYImage(rho, theta);
        image.at<unsigned char>(x, y) = 255;     
    }
}//buildImage

bool WallDetector::findLines(const std::vector<float>& ranges, std::vector<cv::Vec4f>& finalLines){
    //Build Image
    buildImage(ranges);

    //Dilation application
    cv::dilate(image,image,cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,SIZE_MORPHOLOGICAL_OPERATOR_DILATION));

    //Erosion application
    cv::erode(image,image,cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,SIZE_MORPHOLOGICAL_OPERATOR_EROSION));

    //Apply Hough Lines
    std::vector<cv::Vec4i> linesHough;
    cv::HoughLinesP(image,linesHough, RHO_HOUGH_TRANSFORM, THETA_HOUGH_TRANSFORM, THRESHOLD_HOUGH_LINES, MIN_LINE_LENGHT_HOUGH_LINES);    

    //Split lines in 2
    std::vector<int> labels;
    bool fail = true;
    if(cv::partition(linesHough,labels, WallDetector::areLinesToMerge) == 2)
    {
        fail = false;
        std::vector<cv::Vec4i> finalLinesInteger = refineLines(mergeLines(linesHough,labels,1),mergeLines(linesHough,labels,0));

        //Convert To **TRUE** Robot Coordinates 
        for(cv::Vec4i line : finalLinesInteger)
        {
            cv::Point2f point1 = convertToRobotCoordinates(line[1],line[0]); //row, col
            cv::Point2f point2 = convertToRobotCoordinates(line[3],line[2]);
            finalLines.push_back(cv::Vec4f(point1.x,point1.y,point2.x,point2.y));
        }
    }
    return fail;  
}//findLines

cv::Point2f WallDetector::convertToRobotCoordinates(int x, int y)
{
    return cv::Point2f(rangeMax - (x * getHeightPixel()), rangeMax - (x * getWidthPixel()));
}

cv::Vec4i WallDetector::mergeLines(const std::vector<cv::Vec4i>& linesHough, const std::vector<int>& labels, int label)
{
    std::vector<cv::Vec4i> linesToMerge;

    //Select Lines to compute Average
    for(int i = 0; i < labels.size(); i++)    
        if(labels.at(i) == label)        
            linesToMerge.push_back(linesHough.at(i));
        
    //Compute the Average for the lines
    int yMin = INT_MAX;
    int yMax = -1;
    int xAvg = 0;
    for(cv::Vec4i line : linesToMerge)
    {
        yMin = std::min(yMin,line[3]);
        yMax = std::max(yMax,line[1]);
        xAvg += ((line[0] + line[2]) / 2);
    }
    xAvg /= linesToMerge.size();

    return cv::Vec4i(xAvg,yMax,xAvg,yMin);    
}

std::vector<cv::Vec4i> WallDetector::refineLines(const cv::Vec4i& line1, const cv::Vec4i& line2)
{
    std::vector<cv::Vec4i> ret;
    //Compute yMax and yMin
    int yMax = std::max(line1[1],line2[1]);
    int yMin = std::max(line1[3],line2[3]);

    //Return the vector with the 2 lines
    ret.push_back(cv::Vec4i (line1[0],yMax,line1[2],yMin));
    ret.push_back(cv::Vec4i (line2[0],yMax,line2[2],yMin));    
    return ret;
}

cv::Mat WallDetector::drawLines(const std::vector<cv::Vec4i>& lines)
{
    cv::Mat ret(image.rows,image.cols,CV_8UC1);

    //Draw Lines
    for(cv::Vec4i line : lines)
    {
        cv::line(ret,cv::Point(line[0], line[1]), //Start
                     cv::Point(line[2], line[3]), //End
                 cv::Scalar(255,255,255), //Color Line
                 1, //thickness
                 cv::LINE_AA //Line Type
        );
    }
    return ret;
}

//STATIC 
bool WallDetector::areLinesToMerge(const cv::Vec4i& line1, const cv::Vec4i& line2)
{
    if(std::abs(line1[0]-line2[0]) <= MAX_DISTANCE_EQUAL_POINTS && 
        std::abs(line1[2]-line2[2]) <= MAX_DISTANCE_EQUAL_POINTS) //Same column    
        if((std::abs(line1[1] - line2[1]) <= MAX_DISTANCE_EQUAL_POINTS) || 
           (std::abs(line1[3] - line2[3]) <= MAX_DISTANCE_EQUAL_POINTS) || 
           (std::abs(line1[1] - line2[3]) <= MAX_DISTANCE_EQUAL_POINTS_HEIGHT) || 
           (std::abs(line2[1] - line1[3]) <= MAX_DISTANCE_EQUAL_POINTS_HEIGHT))        
                return true;                    
    return false;    
}