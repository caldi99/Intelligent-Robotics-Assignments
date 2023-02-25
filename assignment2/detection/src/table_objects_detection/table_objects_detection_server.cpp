#include <detection/table_objects_detection/table_objects_detection_server.h>

TableObjectsDetectionServer::TableObjectsDetectionServer() : actionServer(nh, detection::TOPIC_TABLE_OBJECTS_DETECTION, boost::bind(&TableObjectsDetectionServer::startDetection, this, _1), false)
{
    actionServer.start();
    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer TableObjectsDetectionServer");
}

void TableObjectsDetectionServer::startDetection(const detection_msgs::ObjectsDetectionGoalConstPtr &goal)
{
    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer startDetection");
    this->colorID = goal->colorID;
    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer searching for the table");
    this->feedback.state = 0;
    actionServer.publishFeedback(feedback);
    moveHeadDown();
    pointTable();
    pointTable();

    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer saving joint position of the table" << joint1 << joint2);

    // save joints position of the table for later
    ros::NodeHandle node;
    sensor_msgs::JointState::ConstPtr jointStateMessagePointer = ros::topic::waitForMessage<sensor_msgs::JointState>(detection::TOPIC_JOINT_STATUS, node);
    joint1 = jointStateMessagePointer->position[9];
    joint2 = jointStateMessagePointer->position[10];

    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer searching for the colored object");
    this->feedback.state = 1;
    actionServer.publishFeedback(feedback);
    pointColor();

    // reset head to table position
    resetPosition();

    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer searching for the other objects in the table");
    this->feedback.state = 2;
    actionServer.publishFeedback(feedback);
    pointObstacleTags();

    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer detection finished");

    //reset of the server
    observedIds.clear();
    joint1 = 0.0;
    joint2 = 0.0;

    // set the action state to succeeded
    actionServer.setSucceeded(result);

    result.detections.poses.clear();
    
}

void TableObjectsDetectionServer::moveHeadDown()
{
    // move the head down a little by using the manipulation package
    ros::ServiceClient client = nh.serviceClient<manipulation_msgs::HeadMovement>(detection::TOPIC_HEAD_MOVEMENT);
    manipulation_msgs::HeadMovement srv;
    srv.request.mode = 1;
    if (client.call(srv))
    {
        ROS_INFO_STREAM("(Server) detection pointTable Moving the head down");
    }
    else
    {
        ROS_ERROR("(Server) detection pointTable Failed to call the head movement server for moving down the head");
    }
}

void TableObjectsDetectionServer::pointTable()
{

    // get the TIAGo camera image
    sensor_msgs::ImageConstPtr imgMsg = ros::topic::waitForMessage<sensor_msgs::Image>(detection::TOPIC_IMAGE, nh);
    cv_bridge::CvImagePtr cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);

    cv::Mat imgHSV;

    // Convert from BGR to HSV colorspace
    cv::cvtColor(cvImgPtr->image, imgHSV, cv::COLOR_BGR2HSV);

    cv::Mat thresholdedImg;

    // search for the color treshold
    cv::inRange(imgHSV, cv::Scalar(20, 63, 0), cv::Scalar(23, 255, 81), thresholdedImg);

    // cv::imshow("Thresh", thresholdedImg);
    // cv::waitKey();

    cv::Rect boundingRect = cv::boundingRect(thresholdedImg);

    // cv::rectangle(thresholdedImg, boundingRect, cv::Scalar(255,255,255));
    // cv::imshow("Rect", thresholdedImg);
    // cv::waitKey();

    // point to the table
    pointArea(boundingRect);
}

void TableObjectsDetectionServer::pointColor()
{
    // get the TIAGo camera image
    sensor_msgs::ImageConstPtr imgMsg = ros::topic::waitForMessage<sensor_msgs::Image>(detection::TOPIC_IMAGE, nh);
    cv_bridge::CvImagePtr cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);

    cv::Mat imgHSV;
    // Convert from BGR to HSV colorspace
    cv::cvtColor(cvImgPtr->image, imgHSV, cv::COLOR_BGR2HSV);

    cv::Mat thresholdedImg;

    // search for the color treshold
    if (colorID == 1)
    { // BLUE
        cv::inRange(imgHSV, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), thresholdedImg);
    }
    else if (colorID == 2)
    { // GREEN
        cv::inRange(imgHSV, cv::Scalar(36, 25, 25), cv::Scalar(70, 255, 255), thresholdedImg);
    }
    else if (colorID == 3)
    { // RED
        cv::inRange(imgHSV, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), thresholdedImg);
    }

    cv::Rect boundingRect = cv::boundingRect(thresholdedImg);

    // point to the color
    pointArea(boundingRect);

    //record the ids that tiago see
    ros::NodeHandle node;
    detection_msgs::Detections::ConstPtr detectionsMessagePointer = ros::topic::waitForMessage<detection_msgs::Detections>(detection::TOPIC_POSES_DETECTION, node);
    for (int i = 0; i < detectionsMessagePointer->poses.size(); i++)
    {
        observedIds.insert(detectionsMessagePointer->poses.at(i).id);
        this->result.detections.poses.push_back(detectionsMessagePointer->poses.at(i));
    }
}

void TableObjectsDetectionServer::pointObstacleTags()
{

    // get the TIAGo camera image
    sensor_msgs::ImageConstPtr imgMsg = ros::topic::waitForMessage<sensor_msgs::Image>(detection::TOPIC_IMAGE, nh);
    cv_bridge::CvImagePtr cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);

    // find the black color of the AprilTags
    cv::Mat imgHSV;
    cv::cvtColor(cvImgPtr->image, imgHSV, cv::COLOR_BGR2HSV);
    cv::Mat thresholdedImgBlack;
    cv::inRange(imgHSV, cv::Scalar(0, 0, 0), cv::Scalar(50, 50, 50), thresholdedImgBlack);
    // cv::imshow( "Black", thresholdedImgBlack );
    // cv::waitKey();

    // extract the rectangles of the AprilTags by using tag black colors
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholdedImgBlack, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<cv::Point2f> centers(contours.size());
    std::vector<float> radius(contours.size());

    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = cv::boundingRect(contours_poly[i]);
        minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
    }

    /*cv::RNG rng(12345);
    cv::Mat drawing = cv::Mat::zeros((cvImgPtr->image).size(), CV_8UC3 );
    for( int i = 0; i< boundRect.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
    }
    cv::imshow( "Contours", drawing );
    cv::waitKey();*/

    cv::groupRectangles(boundRect, 1, 0.85);

    /*
    drawing = cv::Mat::zeros((cvImgPtr->image).size(), CV_8UC3 );
    for( int i = 0; i< boundRect.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
    }
    cv::imshow( "Filter", drawing );
    cv::waitKey();
    */

    // boundRect contains all the rectangles of the april tags detected, we need to go through them
    // but first we need to remove the colored object tag because we have already seen it

    cv::Mat imgHSVObject;
    cv::cvtColor(cvImgPtr->image, imgHSVObject, cv::COLOR_BGR2HSV);

    cv::Mat thresholdedImgObject;

    if (colorID == 1)
    { // BLUE
        cv::inRange(imgHSV, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), thresholdedImgObject);
    }
    else if (colorID == 2)
    { // GREEN
        cv::inRange(imgHSV, cv::Scalar(36, 25, 25), cv::Scalar(70, 255, 255), thresholdedImgObject);
    }
    else if (colorID == 3)
    { // RED
        cv::inRange(imgHSV, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), thresholdedImgObject);
    }

    // cv::imshow( "Obj", thresholdedImgObject );
    // cv::waitKey();

    //remove the colored object tag from the list of tags that must be visited
    cv::Rect boundingRectObject = cv::boundingRect(thresholdedImgObject);
    std::vector<cv::Rect> finalRects;
    for (int i = 0; i < boundRect.size(); i++)
    {
        cv::Rect intersection = boundRect[i] & boundingRectObject;
        if (intersection.area() == 0)
            finalRects.push_back(boundRect[i]);
    }

    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer collision objects rectangles to visit" << finalRects.size());

    /*
    cv::RNG rng(12345);
    cv::Mat drawing = cv::Mat::zeros((cvImgPtr->image).size(), CV_8UC3 );
    for( int i = 0; i< finalRects.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        cv::rectangle( drawing, finalRects[i].tl(), finalRects[i].br(), color, 2 );
    }
    //cv::rectangle( drawing, boundingRectObject.tl(), boundingRectObject.br(), cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) ), 2 );
    cv::imshow( "Filter", drawing );
    cv::waitKey();
    */
    ros::NodeHandle node;
 
    // starting the exploration and record the ids that tiago see
    for (int i = 0; i < finalRects.size(); i++)
    {
        ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer collision objects rectangles to visit" << i);
        pointArea(finalRects[i]);

        detection_msgs::Detections::ConstPtr detectionsMessagePointer = ros::topic::waitForMessage<detection_msgs::Detections>(detection::TOPIC_POSES_DETECTION, node);
        for (int i = 0; i < detectionsMessagePointer->poses.size(); i++)
        {
            auto it = observedIds.find(detectionsMessagePointer->poses.at(i).id);
            if (it == observedIds.end())
            {
                this->result.detections.poses.push_back(detectionsMessagePointer->poses.at(i));
                observedIds.insert(detectionsMessagePointer->poses.at(i).id);
            }
        }

        // reset head

        resetPosition();
    }

    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer number of detected april tags:" << this->result.detections.poses.size());
}

void TableObjectsDetectionServer::resetPosition()
{
    manipulation_msgs::HeadMovement srv;
    srv.request.mode = 2;

    // send to the manipulation head movement node the joints values of the table position
    srv.request.rotation_vertical = joint2;
    srv.request.rotation_horizontal = joint1;
    if (clientHead.call(srv))
    {
        ROS_INFO_STREAM("Reset Head");
    }
    else
    {
        ROS_ERROR("Failed to call service for head movement");
    }
}

void TableObjectsDetectionServer::pointArea(cv::Rect colorizedArea)
{
    manipulation_msgs::HeadMovement srv;
    srv.request.mode = 3;

    // send to the manipulation head movement node the image point where to look 
    geometry_msgs::Point center;
    center.x = colorizedArea.x + colorizedArea.width / 2;
    center.y = colorizedArea.y + colorizedArea.height / 2;
    srv.request.point = center;
    if (clientHead.call(srv))
    {
        ROS_INFO_STREAM("Head moved");
    }
    else
    {
        ROS_ERROR("Failed to call service for head movement");
    }
}

// --------------------------------------------------------------------- NOT USED BUT USEFUL ---------------------------------------------------------*/

/* DEBUG METHOD FOR TESTING WITHOUT ACTIONS
void TableObjectsDetectionServer::start(){
    moveHeadDown();
    pointTable();
    pointTable();

    //save position of the table
    ros::NodeHandle node;
    sensor_msgs::JointState::ConstPtr jointStateMessagePointer = ros::topic::waitForMessage<sensor_msgs::JointState>(detection::TOPIC_JOINT_STATUS, node);
    joint1 = jointStateMessagePointer->position[9];
    joint2 = jointStateMessagePointer->position[10];

    ROS_INFO_STREAM_STREAM(joint1 << joint2);

    //tavolo trovato
    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer tavolo trovato");

    pointColor();
    ROS_INFO_STREAM("(Server) detection TableObjectsDetectionServer colore trovato");

    //reset posizione, da sostituire con valori joint
    resetPosition();

    pointObstacleTags();

}

*/

/*
void TableDetectionServer::lookToPoint(cv::Point2d center){
    geometry_msgs::Point point;
    point.x = center.x;
    point.y = center.y;

    //create the request
    manipulation::HeadMovement srv;
    srv.request.mode = 3;
    srv.request.point = point;

    if (clientHead.call(srv))
    {
        ROS_INFO_STREAM("Head moved");
    }
    else
    {
        ROS_ERROR("Failed to call service for head movement");
    }
}

bool TableDetectionServer::checkObject(cv::Mat img){
    //search for the color treshold
    cv::Mat imgHSV;

    // Convert from BGR to HSV colorspace
    cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);

    cv::Mat thresholdedImg;

    // Detect the object based on HSV Range Values and based on colored object to search
    if(colorID == 1){ //BLUE
        cv::inRange(imgHSV, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), thresholdedImg);
    } else if(colorID == 2){ //GREEN
        cv::inRange(imgHSV, cv::Scalar(36, 25, 25), cv::Scalar(70, 255, 255), thresholdedImg);
    } else if(colorID == 3) {  //RED
        cv::inRange(imgHSV, cv::Scalar(0, 50,50), cv::Scalar(10, 255, 255), thresholdedImg);
    }

    cv::Rect boundingRect = cv::boundingRect(thresholdedImg);

    //check for the integrality of the object in the image
    cv::Rect rectWithBorder (boundingRect.x - 5, boundingRect.y + 5, boundingRect.width + 10, boundingRect.height + 10);

    if((boundingRect & rectWithBorder) == boundingRect)
        false;
    return true;
}
*/
