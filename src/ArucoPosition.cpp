#include "rs_aruco_detection/ArucoPosition.h"

// do i need this libraries in here? or in the header?
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"

namespace catec_suministra_detection {

// Constructor
ArucoMarkerPosition::ArucoMarkerPosition(ros::NodeHandle& nh) : _nh(nh) // LAMBDA
// just handles everywhere i dont get it, why they are defined again?
{
    ROS_DEBUG("[ArucoMarkerPosition::ArucoMarkerPosition]"); // i just believe in this


    // Subscribe to the realsense image topic
    std::string image_topic; // why this is not in the header?
    _nh.param<std::string>("/suministra_rs/color/image_raw", image_topic, "");

    _rs_image_subscriber = _nh.subscribe<sensor_msgs::Image>(
        image_topic, ROS_TOPIC_BUFFER_SIZE, [&](const sensor_msgs::ImageConstPtr& msg) {
            ROS_DEBUG("[ArucoMarkerPosition::rs_subscriber]");

            // Subscriber code
            //callback detectArucosImageCallback called when a new image arrives
            detectArucosImages(msg);

            // Tip! Use private methods here to keep the code clean
            // this just doesn't look clean to me, it's so confusing, in my first version i create a callback, but doesn't it has to be as a void outside the constructor? so why now have the code here?
        }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()); // what is this?

}

// Constructor
// ArucoMarkerPosition::ArucoMarkerPosition(){ }

// Destructor
ArucoMarkerPosition::~ArucoMarkerPosition()
{ 
    ROS_DEBUG("[ArucoMarkerPosition::~ArucoMarkerPosition]");
}


void ArucoMarkerPosition::setPosition(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}


void ArucoMarkerPosition::getPosition() 
{
    std::cout << "ArucoMarkerPosition: " << this->x << ", " << this->y << ", " << this->z << std::endl;
}

void ArucoMarkerPosition::detectArucosImages(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Convert the ROS image message to an OpenCV image
        _current_image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // AruCo detection
        //_aruco_detector_parameters = cv::aruco::DetectorParameters();
        //_aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
        
        cv::aruco::DetectorParameters _aruco_detector_parameters = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary _aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        cv::aruco::detectMarkers(_current_image, &_aruco_dictionary, marker_corners, marker_ids, &_aruco_detector_parameters, rejected_candidates);
        
        cv::Mat output_image = _current_image.clone();
        cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);

        // Display the image
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Process the image using OpenCV
        cv::Mat image = cv_ptr->image;

        // Display the image
        //cv::imshow("RealSense Image", image);
        //cv::waitKey(1);

    }
    catch (cv_bridge::Exception& e) // executed if the try fails
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
        
}


//void ArucoMarkerPosition::publishArucoPositions() 
//{

//}



} // namespace catec_suministra_detection