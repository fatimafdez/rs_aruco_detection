#include "rs_aruco_detection/ArucoPosition.h"

#include "std_msgs/String.h"                                            // do i need this libraries in here? or in the header?
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"

namespace catec_suministra_detection {

ArucoMarkerPosition::ArucoMarkerPosition(ros::NodeHandle& nh) : _nh(nh) // Constructor 
                                                                        // LAMBDA - is defined with the sintax [capture list] (function arguments) {body}
                                                                        // Apparently this code store inside the subscriber object what it needs to do, it's more clean
                                                                        // & indicates that we are passing the object by reference, so we are not copying it, we are just passing the pointer to the object -> indicates to capture everything, so we have access to all the variables and methods of the class, as if we write everything outside the lambda function
                                                                        // just handles everywhere i dont get it, why they are defined again?
{
    ROS_DEBUG("[ArucoMarkerPosition::ArucoMarkerPosition]");            // i just believe in this


    // Subscribe to the realsense image topic
    std::string image_topic;                                            // why this is not in the header?
    _nh.param<std::string>("/suministra_rs/color/image_raw", image_topic, "");

    _rs_image_subscriber = _nh.subscribe<sensor_msgs::Image>(
        image_topic, ROS_TOPIC_BUFFER_SIZE, [&](const sensor_msgs::ImageConstPtr& image_msg) {
            ROS_DEBUG("[ArucoMarkerPosition::rs_subscriber]");

            // Subscriber code
            
            detectArucosImages(image_msg);                              //callback detectArucosImageCallback called when a new image arrives
            cv::imshow("Image Window", _current_image);
            cv::waitKey(1);
                                                                        // Tip! Use private methods here to keep the code clean
            
        }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());    // the subscriptor and publishers follow a TCP/IP protocol for communication,   
                                                                        // interlally it opens a socket and sends the data through it (by datagrams), this protocol uses less bandwidth, making a compression of the data, wich adds a delay, so to reduce this delays (which is quite important for odometry, lidar, imu ...) we use this line of code, this uses more bandwidth but reduces the delay, so it's better for this kind of data

}


ArucoMarkerPosition::~ArucoMarkerPosition()                             // Destructor
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

void ArucoMarkerPosition::detectArucosImages(const sensor_msgs::ImageConstPtr& image_msg)
{
    try
    {
        
        _current_image = cv_bridge::toCvShare(image_msg, "bgr8")->image;            // Convert the ROS image message to an OpenCV image

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
        
        cv::aruco::DetectorParameters aruco_detector_parameters = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        cv::aruco::detectMarkers(_current_image, &aruco_dictionary, marker_corners, marker_ids, &aruco_detector_parameters, rejected_candidates);
        
        cv::Mat output_image = _current_image.clone();                              // Copy the image to draw the markers
        cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);   // Check if the detected markers are correct

        if (marker_ids.size() > 0)
            cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);

        cv::imshow("out", output_image);
        char key = (char) cv::waitKey(10);

        if (key == 27) return;

        //cv_bridge::CvImageConstPtr cv_ptr;
        //cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

        
        //cv::Mat image = cv_ptr->image;                                  // Process the image using OpenCV

        
        //cv::imshow("RealSense Image", image); // Display the image
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