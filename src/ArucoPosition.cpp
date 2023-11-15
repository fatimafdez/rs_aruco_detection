#include "rs_aruco_detection/ArucoPosition.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <opencv2/opencv.hpp>
#include <sstream>

namespace catec_suministra_detection
{
    // Constructor
    ArucoMarkerPosition::ArucoMarkerPosition(){ }

    // Destructor
    ArucoMarkerPosition::~ArucoMarkerPosition(){ }

    // setPosition
    void ArucoMarkerPosition::setPosition(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    // getPosition
    void ArucoMarkerPosition::getPosition() 
    {
        std::cout << "ArucoMarkerPosition: " << this->x << ", " << this->y << ", " << this->z << std::endl;
    }

    void ArucoMarkerPosition::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // Process the image using OpenCV
            cv::Mat image = cv_ptr->image;

            // Display the image
            cv::imshow("RealSense Image", image);
            cv::waitKey(1);


            // Convert the ROS image message to an OpenCV image
            //current_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        
    }


} // namespace catec_suministra_detection