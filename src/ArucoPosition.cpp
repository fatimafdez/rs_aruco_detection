#include "rs_aruco_detection/ArucoPosition.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
// from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
// OpenCV library
#include <opencv2/opencv.hpp>
#include <sstream>

namespace catec_suministra_detection
{
// Constructor
ArucoMarkerPosition::ArucoMarkerPosition(ros::NodeHandle& nh) : _nh(nh){}

// Destructor
ArucoMarkerPosition::~ArucoMarkerPosition() { }

// Member function definition for setPosition
void ArucoMarkerPosition::setPosition(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

// Member function definition for getPosition
void ArucoMarkerPosition::getPosition() const {
    // Implement the function to return the position as a string
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
}
} // namespace catec_suministra_detection