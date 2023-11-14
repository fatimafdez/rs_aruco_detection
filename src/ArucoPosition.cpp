#include "rs_aruco_detection/ArucoPosition.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
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

} // namespace catec_suministra_detection