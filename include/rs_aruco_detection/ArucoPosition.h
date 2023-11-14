#pragma once
#include <ros/ros.h>
#include <string>  // Include necessary header for std::string

#define ROS_TOPIC_BUFFER_SIZE 1

namespace catec_suministra_detection {
    class ArucoMarkerPosition {
    public:
        ArucoMarkerPosition();
        ~ArucoMarkerPosition();
        void setPosition(double x, double y, double z);
        void getPosition();
    private:
        double x, y, z;
    };

} // namespace catec_suministra_detection