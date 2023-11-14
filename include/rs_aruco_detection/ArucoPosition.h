#pragma once
#include <ros/ros.h>
#include <string>  // Include necessary header for std::string

#define ROS_TOPIC_BUFFER_SIZE 1

namespace catec_suministra_detection
{
class ArucoMarkerPosition {
public:
    ArucoMarkerPosition(ros::NodeHandle& nh);  // Constructor
    virtual ~ArucoMarkerPosition();  // Destructor

    void setPosition(double x, double y, double z);
    std::string getPosition() const;

private:
    double x;
    double y;
    double z;
};

} // namespace catec_suministra_detection