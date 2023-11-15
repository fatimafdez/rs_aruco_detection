#pragma once
#include <ros/ros.h>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

#define ROS_TOPIC_BUFFER_SIZE 1

namespace catec_suministra_detection {
    class ArucoMarkerPosition {
    public:
        ArucoMarkerPosition();
        ~ArucoMarkerPosition();
        void setPosition(double x, double y, double z);
        void getPosition();
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);  

    private:
        double x, y, z;
        ros::Subscriber image_sub;
        cv::Mat current_image;

    };

} // namespace catec_suministra_detection