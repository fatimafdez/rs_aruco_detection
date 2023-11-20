#pragma once
#include <ros/ros.h>
#include <ros/transport_hints.h>

#include <std_msgs/UInt8.h>

#include <iostream>
#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>


#define ROS_TOPIC_BUFFER_SIZE 1

namespace catec_suministra_detection {

class ArucoMarkerPosition 
{
    public:
        ArucoMarkerPosition(ros::NodeHandle& nh); // Constructor with node handle, does that means i dont need to create a node handle in the main? no, but why?
        ~ArucoMarkerPosition(); // why this is virtual?

        // Public methods
        void setPosition(double x, double y, double z);
        void getPosition();
        
    private: // why 2 private parts?
        // Private methods
        void detectArucosImages(const sensor_msgs::ImageConstPtr& msg); 
        //void publishArucoPositions(const sensor_msgs::ImageConstPtr& msg);
        
    private:
        // Private attributes
        double x, y, z;
        ros::NodeHandle& _nh; // why another handle?
        ros::Subscriber _rs_image_subscriber;
        cv::Mat _current_image;
        //cv::aruco::DetectorParameters _aruco_detector_parameters;
        //cv::aruco::Dictionary _aruco_dictionary;
        //cv::aruco::detectMarkers _detect_aruco_markers;
        //cv::aruco::drawDetectedMarkers _draw_detected_markers;

};

} // namespace catec_suministra_detection