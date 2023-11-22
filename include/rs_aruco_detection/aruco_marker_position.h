#pragma once
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <std_msgs/UInt8.h>

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

#define ROS_TOPIC_BUFFER_SIZE 1

namespace catec_suministra {

class ArucoMarkerPosition
{
  public:
    ArucoMarkerPosition(ros::NodeHandle& nh); // Constructor with node handle, does that means i dont need to create a
                                              // node handle in the main? no, but why?
    virtual ~ArucoMarkerPosition();

    // Public methods
    void setPosition(double x, double y, double z);
    void getPosition();

  private:
    // Private methods
    void detectArucosImages(const sensor_msgs::ImageConstPtr& image_msg);
    std::pair<cv::Mat, cv::Mat> getCalibrationParameters(const std::string& filename);
    // void readCalibrationParameters(const std::string& filename);
    // void publishArucoPositions(const sensor_msgs::ImageConstPtr& msg);

  private:
    // Private attributes
    double                                 x, y, z;
    ros::NodeHandle&                       _nh;
    ros::Subscriber                        _rs_image_subscriber;
    cv::Mat                                _current_image;
    cv::Mat                                _obj_points;
    cv::Ptr<cv::aruco::DetectorParameters> _aruco_detector_parameters;
    cv::Ptr<cv::aruco::Dictionary>         _aruco_dictionary;

    // cv::aruco::detectMarkers _detect_aruco_markers;
    // cv::aruco::drawDetectedMarkers _draw_detected_markers;
};

} // namespace catec_suministra