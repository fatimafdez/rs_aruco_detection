#pragma once

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <std_msgs/UInt8.h>
#include <suministra_msgs/MarkerDetection.h>
#include <suministra_msgs/MarkerDetectionArray.h>

#include <iostream>
#include <numeric>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

#define ROS_TOPIC_BUFFER_SIZE 100

namespace catec_suministra {

typedef struct
{
    std::vector<cv::Vec4d>   rvecs;
    std::vector<cv::Vec3d>   tvecs;
    std::vector<int>         ids;
    std::vector<std::string> frames;
} ArucoDetections;

class ArucoMarkerPosition
{
  public:
    ArucoMarkerPosition(ros::NodeHandle& nh); // Constructor with node handle, does that means i dont need to create a
                                              // node handle in the main? no, but why?
    virtual ~ArucoMarkerPosition();

  private:
    // Private methods
    ArucoDetections detectArucosPoses(const sensor_msgs::ImageConstPtr&);
    void            readCalibrationParameters(const std::string&);
    void            readMarkersLength(const std::string&);
    void            publishDetections(const std_msgs::Header&, ArucoDetections);

  private:
    // Private attributes
    ros::NodeHandle&                       _nh;
    ros::Subscriber                        _rs_image_subscriber;
    ros::Publisher                         _rs_aruco_pose_publisher;
    std::vector<int>                       _marker_ids;
    std::vector<int>                       _store_marker_ids;
    std::vector<double>                    _store_marker_lengths;
    std::vector<std::string>               _store_marker_frames;
    cv::Mat                                _current_image;
    cv::Mat                                _intrinsics_matrix;
    cv::Mat                                _dist_coeffs_vector;
    cv::Mat                                _obj_points;
    cv::Ptr<cv::aruco::DetectorParameters> _aruco_detector_parameters;
    cv::Ptr<cv::aruco::Dictionary>         _aruco_dictionary;
    ros::console::Level                    _log_level;
};

} // namespace catec_suministra
