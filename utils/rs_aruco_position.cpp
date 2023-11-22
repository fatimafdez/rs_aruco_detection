#include "rs_aruco_detection/aruco_marker_position.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rs_aruco_position"); // Initialize ROS
    ros::NodeHandle nh("~");                    // Create a ROS node handle

    catec_suministra::ArucoMarkerPosition rs_aruco_position(nh); // Instance of my class
    ros::MultiThreadedSpinner                       spinner(4);
    spinner.spin();
    return 0;
}