#include "rs_aruco_detection/ArucoPosition.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs_aruco_position");  // Initialize ROS
    ros::NodeHandle nh("~");  // Create a ROS node handle

    catec_suministra_detection::ArucoMarkerPosition rs_aruco_position(nh);  // Instance of my class
    ros::MultiThreadedSpinner spinner(4); 
    spinner.spin();
    return 0;
}