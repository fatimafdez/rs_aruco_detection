
#include <rs_aruco_detection/ArucoMarkerPosition.h>  // Include your header file

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs_aruco_position");  // Initialize ROS
    ros::NodeHandle nh;  // Create a ROS node handle

    aruco_marker_position_detections::ArucoPosition aruco_position(nh);  // Create an instance of your class
    ros::MultiThreadedSpinner spinner(4); 
    spinner.spin();
    return 0;
}