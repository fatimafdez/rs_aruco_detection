
#include "ros/ros.h"
#include "std_msgs/String.h"
// from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
// OpenCV library
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_position_detector"); // aruco_position_detector -> node name
    ros::NodeHandle nh; // comunicate with ROS system

    ros::Publisher position_pub = nh.advertise<std_msgs::String>("rs_aruco_detector", 1000); // rs_aruco_detector -> topic name
    ros::Rate loop_rate(10); // 10Hz

    int count = 0;
    while (ros::ok()) // while ROS is running
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "aruco_position_detector: " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        position_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
}