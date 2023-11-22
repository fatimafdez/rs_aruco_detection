#include "rs_aruco_detection/aruco_marker_position.h"

#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h" // do i need this libraries in here? or in the header?

namespace catec_suministra {

ArucoMarkerPosition::ArucoMarkerPosition(ros::NodeHandle& nh) :
        _nh(nh) // Constructor
                // LAMBDA - is defined with the sintax [capture list] (function arguments) {body}
                // Apparently this code store inside the subscriber object what it needs to do, it's more clean
                // & indicates that we are passing the object by reference, so we are not copying it, we are just
                // passing the pointer to the object
                // [&] indicates to capture everything, so we have access to all the variables and methods of the class,
                // as if we write everything outside the lambda function (ros::NodeHandle& nh) : _nh(nh) -> the _nh(nh)
                // -> nh is define in the cpp file, which creates the node handler, in the header file, we assign a
                // specific space for the node handler, and in this line we associate the node handler created in the
                // cpp to the espace assigned in the header just handles everywhere i dont get it, why they are defined
                // again? i dont understand this (ros::NodeHandle& nh)
{
    ROS_DEBUG("[ArucoMarkerPosition::ArucoMarkerPosition]");

    std::string rs_image_topic;
    std::string camera_calibration_file;

    _nh.param<std::string>(
            "rs_image_topic",
            rs_image_topic,
            "/suministra_rs/color/image_raw"); // give the value of the parameter rs_image_topic to the variable
                                               // rs_image_topic, if the parameter is not defined, then use the default
                                               // value "/suministra_rs/color/image_raw"
    _nh.param<std::string>("calib_file_path", camera_calibration_file, "camera_calibration_file.yaml");

    std::cout << " Me quiero subscribir a " << rs_image_topic << std::endl; // print to see if it works

    std::pair<cv::Mat, std::vector<int>> calibration_parameters = getCalibrationParameters(camera_calibration_file); // Read the camera calibration parameters

    _aruco_detector_parameters = cv::aruco::DetectorParameters::create();
    _aruco_dictionary          = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    _obj_points                = (4, 1, CV_32FC3); // Setting a coordinate system
    //_camera_params.readFromFile(camera_calibration_file);

    _rs_image_subscriber = _nh.subscribe<sensor_msgs::Image>(
            rs_image_topic,
            ROS_TOPIC_BUFFER_SIZE,
            [&](const sensor_msgs::ImageConstPtr& image_msg) {
                ROS_DEBUG("[ArucoMarkerPosition::rs_subscriber]"); // Subscribe to the realsense image topic

                // Subscriber code

                detectArucosImages(image_msg); // callback detectArucosImageCallback called when a new image arrives

                // Tip! Use private methods here to keep the code clean
            },
            ros::VoidConstPtr(),
            ros::TransportHints()
                    .tcpNoDelay()); // subscribers and publishers follow a TCP/IP protocol for communication
                                    // internally it opens a socket and sends the data through it (by datagrams), this
                                    // protocol uses less bandwidth, making a compression of the data, which adds a
                                    // delay, so to reduce this delays (which is quite important for odometry, lidar,
                                    // imu ...) we use this line of code, this uses more bandwidth but reduces the
                                    // delay, so it's better for this kind of applications
}

ArucoMarkerPosition::~ArucoMarkerPosition() // Destructor
{
    ROS_DEBUG("[ArucoMarkerPosition::~ArucoMarkerPosition]");
}

void ArucoMarkerPosition::setPosition(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

void ArucoMarkerPosition::getPosition()
{
    ROS_DEBUG("[%s]", __PRETTY_FUNCTION__);

    std::cout << "ArucoMarkerPosition: " << this->x << ", " << this->y << ", " << this->z << std::endl;
}

void ArucoMarkerPosition::detectArucosImages(const sensor_msgs::ImageConstPtr& image_msg)
{
    ROS_DEBUG("[%s]", __PRETTY_FUNCTION__);

    try {
        std::cout << " Estoy en el callback " << std::endl;

        _current_image
                = cv_bridge::toCvShare(image_msg, "bgr8")->image; // Convert the ROS image message to an OpenCV image
        cv::Mat output_image = _current_image.clone();            // Copy the image to draw the markers

        std::string camera_calibration_file;
        cv::Mat     camera_matrix, dist_coeffs; // Variables to store the camera matrix and distortion coefficients
        std::vector<cv::Vec3d> rotation_vec, translation_vec; // Vectors to store rotation and translation
        std::vector<int>       marker_ids;                    // Vector to store the ids of the detected markers
        int                    num_markers = marker_ids.size();
        std::vector<std::vector<cv::Point2f>> marker_corners,
                rejected_candidates; // Vector to store the corners of the detected and rejected candidates

        // ros::param::get("~calibration_file", camera_calibration_file);
        // cv::FileStorage rs_params(camera_calibration_file, cv::FileStorage::READ);  // Read the camera calibration
        // file

        cv::aruco::detectMarkers(
                _current_image,
                _aruco_dictionary,
                marker_corners,
                marker_ids,
                _aruco_detector_parameters,
                rejected_candidates);

        // rs_params["camera_matrix"] >> camera_matrix;
        // rs_params["distortion_coefficients"] >> dist_coeffs;
        // rs_params.release();

        std::cout << " Detecto markers " << std::endl;

        // for (int i = 0; i < num_markers; i++)
        // {
        //     solvePnP(obj_points, marker_corners.at(i), camera_matrix, dist_coeffs, rotation_vec.at(i),
        //     translation_vec.at(i));
        // }

        for (const auto& id : marker_ids) // Print the ids of the detected markers
            std::cout << id << ' ';
        std::cout << std::endl;

        for (const auto& corner : marker_corners) // Print the corners of the detected markers
        {
            for (const auto& point : corner) std::cout << "(" << point.x << ", " << point.y << ") ";
            std::cout << std::endl;
        }

        if (marker_ids.size() > 0) // Draw the valid detected markers
            cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);

        cv::imshow("output_image", output_image); // Display the image in a window
        char key = (char)cv::waitKey(1);

        if (key == 27)
            return;

    } catch (cv_bridge::Exception& e) // executed if the try fails
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

std::pair<cv::Mat, cv::Mat> ArucoMarkerPosition::getCalibrationParameters(const std::string& filename)
{
    ROS_DEBUG("[%s]", __PRETTY_FUNCTION__);

    cv::Mat intrinsics_matrix, dist_coeffs_matrix;

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("[%s] Could not open file `%s`", ros::this_node::getName().data(), filename.data());
        throw std::runtime_error("Could not open file");
    }

    cv::FileNode colorNode = fs["color"];
    if (colorNode.empty()) {
        throw std::runtime_error("Could not find 'color' node in file: " + filename);
    }

    cv::FileNode intrinsicsNode = colorNode["intrinsics"];
    cv::FileNode distorsionNode = colorNode["distortion_coefficients"];

    double fx = intrinsicsNode[0];
    double fy = intrinsicsNode[1];
    double cx = intrinsicsNode[2];
    double cy = intrinsicsNode[3];
    int    k1 = distorsionNode[0];
    int    k2 = distorsionNode[1];
    int    p1 = distorsionNode[2];
    int    p2 = distorsionNode[3];

    intrinsics_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    dist_coeffs_matrix = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, 0);

    return std::make_pair(intrinsics_matrix, dist_coeffs_matrix);
}

// void ArucoMarkerPosition::publishArucoPositions()
//{

//}

} // namespace catec_suministra