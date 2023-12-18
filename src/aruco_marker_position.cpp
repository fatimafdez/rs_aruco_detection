#include "rs_aruco_detection/aruco_marker_position.h"

namespace catec_suministra {

ArucoMarkerPosition::ArucoMarkerPosition(ros::NodeHandle& nh) : _nh(nh) // Constructor
{
    ROS_DEBUG("[%s]", __PRETTY_FUNCTION__);

    std::map<std::string, ros::console::levels::Level> logger;
    ros::console::get_loggers(logger);
    _log_level = logger[ROSCONSOLE_DEFAULT_NAME];

    std::string rs_image_topic, aruco_pose_topic, calibration_file, markers_length_file;

    _nh.param<std::string>(
            "rs_image_topic",
            rs_image_topic,
            "/suministra_rs/color/image_raw"); // give the value of the parameter rs_image_topic to the variable
                                               // rs_image_topic, if the parameter is not defined, then use the default
                                               // value "/suministra_rs/color/image_raw"

    _nh.param<std::string>("aruco_pose_topic", aruco_pose_topic, "/rs_aruco_detection/aruco_pose");
    _nh.param<std::string>("calibration_file", calibration_file, "rs_calibration_parameters.yaml");
    _nh.param<std::string>("markers_length_file", markers_length_file, "markers_length.yaml");

    std::cout << " Me quiero subscribir a " << rs_image_topic << std::endl; // print to see if it works

    readCalibrationParameters(calibration_file);
    readMarkersLength(markers_length_file);

    _aruco_detector_parameters = cv::aruco::DetectorParameters::create();
    _aruco_dictionary          = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    _obj_points                = (4, 1, CV_32FC3); // Setting a coordinate system

    _rs_aruco_pose_publisher
            = _nh.advertise<suministra_msgs::MarkerDetectionArray>(aruco_pose_topic, ROS_TOPIC_BUFFER_SIZE);

    _rs_image_subscriber = _nh.subscribe<sensor_msgs::Image>(
            rs_image_topic,
            ROS_TOPIC_BUFFER_SIZE,
            [&](const sensor_msgs::ImageConstPtr& image_msg) {
                ROS_DEBUG("[ArucoMarkerPosition::rs_subscriber]");
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
                ArucoDetections aruco_detections = detectArucosPoses(image_msg);
                publishDetections(image_msg->header, aruco_detections);
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
    ROS_DEBUG("[%s]", __PRETTY_FUNCTION__);
}

void ArucoMarkerPosition::readCalibrationParameters(
        const std::string&
                calibration_filename) // this function reads a file with the calibration parameters and store the values
                                      // in the variables _intrinsic_matrix and _dist_coeffs_vector
{
    ROS_DEBUG("[%s]", __PRETTY_FUNCTION__);

    cv::FileStorage fs(calibration_filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("[%s] Could not open file `%s`", ros::this_node::getName().data(), calibration_filename.data());
        throw std::runtime_error("Could not open file");
    }

    cv::FileNode color_node = fs["color"];
    if (color_node.empty()) {
        throw std::runtime_error("Could not find 'color' node in file: " + calibration_filename);
    }

    cv::FileNode intrinsics_node = color_node["intrinsics"];
    cv::FileNode distortion_node = color_node["distortion_coefficients"];

    double fx = intrinsics_node[0];
    double fy = intrinsics_node[1];
    double cx = intrinsics_node[2];
    double cy = intrinsics_node[3];
    double k1 = distortion_node[0];
    double k2 = distortion_node[1];
    double p1 = distortion_node[2];
    double p2 = distortion_node[3];

    _intrinsics_matrix  = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    _dist_coeffs_vector = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, 0);

    ROS_INFO(
            "[%s] Calibration parameters loaded from file `%s`",
            ros::this_node::getName().data(),
            calibration_filename.data());
    ROS_INFO("fx: %f, fy: %f", _intrinsics_matrix.at<double>(0, 0), _intrinsics_matrix.at<double>(1, 1));
    ROS_INFO("cx: %f, cy: %f", _intrinsics_matrix.at<double>(0, 2), _intrinsics_matrix.at<double>(1, 2));
    ROS_INFO("k1: %f, k2: %f", _dist_coeffs_vector.at<double>(0, 0), _dist_coeffs_vector.at<double>(0, 1));
    ROS_INFO("p1: %f, p2: %f", _dist_coeffs_vector.at<double>(0, 3), _dist_coeffs_vector.at<double>(0, 4));
}

void ArucoMarkerPosition::readMarkersLength(
        const std::string&
                markers_length_filename) // this reads the markers_configuration.yaml file and store the values in the
                                         // variables _store_marker_ids, _store_marker_lengths and _store_marker_frames
{
    ROS_DEBUG("[%s]", __PRETTY_FUNCTION__);

    cv::FileStorage fs(markers_length_filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("[%s] Could not open file `%s`", ros::this_node::getName().data(), markers_length_filename.data());
        throw std::runtime_error("Could not open file");
    }

    cv::FileNode markers_length_node = fs["markers"];
    if (markers_length_node.empty()) {
        throw std::runtime_error("Could not find 'markers' node in file: " + markers_length_filename);
    }

    cv::FileNode ids_node     = markers_length_node["ids"];
    cv::FileNode lengths_node = markers_length_node["lengths"];
    cv::FileNode frames_node  = markers_length_node["frames"];

    ROS_INFO("[%s] Ids length stored in file '%s'", ros::this_node::getName().data(), markers_length_filename.data());

    ids_node >> _store_marker_ids;
    lengths_node >> _store_marker_lengths;
    frames_node >> _store_marker_frames;

    for (int i = 0; i < _store_marker_ids.size() - 1; i++) {
        ROS_INFO("Stored Ids %d: %d", i + 1, _store_marker_ids[i]);
    }
}

ArucoDetections ArucoMarkerPosition::detectArucosPoses(const sensor_msgs::ImageConstPtr& image_msg)
{
    ROS_DEBUG("[%s]", __PRETTY_FUNCTION__);

    try {
        double                                length;
        int                                   freq, vector_position = 0;
        double                                theta, s;
        cv::Mat                               output_image;
        cv::Vec3d                             axis, rotation_vector;
        cv::Vec4d                             quaternion;
        std::vector<int>                      marker_ids_process, marker_ids_process_ordered;
        std::vector<double>                   marker_lengths_process, marker_lengths_process_ordered;
        std::map<double, int>                 marker_lengths_frequency;
        std::vector<std::string>              marker_frames_process, marker_frames_process_ordered;
        std::vector<cv::Vec4d>                quaternion_vecs_out;
        std::vector<cv::Vec3d>                rotation_vecs, rotation_vecs_draw, translation_vecs, translation_vecs_out;
        std::vector<std::vector<cv::Point2f>> marker_corners, marker_corners_process, marker_corners_process_ordered,
                marker_corners_input, rejected_candidates;

        _current_image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        output_image   = _current_image.clone();

        // aruco function to detect markers, it returns the four corners of the marker, the ids, and the rejected
        // candidates
        cv::aruco::detectMarkers(
                _current_image,
                _aruco_dictionary,
                marker_corners,
                _marker_ids,
                _aruco_detector_parameters,
                rejected_candidates);

        for (int i = 0; i < _marker_ids.size();
             i++) { // this loop checks if the detected markers are stored in the markers_configuration.yaml file
            auto it = std::find(_store_marker_ids.begin(), _store_marker_ids.end(), _marker_ids[i]);

            if (it != _store_marker_ids.end()) { // if the marker is stored, then it is going to be process
                int position = std::distance(_store_marker_ids.begin(), it); // this gives the position of the marker
                                                                             // in the markers_configuration.yaml file

                marker_corners_process.push_back(
                        marker_corners[i]); // store the corners of the marker that are going to be process due to the
                                            // fact that they are stored in the markers_configuration.yaml file
                marker_ids_process.push_back(_marker_ids[i]);
                marker_lengths_process.push_back(_store_marker_lengths[position]);
                marker_frames_process.push_back(_store_marker_frames[position]);

            } else {
                ROS_WARN(
                        "The aruco marker with id %d is not going to be process.",
                        _marker_ids[i]); // if the marker detectec is not stored we will get a warning saying that the
                                         // marker is not going to be process
            }
        }

        std::vector<size_t> indices(marker_lengths_process.size()); // creates a vector with indices
        std::iota(indices.begin(), indices.end(), 0); // fill the vector with values from 0 to the size of the vector
                                                      // marker_lengths_process
        std::sort(indices.begin(), indices.end(), [&marker_lengths_process](size_t i1, size_t i2) { // LAMBDA FUNCTION
            return marker_lengths_process[i1]
                 < marker_lengths_process[i2]; // sorting the vector indices according to the ascending values so the
                                               // output would be an indices vector that may be looking like this {3 1 2
                                               // 0} instead of {0 1 2 3}
            // if two values are equal, then the order is not changed
        });

        marker_ids_process_ordered.resize(marker_lengths_process.size()); // resize new vectors to store the values
        marker_lengths_process_ordered.resize(marker_lengths_process.size());
        marker_frames_process_ordered.resize(marker_lengths_process.size());
        marker_corners_process_ordered.resize(marker_lengths_process.size());

        for (size_t i = 0; i < marker_lengths_process.size();
             i++) { // store the values in the new vectors according to the order of the indices vector
            marker_ids_process_ordered[i]     = marker_ids_process[indices[i]];
            marker_lengths_process_ordered[i] = marker_lengths_process[indices[i]];
            marker_frames_process_ordered[i]  = marker_frames_process[indices[i]];
            marker_corners_process_ordered[i] = marker_corners_process[indices[i]];
            ROS_DEBUG_STREAM("Marker lengths process ordered value: " << marker_lengths_process_ordered[i]);
        }

        for (double num : marker_lengths_process_ordered) {
            marker_lengths_frequency[num]++; // this loop look for the number of times a value is repeated in the vector
                                             // marker_lengths_process_ordered the output would look like this
                                             // {{0.066,3},{0.132,2}} which means that the value 0.066 is repeated 3
                                             // times and the value 0.132 is repeated 2 times
            ROS_DEBUG_STREAM("Marker lengths frequency value: " << marker_lengths_frequency[num]);
            ROS_DEBUG_STREAM("Marker lengths frequency key: " << num);
            ROS_DEBUG_STREAM("Marker lengths frequency size: " << marker_lengths_frequency.size());
        }

        auto freq_aux = marker_lengths_frequency.begin(); // iterator that points to the first element of the map
        for (int i = 0; i < marker_lengths_frequency.size();
             i++) {                    // if there are 2 different lengths this would be repeated 2 times
            length = freq_aux->first;  // length of the marker
            freq   = freq_aux->second; // number of times the length is repeated

            // we just want to process that have the length that is treated in this iteration so we create a vector with
            // the corners of the markers that have the same length
            marker_corners_input.clear();
            for (int i = vector_position; i < (vector_position + freq); i++) {
                marker_corners_input.push_back(marker_corners_process_ordered[i]);
            }

            ROS_DEBUG_STREAM("length: " << length);
            ROS_DEBUG_STREAM("freq: " << freq);

            // this function estimates the pose of the marker, it returns the rotation and translation vectors
            cv::aruco::estimatePoseSingleMarkers(
                    marker_corners_input,
                    length,
                    _intrinsics_matrix,
                    _dist_coeffs_vector,
                    rotation_vecs,
                    translation_vecs,
                    _obj_points);

            for (int i = 0; i < rotation_vecs.size();
                 i++) { // the rotation vector has 3 values, so we need to
                        // transform it to a quaternion and added to the quaternion_vecs_out
                rotation_vector = rotation_vecs[i];
                theta           = cv::norm(rotation_vector);
                s               = sin(theta / 2);
                axis            = rotation_vector / theta;
                quaternion[1]   = axis[0] * s;
                quaternion[2]   = axis[1] * s;
                quaternion[3]   = axis[2] * s;
                quaternion[0]   = cos(theta / 2);
                rotation_vecs_draw.push_back(rotation_vecs[i]);
                quaternion_vecs_out.push_back(quaternion);
                translation_vecs_out.push_back(
                        translation_vecs[i]); // the traslation vector is directly added to the translation_vecs_out
            }

            vector_position = vector_position + freq; // this is to know the position of the next marker that is going
                                                      // to be process
            freq_aux++;
            ROS_DEBUG_STREAM("vector_position: " << vector_position);
        }

        // VISUALIZATION
        if (_log_level == ros::console::levels::Debug) {
            for (int i = 0; i < marker_ids_process.size(); i++) {
                cv::aruco::drawDetectedMarkers(output_image, marker_corners, _marker_ids);
                cv::drawFrameAxes(
                        output_image,
                        _intrinsics_matrix,
                        _dist_coeffs_vector,
                        rotation_vecs_draw[i],
                        translation_vecs_out[i],
                        marker_lengths_process_ordered[i]);
            }
            cv::imshow("RealSense D435i", output_image);
            cv::waitKey(1);
        }

        ROS_DEBUG_STREAM("Image size: " << image_msg->width << "x" << image_msg->height);
        ROS_DEBUG_STREAM("Marker corners process: " << marker_corners_process.size());
        ROS_DEBUG_STREAM("Marker ids process: " << marker_ids_process.size());
        ROS_DEBUG_STREAM("Maker lengths process: " << marker_lengths_process.size());
        ROS_DEBUG_STREAM("Marker frames process: " << marker_frames_process.size());

        for (int i = 0; i < marker_ids_process.size(); i++) {
            ROS_DEBUG_STREAM("Translation vector: " << translation_vecs_out[i]);
            ROS_DEBUG_STREAM("Quaternion: " << quaternion_vecs_out[i]);
        }

        ArucoDetections aruco_detections; // this is the output of the function that is a structure
        aruco_detections.rvecs  = quaternion_vecs_out;
        aruco_detections.tvecs  = translation_vecs_out;
        aruco_detections.ids    = marker_ids_process;
        aruco_detections.frames = marker_frames_process;
        return aruco_detections;

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        ArucoDetections aruco_detections;
        return aruco_detections;
    }
}

void ArucoMarkerPosition::publishDetections(const std_msgs::Header& header, ArucoDetections aruco_detections)
{ // this function publishes the detections in a custom message called MarkerDetectionArray
    ROS_DEBUG("[%s]", __PRETTY_FUNCTION__);
    ROS_DEBUG_STREAM("Ids size: " << aruco_detections.ids.size());

    suministra_msgs::MarkerDetectionArray detections_msg;
    detections_msg.header.stamp    = header.stamp;
    detections_msg.header.frame_id = header.frame_id;

    if (aruco_detections.ids.size() == 0) {
        ROS_DEBUG_STREAM("Not messages to publish ");
        detections_msg.detections.resize(0);
    } else {
        for (int i = 0; i < aruco_detections.ids.size(); i++) {
            suministra_msgs::MarkerDetection detection;

            detection.marker_id          = aruco_detections.ids[i];
            detection.child_frame_id     = aruco_detections.frames[i];
            detection.pose.position.x    = aruco_detections.tvecs[i][0];
            detection.pose.position.y    = aruco_detections.tvecs[i][1];
            detection.pose.position.z    = aruco_detections.tvecs[i][2];
            detection.pose.orientation.x = aruco_detections.rvecs[i][0];
            detection.pose.orientation.y = aruco_detections.rvecs[i][1];
            detection.pose.orientation.z = aruco_detections.rvecs[i][2];
            detection.pose.orientation.w = aruco_detections.rvecs[i][3];
            detections_msg.detections.push_back(detection);
        }
    }
    _rs_aruco_pose_publisher.publish(detections_msg);
}

} // namespace catec_suministra
