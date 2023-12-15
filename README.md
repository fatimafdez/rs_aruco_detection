# RS_ARUCO_DETECTOR #

This repository contains the code for the Aruco detector node. It provides a topic output which includes the Aruco poses of the detected markers in the camera frame whose ids are stored in the `markers_configuration.yaml` file. If the detected markers are not listed in this file, a warning message will be printed.

### Dependencies ###

* [ROS](http://wiki.ros.org/ROS/Installation)
* [OpenCV](https://opencv.org/)
* [suministra_msgs](https://bitbucket.org/fadacatec-ondemand/suministra_msgs/src/main/?search_id=a9a5f567-79df-4e19-8e30-80e3c38aa881)