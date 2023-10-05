#ifndef CAMERA_IMAGES_HPP
#define CAMERA_IMAGES_HPP

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <opencv2/highgui.hpp>

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

class CameraImages {
    public:
    CameraImages(ros::NodeHandle, std::string, int);
    bool Publish(std::string, std::string, int);

    private:
    ros::NodeHandle nh;
    std::string topic_name;
    image_transport::Publisher publisher;
};

#endif // CAMERA_IMAGES_HPP