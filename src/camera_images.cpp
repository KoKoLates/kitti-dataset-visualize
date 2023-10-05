#include "camera_images.hpp"

CameraImages::CameraImages(
    ros::NodeHandle nh_, std::string topic_name_, int queue_size=-1
) :nh(nh_), topic_name(topic_name_) {
    image_transport::ImageTransport it(nh);
    publisher = it.advertise(topic_name, queue_size);
}

bool CameraImages::Publish(
    std::string image_path, std::string tracking_path="", int image_frame=-1
) {
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "error: image path is error" << std::endl;
        return 1;
    }

    // tracking boundary box
    if (image_frame != -1) {
        //TODO
        std::cout << "tracking" << std::endl;
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    sensor_msgs::ImagePtr message = cv_bridge::CvImage(
        header, "bgr8", image).toImageMsg();
    publisher.publish(message);
    return 0;
}

