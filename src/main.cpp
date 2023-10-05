
#include <string>
#include <vector>

#include "ros/ros.h"
#include "kitti_dataset_visualization/camera_images.hpp"

int main(int argc, char* argv[]) {
    std::string main_dir = "../dataset/2011_09_26/2011_09_26_drive_0005_sync/";
    std::string image_dir = "image_02/data/";

    ros::init(argc, argv, "kitti_node");
    ros::NodeHandle nh;
    CameraImages image_publisher(nh, "kitti/raw_image", 10);

    ros::Rate loop_rate(10);
    while (nh.ok()) {
        static int kitti_idx{0};
        kitti_idx %= 154;
        std::stringstream ss;
        ss << std::setw(10) << std::setfill('0') << std::to_string(kitti_idx);
        std::string kitti_num = ss.str();
        
        image_publisher.Publish(main_dir + image_dir + kitti_num, main_dir, kitti_idx);

        kitti_idx++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}