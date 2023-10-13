#!/usr/bin/env python3

from collectors import *
from publishers import *

DATA_PATH = ('/home/aiseed/koko/catkin_ws/src/kitti_dataset_visualization'
             '/dataset/2011_09_26/2011_09_26_drive_0005_sync/')

if __name__ == '__main__':
    rospy.init_node('kitti_node', anonymous=True)
    frame: int = 0
    bridge: CvBridge = CvBridge()
    camera_publisher = rospy.Publisher('kitti_raw_image', Image, queue_size=10)
    pcloud_publisher = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
    ego_vehicle_publisher = rospy.Publisher('kitti_ego_vehicle', Marker, queue_size=10)

    while not rospy.is_shutdown():
        image: cv2.Mat = read_camera(os.path.join(DATA_PATH, f'image_02/data/{frame:010d}.png'))
        point_cloud: np.ndarray = read_point_cloud(os.path.join(DATA_PATH, f'velodyne_points/data/{frame:010d}.bin'))

        publish_camera(camera_publisher, bridge, image)
        publish_point_cloud(pcloud_publisher, point_cloud)
        publish_ego_vehicle(ego_vehicle_publisher)

        rospy.loginfo('Topic published')
        rospy.Rate(10).sleep()
        frame = (frame + 1) % 154
