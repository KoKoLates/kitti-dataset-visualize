#!/usr/bin/env python3

import os

from misc import *
from collectors import *
from publishers import *
from kitti_utils import *
from sensor_msgs.msg import Image, PointCloud2


DATA_PATH = (
    '/home/aiseed/koko/catkin_ws/src/kitti_dataset_visualization'
    '/dataset/2011_09_26/2011_09_26_drive_0005_sync/')
TRACKING_PATH = (
    '/home/aiseed/koko/catkin_ws/src/kitti_dataset_visualization/dataset/tracking/0000.txt'
)
CALIBRATION_PATH = (
    '/home/aiseed/koko/catkin_ws/src/kitti_dataset_visualization/dataset/2011_09_26/'
)
EGOCAR = np.array([[2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73], [-1.95, 0.9, -1.73], 
                   [2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]])

if __name__ == '__main__':
    rospy.init_node('kitti_node', anonymous=True)
    frame: int = 0
    bridge: CvBridge = CvBridge()
    df = read_tracking(TRACKING_PATH)
    calib = Calibration(CALIBRATION_PATH, from_video=True)

    # car: Object = Object(20, True)
    tracker = {} # id: object
    tracker[-1] = Object(np.array([0, 0]), 20, True)
    previous_data = None
    
    cam_pub = rospy.Publisher('kitti_img', Image,       queue_size=10)
    pcl_pub = rospy.Publisher('kitti_pcl', PointCloud2, queue_size=10)
    imu_pub = rospy.Publisher('kitti_imu', Imu,         queue_size=10)
    gps_pub = rospy.Publisher('kitti_gps', NavSatFix,   queue_size=10)
    ego_pub = rospy.Publisher('kitti_ego', MarkerArray, queue_size=10)
    box_pub = rospy.Publisher('kitti_3d',  MarkerArray, queue_size=10)
    loc_pub = rospy.Publisher('kitti_loc', MarkerArray, queue_size=10)
    dis_pub = rospy.Publisher('kitti_dis', MarkerArray, queue_size=10)

    while not rospy.is_shutdown():
        df_tracking_frame = df[df['frame'] == frame]
        types = np.array(df_tracking_frame['type'])
        boxes_2d: np.ndarray = np.array(
            df_tracking_frame[['bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom']])
        boxes_3d: np.ndarray = np.array(
            df_tracking_frame[['height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']])
        track_id: np.ndarray = np.array(df_tracking_frame['track_id'])
        
        corner_3d_velos = []
        centers: Dict[int, np.ndarray] = {} # id: center
        
        minPQDs: list = []

        for box_3d, id in zip(boxes_3d, track_id):
            corner_3d_cam2 = compute_3d_box_cam2(*box_3d)
            corner_3d_velo = calib.project_rect_to_velo(corner_3d_cam2.T)
            corner_3d_velos += [corner_3d_velo]
            centers[id] = np.mean(corner_3d_velo, axis=0)[:2]

            minPQDs += [min_distance_cuboids(EGOCAR, corner_3d_velo)]
        corner_3d_velos += [EGOCAR]
        centers[-1] = np.array([0, 0])
        types = np.append(types, 'Car')


        imu = read_imu(os.path.join(DATA_PATH, f'oxts/data/{frame:010d}.txt')) 
        img = read_img(os.path.join(DATA_PATH, f'image_02/data/{frame:010d}.png'))
        pcl = read_pcl(os.path.join(DATA_PATH, f'velodyne_points/data/{frame:010d}.bin'))

        # update the current position
        if previous_data is None:
            for idx in centers:
                tracker[idx] = Object(centers[idx], 20, True) if idx == -1 \
                    else Object(centers[idx], 10, False)
        
        else:
            translation: float = np.linalg.norm(imu[['vf', 'vl']]) * 0.1
            rotation: float = float(imu.yaw - previous_data.yaw)
            for idx in centers:
                if idx in tracker:
                    tracker[idx].update(translation, rotation, centers[idx])
                else:
                    tracker[idx] = Object(centers[idx], 20, True) if idx == -1 \
                    else Object(centers[idx], 10, False)

            for idx in tracker:
                if idx not in centers:
                    tracker[idx].update(translation, rotation, None)

        previous_data = imu

        publish_img(cam_pub, bridge, img, boxes_2d, types)
        publish_pcl(pcl_pub, pcl)
        publish_ego(ego_pub)
        publish_imu(imu_pub, imu)
        publish_gps(gps_pub, imu)
        publish_loc(loc_pub, tracker, centers)
        publish_box(box_pub, corner_3d_velos, types)
        publish_dis(dis_pub, minPQDs)

        rospy.Rate(10).sleep()
        frame = (frame + 1) % 154
        if not frame: [tracker[idx].release() for idx in tracker]

