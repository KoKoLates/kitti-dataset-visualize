#!/usr/bin/python3
import os
import argparse

from misc import *
from publisher import *
from collector import *

from sensor_msgs.msg import Image, PointCloud2


def main() -> None:
    rospy.init_node('kitti_node', anonymous=True)
    frame: int = 0
    bridge = CvBridge()

    vehicle: np.ndarray = np.array([
        [2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73], [-1.95, 0.9, -1.73], 
        [2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]
    ])

    track_path = rospy.get_param("track", "DEFAULT_TRACK_PATH")
    calib_path = rospy.get_param("calib", "DEFAULT_CALIB_PATH")
    value_path = rospy.get_param("value", "DEFAULT_VALUE_PATH")

    track = read_obj(track_path)
    calib = Calibration(calib_path, from_video=True)

    img_pub = rospy.Publisher('kitti_img', Image,       queue_size=5)
    pcl_pub = rospy.Publisher('kitti_pcl', PointCloud2, queue_size=5)
    dae_pub = rospy.Publisher('kitti_dae', MarkerArray, queue_size=5)
    box_pub = rospy.Publisher('kitti_box', MarkerArray, queue_size=5)
    pos_pub = rospy.Publisher('kitti_pos', MarkerArray, queue_size=5)
    len_pub = rospy.Publisher('kitti_len', MarkerArray, queue_size=5)

    poses: Dict[int, Object] = {} # id: Object
    poses[-1] = Object(np.array([[0, 0]]), 20, True)

    previous: pd.DataFrame = None
    while not rospy.is_shutdown():
        frame_track = track[track['frame'] == frame]
        types_track = np.array(frame_track['type'])
        
        boxes_2d = np.array(frame_track[['bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom']])
        boxes_3d = np.array(frame_track[['height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']])
        track_id = np.array(frame_track['track_id'])

        ## update 3D boundary box
        centers: Dict[int, np.ndarray] = {} # id: center
        min_pqd: list = []
        corners: list = []

        for box_3d, id in zip(boxes_3d, track_id):
            corner_3d_cam2 = compute_3d_box_cam2(*box_3d)
            corner_3d_velo = calib.project_rect_to_velo(corner_3d_cam2.T)
            centers[id] = np.mean(corner_3d_velo, axis=0)[:2]

            min_pqd.append(min_distance_cuboids(vehicle, corner_3d_velo))
            corners.append(corner_3d_velo)

        centers[-1] = np.array([0, 0])
        corners.append(vehicle)
        types_track = np.append(types_track, 'Car')

        ## update data
        img = read_img(os.path.join(value_path, f'image_02/data/{frame:010d}.png'))
        imu = read_imu(os.path.join(value_path, f'oxts/data/{frame:010d}.txt'))
        pcl = read_pcl(os.path.join(value_path, f'velodyne_points/data/{frame:010d}.bin'))

        ## update current position
        if previous is not None:
            t: float = np.linalg.norm(imu[['vf', 'vl']]) * 0.1
            r: float = float((imu['yaw'] - previous['yaw']).iloc[0])
            for id in centers:
                if id not in poses:
                    poses[id] = Object(centers[id], 20, True) \
                    if id == -1 else Object(centers[id], 10, False)
                    continue
                poses[id].update(t, r, centers[id])

            for id in poses:
                if id not in centers:
                    poses[id].update(t, r, None)
        else:
            for id in centers:
                poses[id] = Object(centers[id], 20, True) \
                if id == -1 else Object(centers[id], 10, False)

        previous = imu
        
        publish_img(img_pub, bridge, img, boxes_2d, types_track)
        publish_pcl(pcl_pub, pcl)
        publish_dae(dae_pub)
        publish_pos(pos_pub, poses)
        publish_box(box_pub, corners, types_track)
        publish_len(len_pub, min_pqd)

        rospy.Rate(10).sleep()
        frame = (frame + 1) % 154
        if not frame:
            [poses[id].release() for id in poses]


if __name__ == "__main__":
    main()

