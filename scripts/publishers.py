#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import pandas as pd
import sensor_msgs.point_cloud2 as pcl2

from cv_bridge import CvBridge
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import quaternion_from_euler

from misc import Object
from typing import Tuple, Dict

def publish_img(img_pub: rospy.Publisher, bridge: CvBridge, 
                image: cv2.Mat, boxes: np.ndarray, types: np.ndarray) -> None:
    """
    @param img_pub: ros publisher of camera image.
    @param bridge : cv bridge for image format conversion.
    @param image  : processing image frame.
    """
    detection_color_dict: Dict[str, Tuple[int]] = {
        'Car':        (255, 255,   0),
        'Cyclist':    (141,  40, 255),
        'Pedestrian': (  0, 255, 255)
    }
    
    for type, box in zip(types, boxes):
        tl = int(box[0]), int(box[1])
        br = int(box[2]), int(box[3])
        cv2.rectangle(image, tl, br, detection_color_dict[type], 2)
    
    img_pub.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))


def publish_pcl(pcl_pub: rospy.Publisher, point_cloud: np.ndarray) -> None:
    """
    @param pcl_pub: ros publisher of point cloud map.
    @param point_cloud: the raw data of point clouds.
    """
    header: Header  = Header()
    header.frame_id = 'map'
    header.stamp    = rospy.Time.now()
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))


def publish_ego(ego_pub: rospy.Publisher) -> None:
    """
    Publish the vehicle model and view point line fot the camera
    as the marker array for the topic.
    @param ego_pub: the publisher for the marker array.
    """
    marker_array: MarkerArray = MarkerArray()

    ## marker for view point line 
    marker: Marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()

    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration()

    rate = rospy.Rate(10)
    marker.color.r: float = 0.0
    marker.color.g: float = 1.0
    marker.color.b: float = 0.0
    marker.color.a: float = 1.0
    marker.scale.x: float = 0.1

    quaternion_0 = quaternion_from_euler(0, 0, 0)
    marker.pose.orientation.x = quaternion_0[0]
    marker.pose.orientation.y = quaternion_0[1]
    marker.pose.orientation.z = quaternion_0[2]
    marker.pose.orientation.w = quaternion_0[3]

    marker.points = []
    marker.points.append(Point(10,-10, 0))
    marker.points.append(Point( 0,  0, 0))
    marker.points.append(Point(10, 10, 0))

    ## mesh marker for vehicle models
    mesh_marker: Marker = Marker()
    mesh_marker.header.frame_id = 'map'
    mesh_marker.header.stamp = rospy.Time.now()

    mesh_marker.id = -1
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.mesh_resource = 'package://kitti_dataset_visualization/rviz/vehicle.dae'

    mesh_marker.pose.position.x: float =  0.0
    mesh_marker.pose.position.y: float =  0.0
    mesh_marker.pose.position.z: float = -1.73

    quater = quaternion_from_euler(np.pi / 2, 0, np.pi)
    mesh_marker.pose.orientation.x = quater[0]
    mesh_marker.pose.orientation.y = quater[1]
    mesh_marker.pose.orientation.z = quater[2]
    mesh_marker.pose.orientation.w = quater[3]

    mesh_marker.color.r: float = 1.0
    mesh_marker.color.g: float = 1.0
    mesh_marker.color.b: float = 1.0
    mesh_marker.color.a: float = 1.0
    mesh_marker.scale.x: float = 0.9
    mesh_marker.scale.y: float = 0.9
    mesh_marker.scale.z: float = 0.9

    marker_array.markers.append(marker)
    marker_array.markers.append(mesh_marker)

    ego_pub.publish(marker_array)


def publish_imu(imu_pub: rospy.Publisher, data: pd.DataFrame) -> None:
    """
    @param imu_pub: the publisher for publishing imu sensoring data.
    @param data: imu data from the kitti dataset.
    """
    imu: Imu = Imu()
    imu.header.frame_id = 'map'
    imu.header.stamp = rospy.Time.now()

    quater = quaternion_from_euler(float(data.roll ), 
                                   float(data.pitch),
                                   float(data.yaw  ))
    imu.orientation.x = quater[0]
    imu.orientation.y = quater[1]
    imu.orientation.z = quater[2]
    imu.orientation.w = quater[3]

    imu.linear_acceleration.x = data.af 
    imu.linear_acceleration.y = data.al 
    imu.linear_acceleration.z = data.au

    imu.angular_velocity.x = data.wf 
    imu.angular_velocity.y = data.wl 
    imu.angular_velocity.z = data.wu

    imu_pub.publish(imu)


def publish_gps(gps_pub: rospy.Publisher, data: pd.DataFrame) -> None:
    """
    @param gps_pub: the publisher for the gps information.
    @param data: the sensoring data from the kitti dataset.
    """
    gps: NavSatFix = NavSatFix()
    gps.header.frame_id = 'map'
    gps.header.stamp = rospy.Time.now()

    gps.latitude  = data.lat 
    gps.altitude  = data.alt
    gps.longitude = data.lon

    gps_pub.publish(gps)

def publish_box(box_pub: rospy.Publisher, bboxes: list, types: np.ndarray) -> None:
    """
    @param box_pub: 
    @param bboxes:
    @param types:
    """
    detection_color_dict = {
        'Car':        (255, 255,   0),
        'Cyclist':    (141,  40, 255),
        'Pedestrian': (  0, 255, 255)
    }

    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0], # lower face
        [4, 5], [5, 6], [6, 7], [7, 4], # upper face
        [4, 0], [5, 1], [6, 2], [7, 3], # connect upper and lower
        [4, 1], [5, 0]                  # frone face
    ]

    marker_array: MarkerArray = MarkerArray()
    for idx, bbox in enumerate(bboxes):
        marker: Marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()

        marker.id = idx
        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST
        marker.lifetime = rospy.Duration(0.5)

        marker.color.b = detection_color_dict[types[idx]][0] / 255.0
        marker.color.g = detection_color_dict[types[idx]][1] / 255.0
        marker.color.r = detection_color_dict[types[idx]][2] / 255.0
        marker.color.a = 1.0
        marker.scale.x = 0.1

        quaternion_0 = quaternion_from_euler(0, 0, 0)
        marker.pose.orientation.x = quaternion_0[0]
        marker.pose.orientation.y = quaternion_0[1]
        marker.pose.orientation.z = quaternion_0[2]
        marker.pose.orientation.w = quaternion_0[3]

        marker.points = []
        for line in lines:
            p1, p2 = bbox[line[0]], bbox[line[1]]
            marker.points.append(Point(p1[0], p1[1], p1[2]))
            marker.points.append(Point(p2[0], p2[1], p2[2]))
        
        marker_array.markers.append(marker)
    
    box_pub.publish(marker_array)


def publish_loc(loc_pub: rospy.Publisher, tracker: Dict[int, Object], 
                center: Dict[int, np.ndarray]) -> None:
    marker_array: MarkerArray = MarkerArray()

    for idx in center:
        marker: Marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()

        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.5)
        marker.type = Marker.LINE_STRIP
        marker.id = idx

        marker.color.r = tracker[idx].color[0]
        marker.color.g = tracker[idx].color[1]
        marker.color.b = tracker[idx].color[2]
        marker.color.a = 1.0
        marker.scale.x = 0.1

        quaternion_0 = quaternion_from_euler(0, 0, 0)
        marker.pose.orientation.x = quaternion_0[0]
        marker.pose.orientation.y = quaternion_0[1]
        marker.pose.orientation.z = quaternion_0[2]
        marker.pose.orientation.w = quaternion_0[3]

        marker.points: list = []
        for point in tracker[idx].locations:
            marker.points.append(Point(point[0], point[1], 0))

        marker_array.markers.append(marker)
    
    loc_pub.publish(marker_array)


def publish_dis(dis_pub: rospy.Publisher, minPQDs: np.ndarray) -> None:
    marker_array: MarkerArray = MarkerArray()

    for idx, (min_p, min_q, min_d) in enumerate(minPQDs):
        marker: Marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()

        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.5)
        marker.type = Marker.LINE_STRIP
        marker.id = idx

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        marker.scale.x = 0.1

        quaternion_0 = quaternion_from_euler(0, 0, 0)
        marker.pose.orientation.x = quaternion_0[0]
        marker.pose.orientation.y = quaternion_0[1]
        marker.pose.orientation.z = quaternion_0[2]
        marker.pose.orientation.w = quaternion_0[3]

        marker.points: list = []
        marker.points.append(Point(min_p[0], min_p[1], 0))
        marker.points.append(Point(min_q[0], min_q[1], 0))
        marker_array.markers.append(marker)

        text_marker: Marker = Marker()
        text_marker.header.frame_id = 'map'
        text_marker.header.stamp = rospy.Time().now()

        text_marker.id = idx + 1000
        text_marker.action = Marker.ADD
        text_marker.lifetime = rospy.Duration(0.5)
        text_marker.type = Marker.TEXT_VIEW_FACING

        p = (min_p + min_q) / 2
        text_marker.pose.position.x = p[0]
        text_marker.pose.position.y = p[1]
        text_marker.pose.position.z = 0.0

        text_marker.text = f'{min_d:.2f}'

        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 0.8
        text_marker.scale.x = 1
        text_marker.scale.y = 1
        text_marker.scale.z = 1
        marker_array.markers.append(text_marker)
    
    dis_pub.publish(marker_array)

