#! /usr/bin/env python3

import cv2
import rospy
import numpy as np
import pandas as pd
import sensor_msgs.point_cloud2 as pcl2

from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from misc import Object
from typing import Dict


def publish_img(
    publisher: rospy.Publisher,
    bridge: CvBridge,
    image: cv2.Mat,
    boxes: np.ndarray,
    types: np.ndarray
) -> None:
    color_map: Dict[str, tuple[int, int, int]] = {
        'Car':        (255, 255,   0),
        'Cyclist':    (141,  40, 255),
        'Pedestrian': (  0, 255, 255),
    }

    for type, box in zip(types, boxes):
        x0 = int(box[0]), int(box[1])
        x1 = int(box[2]), int(box[3])
        cv2.rectangle(image, x0, x1, color_map[type], 2)

    publisher.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))


def publish_pcl(
    publisher: rospy.Publisher,
    point_cloud: np.ndarray
) -> None:
    header = Header()
    header.frame_id = 'map'
    header.stamp = rospy.Time.now()
    publisher.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))


def publish_dae(publisher: rospy.Publisher) -> None:
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration()
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.1

    q = quaternion_from_euler(0, 0, 0)
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]
    marker.points = [
        Point(10, -10, 0), Point(0, 0, 0), Point(10, 10, 0)
    ]

    # mesh marker for models
    mesh_marker = Marker()
    mesh_marker.header.frame_id = 'map'
    mesh_marker.header.stamp = rospy.Time.now()
    mesh_marker.id = -1
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.lifetime = rospy.Duration(0.5)
    mesh_marker.mesh_resource = "package://kitti_dataset_visualize/rviz/vehicle.dae"
    mesh_marker.color.r = 1.0
    mesh_marker.color.g = 1.0
    mesh_marker.color.b = 1.0
    mesh_marker.color.a = 1.0
    mesh_marker.scale.x = 0.9
    mesh_marker.scale.y = 0.9
    mesh_marker.scale.z = 0.9

    q = quaternion_from_euler(np.pi / 2, 0, np.pi)
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w = q[3]
    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = -1.73

    marker_array = MarkerArray()
    marker_array.markers.append(marker)
    marker_array.markers.append(mesh_marker)
    publisher.publish(marker_array)


def publish_box(
    publisher: rospy.Publisher,
    bbox: list,
    type: np.ndarray
) -> None:
    color_map: Dict[str, tuple] = {
        'Car':        (255, 255,   0),
        'Cyclist':    (141,  40, 255),
        'Pedestrian': (  0, 255, 255)
    }

    lines: list[list[int, int]] = [
        [0, 1], [1, 2], [2, 3], [3, 0], # lower face
        [4, 5], [5, 6], [6, 7], [7, 4], # upper face
        [4, 0], [5, 1], [6, 2], [7, 3], # connect upper and lower
        [4, 1], [5, 0]                  # frone face
    ]

    marker_array = MarkerArray()
    for id, box in enumerate(bbox):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.id = id
        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST
        marker.lifetime = rospy.Duration(0.5)
        marker.color.b = color_map[type[id]][0] / 255.0
        marker.color.g = color_map[type[id]][1] / 255.0
        marker.color.r = color_map[type[id]][2] / 255.0
        marker.color.a = 1.0
        marker.scale.x = 0.1

        q = quaternion_from_euler(0, 0, 0)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.points = []
        for l in lines:
            p1, p2 = box[l[0]], box[l[1]]
            marker.points.append(Point(p1[0], p1[1], p1[2]))
            marker.points.append(Point(p2[0], p2[1], p2[2]))
        
        marker_array.markers.append(marker)

    publisher.publish(marker_array)


def publish_pos(
    publisher: rospy.Publisher,
    poses: Dict[int, Object],
) -> None:
    marker_array = MarkerArray()

    for id in poses:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.5)
        marker.type = Marker.LINE_STRIP
        marker.id = id
        marker.color.r = poses[id].color[0]
        marker.color.g = poses[id].color[1]
        marker.color.b = poses[id].color[2]
        marker.color.a = 1.0
        marker.scale.x = 0.1

        q = quaternion_from_euler(0, 0, 0)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.points = []
        for p in poses[id].locations:
            marker.points.append(Point(p[0], p[1], 0))

        marker_array.markers.append(marker)
    
    publisher.publish(marker_array)


def publish_len(
    publisher: rospy.Publisher,
    min_pqd: np.ndarray
) -> None:
    marker_array = MarkerArray()

    for id, (min_p, min_q, min_d) in enumerate(min_pqd):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.5)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        marker.scale.x = 0.1

        q = quaternion_from_euler(0, 0, 0)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        marker.points = []
        marker.points.append(Point(min_p[0], min_p[1], 0))
        marker.points.append(Point(min_q[0], min_q[1], 0))
        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header.frame_id = 'map'
        text_marker.header.stamp = rospy.Time.now()
        text_marker.id = id + 1000
        text_marker.action = Marker.ADD
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.lifetime = rospy.Duration(0.5)
        text_marker.text = f'{min_d:.2f}'

        p = (min_p + min_q) / 2
        text_marker.pose.position.x = p[0]
        text_marker.pose.position.y = p[1]
        text_marker.pose.position.z = 0.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 0.8
        text_marker.scale.x = 1
        text_marker.scale.y = 1
        text_marker.scale.z = 1
        marker_array.markers.append(text_marker)

    publisher.publish(marker_array)
