#! /usr/bin/python3
import numpy as np
from collections import deque

class Object(object):
    def __init__(self, center: np.ndarray, max_len: int, type: bool) -> None:
        self.max_len: int = max_len
        self.locations: deque = deque(maxlen=max_len)
        self.locations.appendleft(center)
        self.color: list = [1.0, 0.0, 0.0] if type else [1.0, 1.0, 0.0]
        
    def update(self, translation: float, rotation: float, center: np.ndarray) -> None:
        for idx in range(len(self.locations)):
            x0, y0 = self.locations[idx]
            x1 =  x0 * np.cos(rotation) + y0 * np.sin(rotation) - translation
            y1 = -x0 * np.sin(rotation) + y0 * np.cos(rotation)
            self.locations[idx] = np.array([x1, y1])
        
        if center is not None:
            self.locations.appendleft(center)

    def release(self) -> None:
        self.locations = deque(maxlen=self.max_len)


def compute_3d_box_cam2(h, w, l, x, y, z, yaw) -> np.ndarray:
    """ Compute the 3-dimensional coordinate of cube (8 vertices) in camera coordinates.
    parsing the boundary data from the camera sensor with labeled in kitti dataset.
    @param h, w, l: height, width and length of object
    @param x, y, z: the x, y and z coordinates of object
    @param yaw: rotation of object in radians
    @return the coordinates of cube with 8-vertices
    @rtype: np.ndarray
    """
    R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]]) # rotation matrix
    x_corners = [l/2, l/2,-l/2,-l/2, l/2, l/2,-l/2,-l/2]
    y_corners = [  0,   0,   0,   0,  -h,  -h,  -h,  -h]
    z_corners = [w/2,-w/2,-w/2, w/2, w/2,-w/2,-w/2, w/2]
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    corners_3d_cam2 += np.vstack([x, y, z])
    return corners_3d_cam2


def distance_point_2_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> tuple:
    """ calculate the min distance of a point p to segment ab
    return the minimum distance and the intersect point q
    @param p: the outside point
    @param a, b: the end point of line segment
    @return: minimum distance and the intersect point q
    @rtype: tuple[float, np.ndarray]
    """
    ap = p - a
    bp = p - b
    ab = b - a

    if np.dot(ab, ap) >= 0 and np.dot(-ab, bp) >= 0:
        return np.abs(np.cross(ap, bp)) / np.linalg.norm(ab), np.dot(ap, ab) / np.dot(ab, ab) * ab + a

    d_ap = np.linalg.norm(ap)
    d_bp = np.linalg.norm(bp)
    return (d_ap, a) if d_ap < d_bp else (d_bp, b)


def min_distance_cuboids(cube1: np.ndarray, cube2: np.ndarray) -> tuple:
    """ Calculate the min distance between two cube.
    @param cube1, cube2: two cube to be calculated
    @return: the point in `cube1` and `cube2` and the min distance
    @rtype: tuple[np.ndarray, np.ndarray, float]
    """
    min_d = 1e5
    for i in range(4):
        for j in range(4):
            d, Q = distance_point_2_segment(cube1[i, :2], cube2[j, :2], cube2[j+1, :2])
            if d < min_d:
                min_d = d
                min_p = cube1[i, :2]
                min_q = Q
    for i in range(4):
        for j in range(4):
            d, Q = distance_point_2_segment(cube2[i, :2], cube1[j, :2], cube1[j+1, :2])
            if d < min_d:
                min_d = d
                min_p = cube2[i, :2]
                min_q = Q
    return min_p, min_q, min_d