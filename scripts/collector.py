#!/usr/bin/python3

import cv2
import numpy as np
import pandas as pd


def read_img(path: str) -> cv2.Mat:
    return cv2.imread(path)


def read_pcl(path: str) -> pd.DataFrame:
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def read_imu(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = [
        'lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn', 've', 'vf', 'vl', 'vu', 
        'ax', 'ay', 'az', 'af', 'al', 'au', 'wx', 'wy', 'wz', 'wf', 'wl', 'wu', 
        'posacc', 'velacc', 'navstat', 'numsats', 'posmode', 'velmode', 'orimode'
    ]
    return df


def read_obj(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = [
        'frame', 'track_id', 'type', 'truncated', 'occluded', 'alpha', 
        'bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom', 
        'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y'
    ]
    df.loc[df['type'].isin(['Truck', 'Van', 'Tram']), 'type'] = 'Car'
    df = df[df['type'].isin(['Car', 'Cyclist', 'Pedestrian'])]
    return df
