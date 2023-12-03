#!/usr/bin/env python3

import cv2
import numpy as np
import pandas as pd

def read_img(path: str) -> cv2.Mat:
    return cv2.imread(path)

def read_pcl(path: str) -> np.ndarray:
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)

def read_imu(path: str) -> np.ndarray:
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = [
        'lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn', 've', 'vf', 'vl', 'vu', 
        'ax', 'ay', 'az', 'af', 'al', 'au', 'wx', 'wy', 'wz', 'wf', 'wl', 'wu', 
        'posacc', 'velacc', 'navstat', 'numsats', 'posmode', 'velmode', 'orimode'
    ]
    return df 
