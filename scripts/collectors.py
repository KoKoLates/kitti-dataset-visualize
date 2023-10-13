#!/usr/bin/env python3

import cv2
import numpy as np

def read_camera(path: str) -> cv2.Mat:
    return cv2.imread(path)

def read_point_cloud(path: str) -> np.ndarray:
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)
