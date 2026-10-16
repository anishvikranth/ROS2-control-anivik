#!/usr/bin/env python3
import numpy as np
from sensor_msgs.msg import CameraInfo


class CameraModel:
    """
    Even though the camera info is available in topics, for the models and all to use we will take them 
    and store into a file so that we have convinient numpy arrays instead of dealing with camera Info
    """

    def __init__(self):

        self.initialized = False

        # Image size
        self.width = None
        self.height = None

        # Intrinsic matrix
        self.K = None

        # Distortion coefficients
        self.D = None

        # Rectification matrix
        self.R = None

        # Projection matrix
        self.P = None

        # Baseline (meters)
        self.baseline = None

    def update(self, msg: CameraInfo):
       
        #Update the camera model from a CameraInfo message.       

        self.width = msg.width
        self.height = msg.height
        self.K = np.array(msg.k,dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d,dtype=np.float64)
        self.R = np.array(msg.r,dtype=np.float64).reshape(3, 3)
        self.P = np.array(msg.p,dtype=np.float64).reshape(3, 4)
        self.baseline = self.compute_baseline()
        self.initialized = True

    def compute_baseline(self):
        """
        Compute stereo baseline from the projection matrix.
        For a rectified stereo camera
        Tx = -fx * baseline
        """

        fx = self.P[0, 0]
        tx = self.P[0, 3]

        if abs(fx) < 1e-9:
            return 0.0

        return -tx / fx

    def is_ready(self):
        #Returns True once calibration has been received       
        return self.initialized

    def reset(self):        
        #To clear all stored calibration       
        self.__init__()

    def __str__(self):
# Without this if we just do print CameraModel it will be like this <common.camera_model.CameraModel object at 0x7fd29b51e890>
#This function is automatically called
        if not self.initialized:
            return "CameraModel(Not Initialized)"
        return (
            "CameraModel(\n"
            f"  Resolution : {self.width} x {self.height}\n"
            f"  Baseline   : {self.baseline:.4f} m\n"
            f"  K =\n{self.K}\n"
            f"  D = {self.D}\n"
            ")"
        )