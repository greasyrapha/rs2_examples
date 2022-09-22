#! /usr/bin/env python3

import roslib
import rospy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped
from visualization_msgs.msg import Marker

import argparse
import math
import os
import os.path as osp
import sys
import cv2
import numpy as np

import pyrealsense2 as rs

ROOT = os.getcwd()
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from pathlib import Path
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

class Camera:
    def __init__(self):
        self.stream_width = 640
        self.stream_height = 480
        self.stream_fps = 30

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.stream_width, self.stream_height, rs.format.z16, self.stream_fps)
        self.config.enable_stream(rs.stream.color, self.stream_width, self.stream_height, rs.format.bgr8, self.stream_fps)

        self.profile = self.pipeline.start(self.config)

        while True:
            self.camera_1()

    def camera_1(self):
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()


        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

        color_image = np.asanyarray(color_frame.get_data())
        img0 = color_image

        cv2.imshow('Realsense D455', color_image)
        cv2.waitKey(1)

def main():
    camera = Camera()

if __name__ == "__main__": 
    rospy.init_node('rgbd')
    main()
