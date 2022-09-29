#! /usr/bin/env python3

import roslib
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped

import numpy as np
import pyrealsense2 as rs

class Camera_Imu:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f) #Enable accelerometer 62.5, 250(Hz)
        self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f) #Enable gyroscope 200, 400(Hz)
        self.profile = self.pipeline.start(self.config)

        self.subpose = rospy.Subscriber('/integrated_to_init', Odometry, self.poseCallback, queue_size=1)
        self.pubimu = rospy.Publisher("/imu", Imu, queue_size=1)

        try:
            while True:
                self.imu_1()
        finally:
            self.pipeline.stop()

    def poseCallback(self, slamout):
        self.orientation = [slamout.pose.pose.orientation.z, slamout.pose.pose.orientation.x, slamout.pose.pose.orientation.y, slamout.pose.pose.orientation.w]
        self.imu = Imu()
        self.imu.angular_velocity.x = self.gyro[0]
        self.imu.angular_velocity.y = self.gyro[1]
        self.imu.angular_velocity.z = self.gyro[2]
        self.imu.linear_acceleration.x = self.accel[0]
        self.imu.linear_acceleration.y = self.accel[1]
        self.imu.linear_acceleration.z = self.accel[2]
        self.imu.orientation.x = self.orientation[0]
        self.imu.orientation.y = self.orientation[1]
        self.imu.orientation.z = self.orientation[2]
        self.imu.orientation.w = self.orientation[3]
        self.pubimu.publish(self.imu)

    def imu_1(self):
        frames = self.pipeline.wait_for_frames()

        accel_frame = frames[0].as_motion_frame().get_motion_data()
        gyro_frame = frames[1].as_motion_frame().get_motion_data()

        self.accel = np.asarray([accel_frame.x, accel_frame.y, accel_frame.z])
        self.gyro = np.asarray([gyro_frame.x, gyro_frame.y, gyro_frame.z])

        
def main():
    camera_imu = Camera_Imu()

if __name__ == "__main__": 
    rospy.init_node('imu')
    print("IMU Node is Up!")
    main()
