#! /usr/bin/env python3

import roslib
import rospy
from sensor_msgs.msg import Imu

import numpy as np
import pyrealsense2 as rs

def setImu(x, y, z, roll, pitch, yaw):
    imu = Imu()
    imu.header.frame_id = "/imu_link"
    imu.angular_velocity.x = -yaw
    imu.angular_velocity.y = roll
    imu.angular_velocity.z = -pitch
    imu.linear_acceleration.x = -z
    imu.linear_acceleration.y = x
    imu.linear_acceleration.z = -y
    pubimu.publish(self.imu)

def imu_1():
    frames = pipeline.wait_for_frames()

    accel_frame = frames[0].as_motion_frame().get_motion_data()
    gyro_frame = frames[1].as_motion_frame().get_motion_data()

    accel = np.asarray([accel_frame.x, accel_frame.y, accel_frame.z])
    gyro = np.asarray([gyro_frame.x, gyro_frame.y, gyro_frame.z])

    #print("accel : ", accel)
    #print("gyro : ", gyro)

    setImu(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2])

def main():
    print("IMU Node is up!")
    while true:
        imu_1()

if __name__ == "__main__": 
    rospy.init_node('imu')
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f) #Enable accelerometer 62.5, 250(Hz)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f) #Enable gyroscope 200, 400(Hz)
    profile = self.pipeline.start(self.config)

    pubimu = rospy.Publisher("/imu/data", Imu, queue_size=1)
    main()
