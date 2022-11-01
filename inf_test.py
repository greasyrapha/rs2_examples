#! /usr/bin/env python3

import time
import math
import numpy as np
import pyrealsense2 as rs

class Test:
    def __init__(self):
        self.color_stream_width = 960 #1280
        self.color_stream_height = 540 #720
        self.depth_stream_width = 640 #1024
        self.depth_stream_height = 480 #768
        self.stream_fps = 30

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.color_stream_width, self.color_stream_height, rs.format.bgr8, self.stream_fps)
        self.config.enable_stream(rs.stream.depth, self.depth_stream_width, self.depth_stream_height, rs.format.z16, self.stream_fps)

        self.profile = self.pipeline.start(self.config)

        self.ObjectPosition = np.array([["class_id", "ObjectClass", "x", "y", "z", "sumx", "sumy", "sumz", "num"]])

        for i in range(0, 9):
            self.Tester()
        #while True:
        #    self.Tester()

    def Tester(self):
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()


        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        ObjectClass = 'Test'
        ObjectBbox = [640, 480]

        ###Timer start###
        Start = time.time()    

        CameraObjectPosition = rs.rs2_deproject_pixel_to_point(depth_intrin, (320, 240), depth_frame.get_distance(320, 240))
        RobotObjectPosition = [round(CameraObjectPosition[2], 4), -round(CameraObjectPosition[0], 4), -round(CameraObjectPosition[1], 4)]

        d_dist = math.sqrt(pow(RobotObjectPosition[0], 2) + pow(RobotObjectPosition[1], 2) + pow(RobotObjectPosition[2], 2))

        if d_dist > 0.05 and d_dist < 4:
            posx = RobotObjectPosition[0]
            posy = RobotObjectPosition[1]
            posz = RobotObjectPosition[2]

            w_class = np.where(self.ObjectPosition == ObjectClass)[0]
            n_class = w_class.shape[0]

            knn_score = 0
            knn_rst = 1

            dist_threshold = 0.6

            if n_class > 0:
                for i in w_class:
                    w_x = abs(posx - float(self.ObjectPosition[i, 1:][1]))
                    w_y = abs(posy - float(self.ObjectPosition[i, 1:][2]))
                    w_z = abs(posz - float(self.ObjectPosition[i, 1:][3])) 
                    dist = math.sqrt(pow(w_x, 2) + pow(w_y, 2) + pow(w_z, 2))
                    if dist >= dist_threshold:
                        knn_score += 1
                    elif dist < dist_threshold and dist >= 0.08:
                        self.ObjectPosition[i, 1:][7] = int(self.ObjectPosition[i, 1:][7]) + 1
                        self.ObjectPosition[i, 1:][4] = float(self.ObjectPosition[i, 1:][4]) + float(posx) 
                        self.ObjectPosition[i, 1:][5] = float(self.ObjectPosition[i, 1:][5]) + float(posy) 
                        self.ObjectPosition[i, 1:][6] = float(self.ObjectPosition[i, 1:][6]) + float(posz)
                        self.ObjectPosition[i, 1:][1] = float(self.ObjectPosition[i, 1:][4]) / float(self.ObjectPosition[i, 1:][7])
                        self.ObjectPosition[i, 1:][2] = float(self.ObjectPosition[i, 1:][5]) / float(self.ObjectPosition[i, 1:][7]) 
                        self.ObjectPosition[i, 1:][3] = float(self.ObjectPosition[i, 1:][6]) / float(self.ObjectPosition[i, 1:][7]) 
                        knn_rst = 0
                    elif dist < 0.1:
                        knn_rst = 0
                if knn_score * knn_rst != 0:
                    self.ObjectPosition = np.append(self.ObjectPosition, [[self.ObjectPosition.shape[0] - 1, ObjectClass, posx, posy, posz, posx, posy, posz, 1]], axis=0) #add det_points
            elif n_class == 0:
                self.ObjectPosition = np.append(self.ObjectPosition, [[self.ObjectPosition.shape[0] - 1, ObjectClass, posx, posy, posz, posx, posy, posz, 1]], axis=0) #add det_points

        ###Timer end###
        End = time.time()

        sec = str(End - Start)
        print("Inference time : " + sec)

def main():
    test = Test()

if __name__ == "__main__": 
    main()
