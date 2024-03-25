#!/usr/bin/env python
import cv2
import numpy as np
import os
import quaternion
import rclpy
import sys

from copy import deepcopy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class OutlierRemover(Node):
    def __init__(self):
        super().__init__("outlier_remover_rpi_node")

        self.map_img = None
        self.map_origin = None
        self.yolo_poses = None

        if len(sys.argv) < 8:
            # self.init_map("cris_new_area_fixed.pgm", [-13.1, -7.62], [0.3, 8.6, 0.3141593], 0.05)
            # self.init_map("cris_new_area_fixed.pgm", [-13.1, -7.62], [-0.3, 8.3, 0.3141593], 0.05)
            print("Using default parameters.")
            self.init_map("cris_new_area_fixed.pgm", [-13.1, -7.62], [-0.4, 8.2, 0.3141593], 0.05)
        else:
            self.init_map(sys.argv[1], [float(sys.argv[2]), float(sys.argv[3])], [float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])], float(sys.argv[7]))
            
        self.dr_spaam_sub = self.create_subscription(
                    PoseArray, 
                    'dr_spaam_detections', 
                    self.dr_spaam_callback, 
                    10
                )

        self.yolo_sub = self.create_subscription(
                    PoseArray, 
                    'yolo_detections', 
                    self.yolo_callback, 
                    10
                )
        
        self.back_subtraction_sub = self.create_subscription(
                    PoseArray, 
                    'back_sub_dets', 
                    self.back_sub_callback, 
                    10
                )

        self.inliers_pub = self.create_publisher(
            PoseArray, "static_inliers", 10
        )

        self.back_sub_inliers_pub = self.create_publisher(
            PoseArray, "back_sub_static_inliers", 10
        )

    def init_map(self, map_img, map_origin, rpi_pose, map_scale):
        self.map_img = (255 - cv2.imread(map_img, 0))
        self.map_origin = map_origin
        self.rpi_pose = rpi_pose
        self.map_scale = map_scale        

    def yolo_callback(self, msg):
        poses = self.remove_outliers(msg)
        if poses is not None:
            self.inliers_pub.publish(poses)

    def dr_spaam_callback(self, msg):
        poses = self.remove_outliers(msg)
        if poses is not None:
            self.inliers_pub.publish(poses)

    def back_sub_callback(self, msg):
        poses = self.remove_outliers(msg)
        if poses is not None:
            self.back_sub_inliers_pub.publish(poses)

    def remove_outliers(self, msg):
        if self.map_img is None:
            return  
            
        TR = np.identity(3)
        a = self.rpi_pose[2]
        R = np.array([[np.cos(a),-np.sin(a),0], [np.sin(a),np.cos(a),0], [0,0,1]]) 
        TR = R
        TR[0, 2] = self.rpi_pose[0]
        TR[1, 2] = self.rpi_pose[1]

        print("TR:", TR)
        
        img = self.map_img.copy()
        img[img < 235] += 20
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        # from top-left
        map_offset_x = int(-self.map_origin[0] / 0.05)
        map_offset_y = self.map_img.shape[0] - int(-self.map_origin[1] / 0.05)

        cv2.circle(img, (map_offset_x, map_offset_y), 5, (0,0,255), -1)

        # since map_origin is given as the rpi pose, robot_pos is always (0,0)
        robot_pos = np.array([0,0]) 
        cv2.circle(img, (robot_pos[0]+ map_offset_x, -robot_pos[1]+map_offset_y), 5, (255,0,0), -1)

        pose_array = PoseArray()
        for pose in msg.poses:
            det = np.array([pose.position.x, pose.position.y, 1])
            det_in_mapf = TR @ det

            det_in_mapf_pixels = (det_in_mapf[:2] / 0.05).astype(int)
            det_in_mapf_pixels[1] = -det_in_mapf_pixels[1]

            cell_to_check = det_in_mapf_pixels + np.array([map_offset_x, map_offset_y])

            try:
                cv2.circle(img, tuple(cell_to_check), 10, (0,0,20), 3)
            except:
                print("cell has wrong format")
            # if map cell is free
            step = 10
            sum_obstacles = self.map_img[
                                cell_to_check[1]-step:cell_to_check[1]+step,
                                cell_to_check[0]-step:cell_to_check[0]+step
                            ].sum()

            if sum_obstacles < 3000:
                # print("Inlier:", det_in_mapf)
                # dets_msg.header = msg.header
                # dets_msg.header.frame_id = "mobile_base_double_lidar" 
                pose_array.poses.append(pose)
                try:
                    cv2.circle(img, tuple(cell_to_check), 10, (0,0,255), 3)
                except:
                    print("cell has wrong format")
        # convert to ros msg and publish
        pose_array.header = msg.header
        cv2.imshow("img", img)
        cv2.waitKey(1)
        return pose_array

def main(args=None):
    rclpy.init(args=args)
    node = OutlierRemover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
