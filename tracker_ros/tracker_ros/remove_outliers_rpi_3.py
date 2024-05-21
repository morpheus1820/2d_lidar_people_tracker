#!/usr/bin/env python

import datetime
import numpy as np
import rclpy
import sys

from copy import deepcopy
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Empty

from rclpy.node import Node
from rclpy.time import Time


class OutlierRemoverStatic(Node):
    def __init__(self):
        super().__init__("outlier_remover_static_node")

        self.yolo_poses = None
        self.outlier_poses = []
        self.counter = 0
        self.position_threshold = 0.3
        self.out_file = open("occupancy_over_time-" + datetime.datetime.now().strftime("%m-%d-%Y_%H:%M:%S") + ".csv", "w")
        self.write_to_file_period = 30
        self.write_to_file_prev_sec = 0
                
        self.srv = self.create_service(Empty, 'record_background', self.record_callback)
        
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
            PoseArray, "inliers", 10
        )
        
        self.back_sub_inliers_pub = self.create_publisher(
            PoseArray, "inliers_back_sub", 10
        )
        
    def record_callback(self, request, response):
        self.get_logger().info('Incoming request to record background')
        self.outlier_poses = []
        self.counter = 0
	
        return response
        
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
            
            # write to file
            print("Writing to file?")
            ts = Time.from_msg(msg.header.stamp).seconds_nanoseconds()
            if ts[0] - self.write_to_file_prev_sec >= self.write_to_file_period:
                print("yes")
                date_ts = datetime.datetime.utcfromtimestamp(ts[0])            
                self.out_file.write(date_ts.strftime("%m/%d/%Y, %H:%M:%S") + ", " + str(len(poses.poses)) + "\n")
                self.write_to_file_prev_sec = ts[0]
                
    def remove_outliers(self, msg):
        pose_array = PoseArray()
        for pose in msg.poses:
            p = np.array([pose.position.x, pose.position.y])

            # if creating outliers list
            if self.counter < 50:
                self.outlier_poses.append(p)
            else:
                is_outlier = False
                for op in self.outlier_poses:
                    if np.linalg.norm(op-p) < self.position_threshold:
                        is_outlier = True
                        break 
                
                if not is_outlier:
                    pose_array.poses.append(pose)

        self.counter += 1
        
        # convert to ros msg and publish
        pose_array.header = msg.header
        return pose_array

def main(args=None):
    rclpy.init(args=args)
    node = OutlierRemoverStatic()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
