import cv2
import message_filters  
import numpy as np
import rclpy
import sys
import torch

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge


class YoloDetector(Node):
    """ROS node to detect pedestrian using YOLOv5."""

    def __init__(self):
        super().__init__("yolo_detector_node")
        self._read_params()

        self.br = CvBridge()

        # init YOLOv5
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5m')
        
    def _init(self):
        # init pubs
        self.dets_pub = self.create_lifecycle_publisher(
            PoseArray, "/yolo_detections", 10
        )
        self.rviz_yolo_pub = self.create_lifecycle_publisher(
            Marker, "/yolo_detection_markers", 10
        )
        
        # init subs
        self.img_sub = message_filters.Subscriber(self, Image, "/cer/realsense_repeater/color_image")
        self.depth_sub = message_filters.Subscriber(self, Image, "/cer/realsense_repeater/depth_image")
        self.tss = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.depth_sub], 1, slop=.1)
        self.tss.registerCallback(self.combined_callback)
                
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._init()
        self.get_logger().info('on_configure() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # The different behaviour of publishers will be
        # defined in _scan_callback
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.dets_pub)
        self.destroy_publisher(self.rviz_yolo_pub)
        self.destroy_subscription(self.img_sub)
        self.destroy_subscription(self.depth_sub)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.dets_pub)
        self.destroy_publisher(self.rviz_yolo_pub)
        self.destroy_subscription(self.img_sub)
        self.destroy_subscription(self.depth_sub)
        
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS
    
    def _read_params(self):
        self.declare_parameter("fov", 74)
        self.declare_parameter("img_width", 640.0)

        self.fov = int(self.get_parameter("fov").value)
        self.img_width= float(self.get_parameter("img_width").value)
	
    def combined_callback(self, img_msg, depth_msg):
        if (self.dets_pub is None or not self.dets_pub.is_activated
            or self.rviz_yolo_pub is None or not self.rviz_yolo_pub.is_activated
        ):
            self.get_logger().info('Lifecycle publisher is deactivated. Messages are not published.')
            return
        if (
            self.dets_pub.get_subscription_count() == 0
            and self.rviz_yolo_pub.get_subscription_count() == 0
        ):
            return
           
        current_frame = self.br.imgmsg_to_cv2(img_msg)
        processed_image = self.model(current_frame)       
	
        dets_xy, dets_cls = [], []
        for det in processed_image.xyxy[0].cpu().numpy():
            if det[5] == 0: #if class=person
                start_angle = (det[0] / self.img_width) * self.fov
                end_angle = (det[2] / self.img_width) * self.fov
                angle = ( (det[0] + (det[2] - det[0]) / 2) / self.img_width) * self.fov	  
                angle = np.deg2rad(-angle + self.fov / 2)
                d_z = 3.0
		
                # compute avg depth of detection
                det_center_y = int(det[1] + (det[3]-det[1])/2)
                det_center_x = int(det[0] + (det[2]-det[0])/2) 
                depth_patch = self.br.imgmsg_to_cv2(
		                    depth_msg, 
				    desired_encoding='passthrough'
				    )[det_center_y-10:det_center_y+10, det_center_x-10:det_center_x+10]
                d_z = np.mean(depth_patch) + 0.2                 
                det = [ -d_z * np.cos(angle), -d_z * np.sin(angle), 0]
                dets_xy.append(det)
                dets_cls.append(0.9) # add confidence
                                         
        # convert to ros msg and publish
        dets_msg = detections_to_pose_array(np.array(dets_xy), np.array(dets_cls))
        dets_msg.header = depth_msg.header
        dets_msg.header.frame_id = "mobile_base_double_lidar"
        self.dets_pub.publish(dets_msg)

        # publish rviz marker
        rviz_msg = detections_to_rviz_marker(np.array(dets_xy), np.array(dets_cls), (0.0, 1.0, 0.0, 1.0))
        rviz_msg.header = depth_msg.header
        rviz_msg.header.frame_id = "mobile_base_double_lidar"
        self.rviz_yolo_pub.publish(rviz_msg)
        
def detections_to_pose_array(dets_xy, dets_cls):
    pose_array = PoseArray()
    for d_xy, d_cls in zip(dets_xy, dets_cls):
        d_xy = -d_xy
        # Detector uses following frame convention:
        # x forward, y rightward, z downward, phi is angle w.r.t. x-axis
        p = Pose()         
        p.position.x = float(d_xy[0])
        p.position.y = float(d_xy[1])
        p.position.z = 0.0
        pose_array.poses.append(p)

    return pose_array
    
def detections_to_rviz_marker(dets_xy, dets_cls, color = (1.0, 0.0, 0.0, 1.0)):
    msg = Marker()
    msg.action = Marker.ADD
    msg.ns = "yolo_detector_node"
    msg.id = 1
    msg.type = Marker.LINE_LIST
    #msg.header.frame_id = "mobile_base_double_lidar"
    # set quaternion so that RViz does not give warning
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    msg.scale.x = 0.03  # line width
    msg.color.r = color[0]
    msg.color.g = color[1]
    msg.color.b = color[2]    
    msg.color.a = color[3]

    # circle
    r = 0.4
    ang = np.linspace(0, 2 * np.pi, 20)
    xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

    # to msg
    for d_xy, d_cls in zip(dets_xy, dets_cls):
        d_xy = -d_xy
        for i in range(len(xy_offsets) - 1):
            # start point of a segment
            p0 = Point()
            p0.x = d_xy[0] + xy_offsets[i, 0]
            p0.y = d_xy[1] + xy_offsets[i, 1]
            p0.z = 0.0
            msg.points.append(p0)

            # end point
            p1 = Point()
            p1.x = d_xy[0] + xy_offsets[i + 1, 0]
            p1.y = d_xy[1] + xy_offsets[i + 1, 1]
            p1.z = 0.0
            msg.points.append(p1)

    return msg
    	
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = YoloDetector()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.destroy_node()
        
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
