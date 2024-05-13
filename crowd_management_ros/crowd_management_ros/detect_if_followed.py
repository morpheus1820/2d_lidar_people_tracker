import numpy as np
import rclpy
import sys

from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from scipy import signal
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


class FollowingGroupDetector(Node):
    def __init__(self):
        super().__init__("following_group_detector_node")

        self.inliers = None
        self.odometry = None
        self.turning_step_count = 0
        self.moving_avg_group = None # format: centerx, centery, convariance in robot frame
        self.count = 0
        
        # filtering
        self.is_followed_array = np.zeros(50)
        self.b, self.a = signal.butter(3, 0.05)
        self.zi = signal.lfilter_zi(self.b, self.a)
        self.rot_vel = 0.0

        # subscribers
        self.inliers_sub = self.create_subscription(
            PoseArray, "inliers", self.inliers_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "odometry", self.odom_callback, 10
        )

        #publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, "detected_group_marker", 10
        )
        self.is_followed_pub = self.create_publisher(
            Bool, "is_followed", 10
        )
        self.is_followed_unfiltered_pub = self.create_publisher(
            Bool, "is_followed_unfiltered", 10
        )
        self.is_followed_timeout_pub = self.create_publisher(
            Bool, "is_followed_timeout", 10
        )
        self.is_followed_string_pub = self.create_publisher(
            String, "is_followed_string", 10
        )
        
        # timer
        self.check_following_timer = self.create_timer(1.0, self.check_following_callback)

    def odom_callback(self, msg):
        self.odometry = msg
        
    def inliers_callback(self, msg):
        self.inliers = msg
        
    def check_following_callback(self):
        group_poses = []
        group_center = None
        group_covariance = None
                
        # if robot is turning
        if self.odometry is not None and abs(self.odometry.twist.twist.angular.z) > 0.2:
            self.turning_step_count += 1
        else:
            self.turning_step_count = 0
        if self.turning_step_count > 2:
            is_followed_unfiltered = Bool()
            is_followed_unfiltered.data = True
            self.is_followed_unfiltered_pub.publish(is_followed_unfiltered)
            
            is_followed_string = String()
            is_followed_string.data = "unknown"
            self.is_followed_string_pub.publish(is_followed_string)
         
            return

        # init followed state = False
        is_followed_unfiltered = Bool()
        is_followed_unfiltered.data = False
            
        # if any inliers were received, detect group
        if self.inliers is None:
            time_diff = 1e6
        else:
            time_diff = self.odometry.header.stamp.sec - self.inliers.header.stamp.sec
            
        print(f"{time_diff=}")
        
        if self.inliers is not None and time_diff < 10:
            for pose in self.inliers.poses:
                x = pose.position.x
                y = pose.position.y
                if np.abs(x)  > 0.5 and np.abs(x) < 6 and np.abs(y) < 3:
                    group_poses.append([x,y])
    
            # if group detected
            if len(group_poses) > 1:
                # set followed state = True
                is_followed_unfiltered.data = True
    
                # estimate group position, covariance
                group_center = np.mean(group_poses,axis=0)
                group_covariance = np.max(np.std(group_poses,axis=0))
    
                if group_covariance > 0.6:
                    group_covariance = 0.6
    
                if self.moving_avg_group is not None:
                    self.moving_avg_group[:2] = (self.moving_avg_group[:2] + group_center) / 2
                    self.moving_avg_group[2] = (self.moving_avg_group[2] + group_covariance) / 2
                else:
                    self.moving_avg_group = np.zeros(3)
                    self.moving_avg_group[:2] = group_center
                    self.moving_avg_group[2] = group_covariance
    
                # visualization 
                marker_msg = MarkerArray()
    
                # circle
                r = 0.1
                ang = np.linspace(0, 2 * np.pi, 20)
                xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)
    
                marker = Marker()
                marker.id = 1
                marker.lifetime = Duration(seconds=1.0).to_msg()
                marker.header.stamp = self.inliers.header.stamp
                marker.header.frame_id = "mobile_base_body_link"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = self.moving_avg_group[2] * 6
                marker.scale.y = self.moving_avg_group[2] * 6
                marker.scale.z = self.moving_avg_group[2] * 6
    
                marker.color.a = 0.3
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
    
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = self.moving_avg_group[0]
                marker.pose.position.y = self.moving_avg_group[1]
                marker.pose.position.z = 0.0
                marker_msg.markers.append(marker)
    
                self.marker_pub.publish(marker_msg)

        # publish followed state
        self.is_followed_unfiltered_pub.publish(is_followed_unfiltered)

        # add followed state to window
        self.is_followed_array = np.roll(self.is_followed_array, 1)
        self.is_followed_array[0] = float(is_followed_unfiltered.data)
        
        # filtering
        filtered, _ = signal.lfilter(self.b, self.a, self.is_followed_array, zi=self.zi*self.is_followed_array[0])
        filtered, _ = signal.lfilter(self.b, self.a, filtered, zi=self.zi*self.is_followed_array[0])
        
        # publish filtered followed state
        is_followed = Bool()
        is_followed.data = True if np.mean(filtered) > 0.2 else False
        self.is_followed_pub.publish(is_followed)
        print("mean", np.mean(filtered))
        
        # publish timeout state
        is_followed_timeout = Bool()
        is_followed_timeout.data = True if self.is_followed_array[:20].sum() > 1e-5 else False
        self.is_followed_timeout_pub.publish(is_followed_timeout)
        
        is_followed_string = String()
        is_followed_string.data = "true" if self.is_followed_array[:20].sum() > 1e-5 else "false"
        self.is_followed_string_pub.publish(is_followed_string)
        
def main(args=None):
    rclpy.init(args=args)
    node = FollowingGroupDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
