import rclpy
import sys

from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, Point
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker


class GroupPublisher(Node):
    def __init__(self):
        super().__init__("group_publisher_node")
        
        # subscribers
        self.is_followed_sub = self.create_subscription(
            Bool, "is_followed", self.is_followed_callback, 10
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self.amcl_callback, 10
        )
        self.group_pub = self.create_publisher(
            Marker, "group_marker", 10
        )
        
        self.last_position = None
        self.points = []
        self.colors = []
         
    def amcl_callback(self, msg):
        self.last_position = msg.pose.pose.position

    def is_followed_callback(self, msg):
        if self.last_position is None:
            return
    
        marker = Marker()
        marker.id = 0
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.4
        
        p = Point();
        p.x = self.last_position.x 
        p.y = self.last_position.y
        p.z = 0.0
        self.points.append(p)
        c = ColorRGBA()
        if msg.data == True:
            c.r = 0.0
            c.g = 1.0
        else:
            c.r = 1.0
            c.g = 0.0
        c.b = 0.0
        c.a = 0.1
        
        self.colors.append(c)
        marker.points = self.points
        marker.colors = self.colors
        self.group_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = GroupPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
