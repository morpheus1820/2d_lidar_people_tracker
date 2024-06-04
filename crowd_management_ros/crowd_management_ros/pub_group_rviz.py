import rclpy
import sys
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, Point, Point32, Polygon, PolygonStamped
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import ConvexHull


class GroupPublisher(Node):
    def __init__(self):
        super().__init__("group_publisher_node")
        
        # subscribers
        self.is_followed_sub = self.create_subscription(
            Bool, "is_followed_timeout", self.is_followed_callback, 10
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self.amcl_callback, 10
        )
        self.inliers_sub = self.create_subscription(
            PoseArray, "tracks_pose_array", self.tracks_callback, 10
        )
        self.group_pub = self.create_publisher(
            Marker, "group_marker", 10
        )
        self.numbers_pub = self.create_publisher(
            MarkerArray, "group_size_marker", 10
        )
        self.poly_pub = self.create_publisher(
            PolygonStamped, "group_polygon", 10
        )        
        self.last_position = None
        self.last_tracks = None

        self.points = []
        self.colors = []
        
        self.text_numbers = []
        self.text_positions = []        
         
    def amcl_callback(self, msg):
        self.last_position = msg.pose.pose.position

    def tracks_callback(self, msg):
        self.last_tracks = msg
        
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

        # publish size of group
        count = 0
        for pose in self.last_tracks.poses:
            if np.abs(pose.position.x) > 0.5 and np.abs(pose.position.x) < 6 and np.abs(pose.position.y) < 3:
                count += 1
        self.text_numbers.append(str(count))
        self.text_positions.append(p)
        marker_array = MarkerArray()
        for i, (text, pos) in enumerate(zip(self.text_numbers, self.text_positions))	:
            marker = Marker()	
            marker.id = i
            marker.header.frame_id = "map"
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.scale.x = marker.scale.y = marker.scale.z = 0.4
            marker.text = text
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.pose.position.x = pos.x
            marker.pose.position.y = pos.y
            marker.pose.position.z = pos.z
            	
            #print(pos.x, pos.y, pos.z)
            marker_array.markers.append(marker)
        self.numbers_pub.publish(marker_array)
            
        # publish polygon       
        if self.last_tracks is not None and len(self.last_tracks.poses) > 2:
            poly = Polygon()
            polys = PolygonStamped()
            points = []
            for pose in self.last_tracks.poses:
                if np.abs(pose.position.x) > 0.5 and np.abs(pose.position.x) < 6 and np.abs(pose.position.y) < 3: 
                    points.append([pose.position.x, pose.position.y])
            
            if len(points) > 2:
                hull = ConvexHull(np.array(points))
                idxs = hull.vertices
                for idx in idxs:
            	    p = Point32()
            	    p.x = points[idx][0]
            	    p.y = points[idx][1]
            	    p.z = 0.0
            	    poly.points.append(p)
            	
                polys.polygon = poly
                polys.header.frame_id = "mobile_base_body_link"
                self.poly_pub.publish(polys) 
                

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
