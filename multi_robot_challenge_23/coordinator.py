import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from multi_robot_challenge_23.coverage_map import find_wall_cells, find_viewpoints


class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        self.wall_cells = None
        self.seen = None
        self.viewpoints = []

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self.clbk_map, map_qos)
        self.marker_pub = self.create_publisher(Marker, 'viewpoint_markers', 10)
        self.create_timer(1.0, self.publish_viewpoints)
 

    def clbk_map(self, msg):
        if self.wall_cells is not None:    # Only process the map once
            return

        self.wall_cells, free = find_wall_cells(msg)
        self.seen = np.zeros_like(self.wall_cells)
 
        starts = [(0.0, -1.0), (0.0, -1.0)]
        self.viewpoints = find_viewpoints(msg, free, starts)

        self.get_logger().info(
            f'{int(self.wall_cells.sum())} wall cells, {len(self.viewpoints)} viewpoints'
        )

    def publish_viewpoints(self):
        if not self.viewpoints:
            return

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'viewpoints'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.color.g = 1.0
        marker.color.a = 1.0

        for x, y in self.viewpoints:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            marker.points.append(point)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()