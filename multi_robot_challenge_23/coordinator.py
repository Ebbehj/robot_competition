import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA   # add to imports

from multi_robot_challenge_23.coverage_map import (
    find_wall_cells, find_reachable, find_viewpoints, find_intrestpoints,find_peaks, FACTOR,   # NEW: FACTOR
)


class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        self.wall_cells = None
        self.seen = None
        self.viewpoints = []
        self.map_msg = None                          # NEW
        self.dist = None 
        self.reachable = None                            # NEW

        self.declare_parameter('peak_radius', 2.5)   # NEW (meters)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self.clbk_map, map_qos)
        self.marker_pub = self.create_publisher(Marker, 'viewpoint_markers', 10)
        self.dist_map_pub = self.create_publisher(OccupancyGrid, 'wall_distance_map', map_qos)
        self.dist_marker_pub = self.create_publisher(Marker, 'wall_distance_markers', map_qos)
        self.create_timer(1.0, self.publish_viewpoints)
        self.create_timer(1.0, self.publish_dist_markers)   # NEW
    def clbk_map(self, msg):
        if self.wall_cells is not None:
            return

        self.wall_cells, free = find_wall_cells(msg)
        self.seen = np.zeros_like(self.wall_cells)

        
        dist = find_intrestpoints(msg)
        self.publish_dist_map(msg, dist)
        self.map_msg = msg
        self.dist = dist

        starts = [(0.0, -1.0), (0.0, -1.0)]
        self.reachable = find_reachable(msg, free, starts)

        cell_size = msg.info.resolution * FACTOR
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        self.viewpoints = [
            (x, y) for x, y in find_viewpoints(msg, free, starts)
            if self.reachable[int((y - oy) / cell_size), int((x - ox) / cell_size)]
        ]

        self.get_logger().info(
            f'{int(self.wall_cells.sum())} wall cells, {len(self.viewpoints)} viewpoints'
        )

    # NEW: whole method
    def publish_dist_map(self, msg, dist):
        max_steps = max(int(dist.max()), 1)
        values = np.where(dist >= 0, dist * 100 // max_steps, -1).astype(np.int8)

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = msg.info.resolution * FACTOR
        grid.info.width = dist.shape[1]
        grid.info.height = dist.shape[0]
        grid.info.origin = msg.info.origin
        grid.data = values.flatten().tolist()

        self.dist_map_pub.publish(grid)

   
    def publish_dist_markers(self):
        if self.dist is None:
            return

        msg, dist = self.map_msg, self.dist
        cell_size = msg.info.resolution * FACTOR
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        max_steps = max(int(dist.max()), 1)

        radius_m = self.get_parameter('peak_radius').value
        peaks = find_peaks(dist, radius_m / cell_size)
        peaks = [(r, c) for r, c in peaks if self.reachable[r, c]]


        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'wall_distance_peaks'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = cell_size * 0.9
        marker.scale.y = cell_size * 0.9
        marker.scale.z = 0.05

        for r, c in peaks:
            p = Point()
            p.x = float(ox + (c + 0.5) * cell_size)
            p.y = float(oy + (r + 0.5) * cell_size)
            p.z = 0.05
            marker.points.append(p)

            t = float(dist[r, c]) / max_steps
            marker.colors.append(ColorRGBA(r=1.0 - t, g=t, b=0.0, a=1.0))

        self.dist_marker_pub.publish(marker)


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