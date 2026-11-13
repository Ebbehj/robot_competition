import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA   # add to imports

import json
from std_msgs.msg import String
from multi_robot_challenge_23.coverage_map import ROBOT_CLEARANCE

from multi_robot_challenge_23.coverage_map import (
    find_wall_cells, find_reachable, find_viewpoints, find_intrestpoints,find_peaks, assign_region_ids, FACTOR,   # NEW: FACTOR
)

from multi_robot_challenge_23.graph_builder import build_graph


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

        # Behold grafen så planleggeren kan motta den også ved senere oppstart.
        self.graph_pub = self.create_publisher(
            String, "navigation_graph", map_qos
        )

        self.create_subscription(OccupancyGrid, '/map', self.clbk_map, map_qos)
        self.marker_pub = self.create_publisher(Marker, 'viewpoint_markers', 10)
        self.dist_map_pub = self.create_publisher(OccupancyGrid, 'wall_distance_map', map_qos)
        self.dist_marker_pub = self.create_publisher(Marker, 'wall_distance_markers', map_qos)
        self.create_timer(1.0, self.publish_viewpoints) # Kommenter ut denne for å lettere se inndelingen av POI noder

        # Tom graf frem til kartet er behandlet.
        self.graph = {}
        self.positions = {}

        # Eget topic for å vise forbindelsene.
        self.edge_pub = self.create_publisher(
            Marker, "graph_edges", 10
        )

        self.create_timer(1.0, self.publish_edges)
        self.create_timer(1.0, self.publish_dist_markers)  
  

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

        # Lag grafen fra kartet og de eksisterende observasjonspunktene.
        self.graph, self.positions = build_graph(msg, self.viewpoints)

        # Send forbindelser og koordinater samlet til planleggeren.
        graph_msg = String()
        graph_msg.data = json.dumps({
            "frame_id": "map",
            "clearance": ROBOT_CLEARANCE,
            "graph": self.graph,
            "positions": self.positions,
        })
        self.graph_pub.publish(graph_msg)

        # Hver forbindelse er lagret begge veier, så vi deler på to.
        connection_count = sum(
            len(neighbors) for neighbors in self.graph.values()
        ) // 2

        # En isolert node har ingen godkjente forbindelser.
        isolated_count = sum(
            1 for neighbors in self.graph.values() if not neighbors
        )

        self.get_logger().info(
            f"Graf: {len(self.graph)} noder, "
            f"{connection_count} forbindelser, "
            f"{isolated_count} isolerte noder."
        )

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
        peaks_with_ids = assign_region_ids(peaks, dist.shape)


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

        colors = {
            1: (1.0, 0.0, 0.0),  # Red
            2: (1.0, 1.0, 1.0),  # White
            3: (0.0, 0.0, 1.0),  # Blue
            4: (1.0, 1.0, 0.0),  # Yellow
            5: (1.0, 0.0, 1.0),  # Purple
            6: (0.0, 1.0, 1.0),  # Cyan
        }


        for r, c, id in peaks_with_ids:
            p = Point()
            p.x = float(ox + (c + 0.5) * cell_size)
            p.y = float(oy + (r + 0.5) * cell_size)
            p.z = 0.05
            marker.points.append(p)

            t = float(dist[r, c]) / max_steps

            red, green, blue = colors[id]
            marker.colors.append(ColorRGBA(r=red, g=green, b=blue, a=1.0))

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

    def publish_edges(self):
        if not self.graph:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "graph_edges"
        marker.id = 0

        # Hvert par med punkter beskriver én linje.
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Blå linjer med bredde på 2 cm.
        marker.scale.x = 0.02
        marker.color.b = 1.0
        marker.color.a = 1.0

        for node_id, neighbors in self.graph.items():
            for neighbor_id in neighbors:
                # Forbindelsen finnes begge veier i grafen.
                # Tegn den bare én gang.
                if node_id >= neighbor_id:
                    continue

                for point_id in (node_id, neighbor_id):
                    x, y = self.positions[point_id]

                    point = Point()
                    point.x = float(x)
                    point.y = float(y)
                    point.z = 0.05

                    marker.points.append(point)

        self.edge_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()