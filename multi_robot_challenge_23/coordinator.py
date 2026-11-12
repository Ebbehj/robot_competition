import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import json
from std_msgs.msg import String
from multi_robot_challenge_23.coverage_map import ROBOT_CLEARANCE

from multi_robot_challenge_23.coverage_map import find_wall_cells, find_viewpoints

from multi_robot_challenge_23.graph_builder import build_graph


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

        # Behold grafen så planleggeren kan motta den også ved senere oppstart.
        self.graph_pub = self.create_publisher(
            String, "navigation_graph", map_qos
        )

        self.create_subscription(OccupancyGrid, '/map', self.clbk_map, map_qos)
        self.marker_pub = self.create_publisher(Marker, 'viewpoint_markers', 10)
        self.create_timer(1.0, self.publish_viewpoints)

        # Tom graf frem til kartet er behandlet.
        self.graph = {}
        self.positions = {}

        # Eget topic for å vise forbindelsene.
        self.edge_pub = self.create_publisher(
            Marker, "graph_edges", 10
        )

        self.create_timer(1.0, self.publish_edges)
 

    def clbk_map(self, msg):
        if self.wall_cells is not None:    # Only process the map once
            return

        self.wall_cells, free = find_wall_cells(msg)
        self.seen = np.zeros_like(self.wall_cells)
 
        starts = [(0.0, -1.0), (0.0, -1.0)]
        self.viewpoints = find_viewpoints(msg, free, starts)

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