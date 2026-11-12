"""Demo: kartgraf + mål fra RViz/ROS -> rute til path_follower.

Kun tb3_0. Kartet forutsettes statisk og banen fri for bevegelige hindringer.
"""
import heapq
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from multi_robot_challenge_23.graph_builder import corridor_is_free


def find_shortest_path(graph, start, goal):
    # Samme Dijkstra-algoritme som vi har testet tidligere.
    if start not in graph or goal not in graph:
        raise ValueError("Start og mål må finnes i grafen.")
    distances = {node: float("inf") for node in graph}
    distances[start] = 0.0
    previous = {}
    queue = [(0.0, start)]

    while queue:
        current_cost, current_node = heapq.heappop(queue)
        if current_cost > distances[current_node]:
            continue
        if current_node == goal:
            path = [goal]
            while path[-1] != start:
                path.append(previous[path[-1]])
            path.reverse()
            return path, current_cost
        for neighbor, edge_cost in graph[current_node].items():
            new_cost = current_cost + edge_cost
            if new_cost < distances[neighbor]:
                distances[neighbor] = new_cost
                previous[neighbor] = current_node
                heapq.heappush(queue, (new_cost, neighbor))
    return None, float("inf")


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner")
        self.graph = {}
        self.positions = {}
        self.map_msg = None
        self.grid = None
        self.clearance = 0.5
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Motta også siste kart/graf hvis de ble publisert før oppstart.
        saved_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(String, "planned_path", 10)
        self.create_subscription(String, "navigation_graph", self.graph_callback, saved_qos)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, saved_qos)
        self.create_subscription(String, "priority_goal", self.goal_callback, 10)
        self.create_subscription(PointStamped, "/clicked_point", self.clicked_callback, 10)
        self.get_logger().info("Venter på kart og graf. Velg deretter Publish Point i RViz.")

    def publish_route(self, path):
        # Samme melding som rutefølgeren allerede leser.
        points = [
            {"id": node, "x": self.positions[node][0], "y": self.positions[node][1]}
            for node in path
        ]
        msg = String()
        msg.data = json.dumps(points)
        self.path_pub.publish(msg)

    def reject(self, reason):
        self.publish_route([])
        self.get_logger().warning(reason)

    def map_callback(self, msg):
        # Avbryt gammel rute ved nytt kart.
        self.publish_route([])
        self.map_msg = None
        self.grid = None
        if msg.header.frame_id != "map":
            self.reject("Kartets koordinatsystem må være map.")
            return
        self.grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_msg = msg
        self.get_logger().info("Kart mottatt.")

    def graph_callback(self, msg):
        # Ikke behold en gammel rute hvis grafen endres eller er ugyldig.
        self.publish_route([])
        self.graph = {}
        self.positions = {}
        try:
            data = json.loads(msg.data)
            graph = data["graph"]
            positions = data["positions"]
            clearance = float(data["clearance"])
            if data["frame_id"] != "map" or not math.isfinite(clearance) or clearance < 0:
                raise ValueError("Feil koordinatsystem eller klaring.")
            if not isinstance(graph, dict) or not isinstance(positions, dict):
                raise ValueError("Graf og posisjoner må være ordbøker.")
            if not graph or set(graph) != set(positions):
                raise ValueError("Graf og posisjoner må ha de samme node-ID-ene.")
            for node, neighbors in graph.items():
                if len(positions[node]) != 2 or not all(math.isfinite(v) for v in positions[node]):
                    raise ValueError("Ugyldige koordinater.")
                for neighbor, cost in neighbors.items():
                    if neighbor not in graph or not math.isfinite(cost) or cost < 0:
                        raise ValueError("Ugyldig forbindelse.")
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as error:
            self.reject(f"Kan ikke bruke grafen: {error}")
            return
        self.graph = graph
        self.positions = positions
        self.clearance = clearance
        self.get_logger().info(f"Kartgraf klar: {len(graph)} noder.")

    def clicked_callback(self, msg):
        # Klikket velger nærmeste eksisterende node, ikke et nytt rutepunkt.
        if not self.graph:
            self.reject("Venter på grafen fra koordinatoren.")
            return
        if msg.header.frame_id != "map":
            self.reject("Sett Fixed Frame til map i RViz.")
            return
        if not math.isfinite(msg.point.x) or not math.isfinite(msg.point.y):
            self.reject("Ugyldig klikkposisjon.")
            return
        goal = min(self.positions, key=lambda node: math.hypot(
            self.positions[node][0] - msg.point.x,
            self.positions[node][1] - msg.point.y,
        ))
        self.get_logger().info(f"Klikket valgte {goal} på {self.positions[goal]}.")
        self.plan_to_goal(goal)

    def goal_callback(self, msg):
        # Fremtidig prioriteringsnode kan fortsatt sende en node-ID hit.
        self.plan_to_goal(msg.data)

    def plan_to_goal(self, goal):
        self.publish_route([])
        if not self.graph or self.map_msg is None:
            self.reject("Kart og graf må være mottatt først. Send målet igjen etterpå.")
            return
        if goal not in self.graph:
            self.reject(f"Ukjent målnode: {goal}")
            return
        try:
            transform = self.tf_buffer.lookup_transform("map", "tb3_0/base_footprint", Time())
        except TransformException:
            self.reject("Robotposisjon mangler. Prøv målet igjen om et øyeblikk.")
            return
        
        age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
        # Tillat små tidsavvik mellom TF og simuleringens klokke.
        if age < -0.1 or age > 0.5:
            self.reject(f"TF-alder: {age:.3f} sekunder")
            return
        position = transform.transform.translation
        robot = (position.x, position.y)
        info = self.map_msg.info

        # Prøv nærmeste startnode først. Den må både være direkte tilgjengelig
        # fra roboten og ha en rute gjennom grafen til målet.
        candidates = sorted(self.positions, key=lambda node: math.hypot(
            self.positions[node][0] - robot[0], self.positions[node][1] - robot[1]
        ))
        for start in candidates:
            if not corridor_is_free(
                self.grid, info.resolution, info.origin.position.x,
                info.origin.position.y, robot, self.positions[start], self.clearance
            ):
                continue
            path, cost = find_shortest_path(self.graph, start, goal)
            if path is None:
                continue
            self.publish_route(path)
            self.get_logger().info(
                f"Rute: {' -> '.join(path)}. Grafkostnad: {cost:.2f} m. "
                "Robotens forbindelse til startnoden er også kartkontrollert."
            )
            return
        self.reject("Ingen farbar tilkobling til en grafdel som når målet. Ingen kjøring.")


def main(args=None):
    # Brukes både av ros2 run og launch-filen.
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_route([])
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
