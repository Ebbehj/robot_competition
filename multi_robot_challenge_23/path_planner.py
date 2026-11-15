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
from std_msgs.msg import String, Empty
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import String, Empty, Int32
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
        self.peaks = []
        self.assigned_region = None       
        self.started = False
        self.create_timer(0.5, self.try_start_navigation)
        self.last_graph_data = None
        self.peak_index = 0 # Hvilket peak i regionen roboten skal besøke
        self.awaiting_completion = False# Venter vi på at en automatisk planlagt rute skal fullføres?
        
        
        # Motta også siste kart/graf hvis de ble publisert før oppstart.
        saved_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
                # Replaces self.declare_parameter("assigned_region", 1)
        self.assigned_region = None

        # After saved_qos is defined
        self.create_subscription(Int32, "assigned_region", self.region_callback, saved_qos)
        
        self.robot_name = self.get_namespace().strip("/")

        self.path_pub = self.create_publisher(String, "planned_path", 10)
        self.create_subscription(String, "/navigation_graph", self.graph_callback, saved_qos)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, saved_qos)
        self.create_subscription(String, "priority_goal", self.goal_callback, 10)
        self.create_subscription(PointStamped, "/clicked_point", self.clicked_callback, 10)
        self.create_subscription(Empty, "route_completed", self.route_completed_callback, 10)
        self.get_logger().info("Venter på kart og graf. Velg deretter Publish Point i RViz.")


    def region_callback(self, msg):
        self.assigned_region = msg.data
        self.peak_index = 0
        self.started = False
        self.get_logger().info(f"Tildelt region {msg.data}")

    def get_region_peaks(self):
        return [
            peak for peak in self.peaks
            if peak["region_id"] == self.assigned_region
        ]


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
        # Ikke stopp roboten hvis kartet er uendret
        if (
            self.map_msg is not None
            and msg.header.frame_id == self.map_msg.header.frame_id
            and msg.info == self.map_msg.info
            and np.array_equal(msg.data, self.map_msg.data)
        ):
            return

        # Kartet er nytt eller endret
        self.started = False
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
       
        # Ignorer grafen hvis den er identisk med forrige
        if msg.data == self.last_graph_data:
            return

        # Grafen er ny eller endret
        self.last_graph_data = None
        self.started = False

        self.publish_route([])
        self.graph = {}
        self.positions = {}
        self.peaks = []
        try:
            data = json.loads(msg.data)
            peaks = data.get("peaks", [])
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
        self.peaks = peaks
        self.last_graph_data = msg.data
        self.get_logger().info(f"Kartgraf klar: {len(graph)} noder.")

        self.get_logger().info(f"Antall peaks mottatt: {len(self.peaks)}")
        region_peaks = self.get_region_peaks()
        self.get_logger().info(f"Antall peaks i min region: {len(region_peaks)}")

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
        if msg.data == "resume":
            # Continue with the peaks in the region
            self.started = False
            return

        # A priority goal pauses the automatic peak navigation
        self.started = True
        self.awaiting_completion = False
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
            transform = self.tf_buffer.lookup_transform(
                "map", f"{self.robot_name}/base_footprint", Time()
            )        
        except TransformException:
            self.reject("Robotposisjon mangler. Prøv målet igjen om et øyeblikk.")
            return
        
        age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
        # Tillat små tidsavvik mellom TF og simuleringens klokke.
        if age < -0.1 or age > 0.5:
            self.reject(f"TF-alder: {age:.3f} sekunder")
            return False
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
            return True
        self.reject("Ingen farbar tilkobling til en grafdel som når målet. Ingen kjøring.")

    

    def plan_to_peak(self, peak):
        if not self.positions:
            return False

        x = peak["x"]
        y = peak["y"]

        goal = min(
            self.positions,
            key=lambda node: math.hypot(
                self.positions[node][0] - x,
                self.positions[node][1] - y
            )
        )

        self.get_logger().info(
            f"Region {peak['region_id']} -> grafnode {goal}"
        )

        return self.plan_to_goal(goal)

    def try_start_navigation(self):

        # Ikke start en ny rute hvis vi allerede har startet
        if self.started:
            return

        # Vent på kart og graf
        if self.map_msg is None or not self.graph:
            return

        # Hent alle peaks i robotens region
        region_peaks = self.get_region_peaks()

        if not region_peaks:
            return

        # Har roboten besøkt alle peaks?
        if self.peak_index >= len(region_peaks):

            self.get_logger().info(
                "Alle peaks i regionen er besøkt!"
            )

            self.started = True
            return

        # Velg neste peak
        peak = region_peaks[self.peak_index]

        self.get_logger().info(
            f"Planlegger rute til peak "
            f"{self.peak_index + 1}/{len(region_peaks)}: {peak}"
        )

        # Beregn ruten
        success = self.plan_to_peak(peak)

        if success:
            self.started = True
            self.awaiting_completion = True

    def route_completed_callback(self, msg):

        if not self.awaiting_completion:
            return

        self.awaiting_completion = False

        # Gå videre til neste peak
        self.peak_index += 1

        self.get_logger().info(
            f"Rute fullført. Neste peak-indeks: {self.peak_index}"
        )

        # Tillat automatisk planlegging av neste rute
        self.started = False

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
