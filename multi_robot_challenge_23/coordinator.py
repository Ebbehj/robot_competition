import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA, Int32, String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker

from multi_robot_challenge_23.coverage_map import (
    find_wall_cells, find_reachable, find_viewpoints, find_intrestpoints,
    find_peaks, assign_region_ids, remove_near_spawn, FACTOR, ROBOT_CLEARANCE,
)
from multi_robot_challenge_23.graph_builder import build_graph

WALL_CHECK_DISTANCE = 2.0   # Metres behind the robot

# Robot index -> region
REGIONS_OPEN = {0: 5, 1: 6, 2: 4, 3: 2, 4: 3, 5: 1}   # No wall behind robot 2 or 5
REGIONS_WALL = {0: 4, 1: 5, 2: 6, 3: 1, 4: 2, 5: 3}   # Wall behind robot 2 or 5

# Stor brann (marker 4)
FIRE_DELAY = 0.0    # Sekunder fra brannen er funnet til robotene sendes
FIRE_RADIUS = 1.5    # Meter. Scoring krever at begge er innenfor 2.0
# Robotene scoring-noden følger med på. Originalen følger bare tb3_0 til tb3_4.
SCORED_ROBOTS = {'tb3_0', 'tb3_1', 'tb3_2', 'tb3_3', 'tb3_4', 'tb3_5', 'tb3_6'}

# Farge per region i RViz.
REGION_COLORS = {
    1: (1.0, 0.0, 0.0),  # Rød
    2: (1.0, 1.0, 1.0),  # Hvit
    3: (0.0, 0.0, 1.0),  # Blå
    4: (1.0, 1.0, 0.0),  # Gul
    5: (1.0, 0.0, 1.0),  # Lilla
    6: (0.0, 1.0, 1.0),  # Cyan
}


class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')

        self.wall_cells = None
        self.seen = None
        self.map_msg = None
        self.dist = None
        self.reachable = None
        self.viewpoints = []
        self.peaks = []
        self.peaks_with_ids = []

        # Tom graf frem til kartet er behandlet.
        self.graph = {}
        self.positions = {}

        # Stor brann
        self.fire_pos = None     # (x, y) når brannen er funnet
        self.fire_time = None    # Tidspunktet den ble funnet
        self.fire_team = None    # (hjelper, standby) når de er sendt
        self.fire_done = False

        # Settes fra launch-filen.
        self.declare_parameter('start_xs', [0.0, 0.0])
        self.declare_parameter('start_ys', [-1.0, 1.0])
        self.declare_parameter('robot_names', ['tb3_0', 'tb3_1'])
        self.declare_parameter('reserve_robot', 'tb3_6')
        self.declare_parameter('peak_radius', 2.5)   # Meter
        self.robot_names = self.get_parameter('robot_names').value
        self.reserve_robot = self.get_parameter('reserve_robot').value

        # Transient local: noder som starter senere får også siste melding.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(OccupancyGrid, '/map', self.clbk_map, map_qos)
        self.create_subscription(Point, '/big_fire_found', self.clbk_big_fire, 10)

        # Graf og peaks til alle planleggerne.
        self.graph_pub = self.create_publisher(String, 'navigation_graph', map_qos)

        # Én region og ett mål-topic per robot.
        self.region_pubs = {
            name: self.create_publisher(Int32, f'/{name}/assigned_region', map_qos)
            for name in self.robot_names
        }
        self.goal_pubs = {
            name: self.create_publisher(String, f'/{name}/priority_goal', 10)
            for name in self.robot_names
        }

        # TF gir robotenes posisjon i kartet.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Visualisering i RViz.
        self.marker_pub = self.create_publisher(Marker, 'viewpoint_markers', 10)
        self.edge_pub = self.create_publisher(Marker, 'graph_edges', 10)
        self.dist_map_pub = self.create_publisher(OccupancyGrid, 'wall_distance_map', map_qos)
        self.dist_marker_pub = self.create_publisher(Marker, 'wall_distance_markers', map_qos)

        # Kommenter ut publish_viewpoints for å lettere se inndelingen av peaks.
        self.create_timer(1.0, self.publish_viewpoints)
        self.create_timer(1.0, self.publish_edges)
        self.create_timer(1.0, self.publish_dist_markers)
        self.create_timer(1.0, self.check_fire_team)

    # ------------------------------------------------------------------
    # Kart
    # ------------------------------------------------------------------
    def clbk_map(self, msg):
        # Kartet er statisk, så det behandles bare én gang.
        if self.wall_cells is not None:
            return

        self.wall_cells, free = find_wall_cells(msg)
        self.seen = np.zeros_like(self.wall_cells)

        dist = find_intrestpoints(msg)
        self.map_msg = msg
        self.dist = dist
        self.publish_dist_map(msg, dist)

        starts = list(zip(
            self.get_parameter('start_xs').value,
            self.get_parameter('start_ys').value,
        ))
        self.get_logger().info(f'Starts: {starts}')
        self.reachable = find_reachable(msg, free, starts)

        cell_size = msg.info.resolution * FACTOR
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        # Peaks: beregnes én gang, og de samme brukes av planleggerne og RViz.
        radius_m = self.get_parameter('peak_radius').value
        peaks = find_peaks(dist, radius_m / cell_size)
        peaks = remove_near_spawn(msg, peaks)
        self.peaks = [(r, c) for r, c in peaks if self.reachable[r, c]]
        self.peaks_with_ids = assign_region_ids(self.peaks, dist.shape)

        # find_viewpoints returnerer bare punkter roboten kan nå.
        self.viewpoints = find_viewpoints(msg, free, starts)
        self.graph, self.positions = build_graph(msg, self.viewpoints)

        # Peaks i kartkoordinater.
        peaks_data = [
            {
                'x': float(ox + (c + 0.5) * cell_size),
                'y': float(oy + (r + 0.5) * cell_size),
                'region_id': int(region_id),
            }
            for r, c, region_id in self.peaks_with_ids
        ]

        # Send graf og peaks til planleggerne.
        graph_msg = String()
        graph_msg.data = json.dumps({
            'frame_id': 'map',
            'clearance': ROBOT_CLEARANCE,
            'graph': self.graph,
            'positions': self.positions,
            'peaks': peaks_data,
        })
        self.graph_pub.publish(graph_msg)

        self.assign_regions(starts)

        # Hver forbindelse er lagret begge veier, så vi deler på to.
        connection_count = sum(len(n) for n in self.graph.values()) // 2
        isolated_count = sum(1 for n in self.graph.values() if not n)

        self.get_logger().info(
            f'Graf: {len(self.graph)} noder, {connection_count} forbindelser, '
            f'{isolated_count} isolerte noder.'
        )
        self.get_logger().info(
            f'{int(self.wall_cells.sum())} wall cells, {len(self.viewpoints)} viewpoints, '
            f'{len(peaks_data)} peaks'
        )

    # ------------------------------------------------------------------
    # Regioner
    # ------------------------------------------------------------------
    def wall_behind(self, x, y):
        """True if there is a wall within WALL_CHECK_DISTANCE in the -x direction."""
        info = self.map_msg.info
        cell_size = info.resolution * FACTOR
        row = int((y - info.origin.position.y) / cell_size)
        col = int((x - info.origin.position.x) / cell_size)
        first = max(col - int(WALL_CHECK_DISTANCE / cell_size), 0)

        # Three rows wide, so a wall isn't missed through a one-cell gap
        return bool(self.wall_cells[max(row - 1, 0):row + 2, first:col].any())

    def assign_regions(self, starts):
        blocked = any(
            self.wall_behind(*starts[i]) for i in (2, 5) if i < len(starts)
        )
        mapping = REGIONS_WALL if blocked else REGIONS_OPEN
        self.get_logger().info(f'Vegg bak robot 2 eller 5: {blocked}')

        # Roboter uten plass i tabellen, som standby-roboten, får ingen region.
        for i, name in enumerate(self.robot_names):
            if i not in mapping:
                continue
            msg = Int32()
            msg.data = mapping[i]
            self.region_pubs[name].publish(msg)
            self.get_logger().info(f'{name} -> region {mapping[i]}')

    def send_goal(self, robot_name, node_id):
        """Ber én robot kjøre til en bestemt grafnode, eller 'resume'."""
        msg = String()
        msg.data = node_id
        self.goal_pubs[robot_name].publish(msg)

    # ------------------------------------------------------------------
    # Stor brann
    # ------------------------------------------------------------------
    def robot_position(self, name):
        """Robotens (x, y) i kartet, eller None hvis posisjonen mangler."""
        try:
            t = self.tf_buffer.lookup_transform('map', f'{name}/base_footprint', Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def nodes_near(self, x, y, count=2, spacing=0.6):
        """De nærmeste grafnodene, med nok avstand til at to roboter får plass."""
        def dist(node):
            return math.hypot(self.positions[node][0] - x, self.positions[node][1] - y)

        chosen = []
        for node in sorted(self.positions, key=dist):
            if not self.graph[node]:
                continue   # Isolert node
            px, py = self.positions[node]
            if all(math.hypot(px - self.positions[c][0], py - self.positions[c][1]) >= spacing
                   for c in chosen):
                chosen.append(node)
            if len(chosen) == count:
                break
        return chosen

    def clbk_big_fire(self, msg):
        # Kalles når en robot har fått marker 4 godkjent.
        if self.fire_pos is not None or not self.graph:
            return

        self.fire_pos = (msg.x, msg.y)
        self.fire_time = time.time()
        self.get_logger().info(f'Stor brann funnet. Sender roboter om {FIRE_DELAY:.0f} s.')

    def dispatch_fire_team(self):
        fire_x, fire_y = self.fire_pos

        # Nærmeste søkerobot akkurat nå, blant dem scoring-noden følger.
        distances = {}
        for name in self.robot_names:
            if name == self.reserve_robot or name not in SCORED_ROBOTS:
                continue
            pos = self.robot_position(name)
            if pos is not None:
                distances[name] = math.hypot(pos[0] - fire_x, pos[1] - fire_y)

        nodes = self.nodes_near(fire_x, fire_y)
        if not distances or len(nodes) < 2:
            self.get_logger().warning('Fant ikke robot eller noder til brannen.')
            return

        helper = min(distances, key=distances.get)
        self.send_goal(helper, nodes[0])
        self.send_goal(self.reserve_robot, nodes[1])
        self.fire_team = (helper, self.reserve_robot)
        self.get_logger().info(f'Stor brann: {helper} og {self.reserve_robot} sendes dit.')

    def check_fire_team(self):
        if self.fire_pos is None or self.fire_done:
            return

        # Venter på forsinkelsen, og sender deretter laget.
        if self.fire_team is None:
            if time.time() - self.fire_time >= FIRE_DELAY:
                self.dispatch_fire_team()
            return

        # Laget er på vei: vent til begge er ved brannen.
        for name in self.fire_team:
            pos = self.robot_position(name)
            if pos is None or math.hypot(pos[0] - self.fire_pos[0],
                                         pos[1] - self.fire_pos[1]) > FIRE_RADIUS:
                return

        # Hjelperen fortsetter i regionen sin. Standby-roboten blir stående.
        self.send_goal(self.fire_team[0], 'resume')
        self.fire_team = None
        self.fire_done = True
        self.get_logger().info('Begge er ved brannen.')

    # ------------------------------------------------------------------
    # Visualisering
    # ------------------------------------------------------------------
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
        if not self.peaks_with_ids:
            return

        info = self.map_msg.info
        cell_size = info.resolution * FACTOR
        ox = info.origin.position.x
        oy = info.origin.position.y

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

        # Samme peaks og regioner som planleggerne har fått.
        for r, c, region_id in self.peaks_with_ids:
            p = Point()
            p.x = float(ox + (c + 0.5) * cell_size)
            p.y = float(oy + (r + 0.5) * cell_size)
            p.z = 0.05
            marker.points.append(p)

            red, green, blue = REGION_COLORS.get(region_id, (0.5, 0.5, 0.5))
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
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'graph_edges'
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
                # Forbindelsen finnes begge veier. Tegn den bare én gang.
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