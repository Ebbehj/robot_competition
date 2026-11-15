import json
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value, limit):
    """Begrens value til intervallet [-limit, limit]."""
    return max(-limit, min(limit, value))


def sign(value):
    return 1.0 if value >= 0 else -1.0


def normalize_angle(angle):
    """Normaliser til [-pi, pi] for å velge korteste sving."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__("path_follower")

        # Robotens navn fra namespace, f.eks. "/tb3_2" -> "tb3_2".
        self.robot_name = self.get_namespace().strip("/")
        self.base_frame = f"{self.robot_name}/base_footprint"

        # Ruten inneholder punkter med id, x og y i "map"-systemet.
        self.route = []
        self.target_index = 0
        self.state = "follow_path"
        self.scan_phase = 1

        # Innstillinger for styringen.
        self.position_tolerance = 1.0              # Meter, mellompunkter.
        self.last_node_tolerance = 0.1             # Meter, siste punkt.
        self.Kp = 1.0                              # Forsterkning av vinkelfeilen.
        self.max_angular_speed = 3.0               # rad/s
        self.max_linear_speed = 20              # m/s (TurtleBot3 maks ~0.26)
        self.max_scan_speed = 1.5                  # rad/s under scan_POI
        self.turn_threshold = math.radians(15)
        self.scan_threshold = math.radians(5)
        self.max_pose_age = 0.5                    # Simuleringssekunder.
        self.lookahead_points = 5

        # Relative navn: blir f.eks. /tb3_2/cmd_vel med namespace tb3_2.
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.done_publisher = self.create_publisher(Empty, "route_completed", 10)
        self.create_subscription(String, "planned_path", self.route_callback, 10)

        # TF gir robotens posisjon og retning i kartet.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Kjøres 10 ganger per simuleringssekund.
        self.create_timer(0.1, self.update)

        self.get_logger().info(f"Rutefølger for {self.robot_name} venter på en rute.")

    def stop_robot(self):
        self.cmd_publisher.publish(Twist())

    # ------------------------------------------------------------------
    # Ruter
    # ------------------------------------------------------------------
    def route_callback(self, msg):
        # Planleggeren sender JSON-tekst i en String-melding.
        try:
            route = json.loads(msg.data)
            self.validate_route(route)
        except (json.JSONDecodeError, ValueError, OverflowError) as error:
            # En ugyldig ny rute skal ikke la den gamle fortsette.
            self.route = []
            self.target_index = 0
            self.stop_robot()
            self.get_logger().warning(f"Ugyldig rute: {error}")
            return

        self.stop_robot()
        self.route = route
        self.target_index = 0

        if not route:
            self.get_logger().info("Mottok tom rute. Roboten stoppes.")
            return

        self.get_logger().info(f"Mottok {len(route)} rutepunkter.")

    @staticmethod
    def validate_route(route):
        if not isinstance(route, list):
            raise ValueError("ruten må være en liste")

        for point in route:
            if not isinstance(point, dict) or not all(k in point for k in ("id", "x", "y")):
                raise ValueError("hvert punkt må ha id, x og y")
            if not isinstance(point["id"], str):
                raise ValueError("node-ID må være tekst")

            for axis in ("x", "y"):
                value = point[axis]
                # Tall, men ikke bool, NaN eller uendelig.
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                ):
                    raise ValueError(f"{axis} må være et endelig tall")

    # ------------------------------------------------------------------
    # Posisjon
    # ------------------------------------------------------------------
    def get_pose(self):
        """Returnerer (x, y, yaw) i map, eller None hvis posisjonen mangler eller er gammel."""
        try:
            transform = self.tf_buffer.lookup_transform("map", self.base_frame, Time())
        except TransformException:
            return None

        # Noden må bruke simuleringstid, som TF-dataene.
        age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
        if age > self.max_pose_age or age < -0.1:
            return None

        p = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )
        return p.x, p.y, yaw

    # ------------------------------------------------------------------
    # Hovedløkke
    # ------------------------------------------------------------------
    def update(self):
        # Ingen rute å følge.
        if self.target_index >= len(self.route):
            self.stop_robot()
            return

        pose = self.get_pose()
        if pose is None:
            # Ikke styr roboten uten fersk posisjon.
            self.stop_robot()
            return
        x, y, yaw = pose

        self.advance_targets(x, y)

        # Hele ruten er fullført: stopp, gi beskjed én gang og scan ved neste rute.
        if self.target_index >= len(self.route):
            self.stop_robot()
            self.get_logger().info("Hele ruten er fullført.")
            self.state = "scan_POI"
            self.scan_phase = 1
            self.done_publisher.publish(Empty())
            return

        angle_error, distance = self.aim(x, y, yaw)

        if self.state == "follow_path":
            cmd = self.follow(angle_error, distance)
        else:
            cmd = self.scan(angle_error)

        self.cmd_publisher.publish(cmd)

    def advance_targets(self, x, y):
        """Hopp over rutepunkter roboten allerede er nær nok."""
        while self.target_index < len(self.route):
            target = self.route[self.target_index]
            distance = math.hypot(target["x"] - x, target["y"] - y)

            # Strengere toleranse for siste punkt.
            is_last = self.target_index == len(self.route) - 1
            tolerance = self.last_node_tolerance if is_last else self.position_tolerance

            if distance > tolerance:
                break

            self.get_logger().info(f"Nådde punkt: {target['id']}")
            self.target_index += 1

    def aim(self, x, y, yaw):
        """Vinkelfeil og avstand til gjennomsnittet av de neste punktene."""
        points = self.route[self.target_index:self.target_index + self.lookahead_points]
        avg_x = sum(p["x"] for p in points) / len(points)
        avg_y = sum(p["y"] for p in points) / len(points)

        dx = avg_x - x
        dy = avg_y - y
        angle_error = normalize_angle(math.atan2(dy, dx) - yaw)
        return angle_error, math.hypot(dx, dy)

    def follow(self, angle_error, distance):
        """P-regulator: positiv vinkelfeil gir venstresving."""
        cmd = Twist()
        cmd.angular.z = clamp(self.Kp * angle_error, self.max_angular_speed)
        # Kjør saktere jo mer feil retning roboten peker.
        cmd.linear.x = (
            min(self.max_linear_speed, 0.5 * distance)
            * max(0.0, math.cos(angle_error))
        )
        return cmd

    def scan(self, angle_error):
        """Snur 90 grader bort fra ny retning, og deretter tilbake mot den."""
        cmd = Twist()

        if self.scan_phase == 1:
            # Negativ: nærmere enn 90 grader, snu bort. Positiv: snu mot.
            error = abs(angle_error) - math.pi / 2
            cmd.angular.z = sign(angle_error) * clamp(error, self.max_scan_speed)

            if abs(error) < self.scan_threshold:
                self.get_logger().info("Ser 90 grader fra mål.")
                self.scan_phase = 2
        else:
            # Snu korteste vei mot ny retning.
            cmd.angular.z = clamp(angle_error, self.max_scan_speed)

            if abs(angle_error) < self.turn_threshold:
                self.get_logger().info("Ser mot mål.")
                self.state = "follow_path"
                self.scan_phase = 1

        return cmd


def main(args=None):
    # Brukes både av ros2 run og launch-filen.
    rclpy.init(args=args)
    node = PathFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Forsøk å sende stopp hvis ROS fortsatt er aktivt.
        try:
            if rclpy.ok():
                node.stop_robot()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()