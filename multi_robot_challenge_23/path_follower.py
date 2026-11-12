import json
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__("path_follower")

        # Ruten inneholder punkter med feltene id, x og y.
        # Koordinatene skal være uttrykt i kartets "map"-system.
        self.route = []
        self.target_index = 0

        # Innstillinger for styringen.
        self.position_tolerance = 1.0        # Meter fra punktet.
        self.Kp = 1.0                        # Forsterkning av vinkelfeilen.
        self.max_angular_speed = 3.0         # Radianer per sekund.
        self.max_linear_speed = 20.0         # Meter per sekund.
        self.turn_threshold = math.radians(15)
        self.max_pose_age = 0.5              # Simuleringssekunder.
        self.state = "follow_path"

        # Variabler for scan_POI
        self.fase1 = False
        self.checked_direction = False
        self.direction = None

        # Publisher for første robots kjørekommandoer.
        self.cmd_publisher = self.create_publisher(
            Twist,
            "/tb3_0/cmd_vel",
            10,
        )

        # TF gir robotens posisjon og retning i kartet.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Motta ruter fra planleggeren.
        self.route_subscription = self.create_subscription(
            String,
            "planned_path",
            self.route_callback,
            10,
        )


        # Den kaller update_pose 10 ganger per simuleringssekund.
        self.pose_timer = self.create_timer(0.1, self.update_pose)

        self.get_logger().info("Rutefølgeren venter på en rute.")

    def stop_robot(self):
        # Alle hastigheter i en ny Twist er null.
        self.cmd_publisher.publish(Twist())

    def reject_route(self, reason):
        # En ugyldig ny rute skal ikke la den gamle fortsette.
        self.route = []
        self.target_index = 0
        self.stop_robot()
        self.get_logger().warning(reason)

    def route_callback(self, msg):
        # Planleggeren sender JSON-tekst i en String-melding.
        try:
            route = json.loads(msg.data)
        except json.JSONDecodeError:
            self.reject_route("Mottok ugyldig JSON.")
            return

        if not isinstance(route, list):
            self.reject_route("Ruten må være en liste.")
            return

        # Kontroller hele ruten før vi tar den i bruk.
        for point in route:
            if not isinstance(point, dict):
                self.reject_route("Et rutepunkt må være en ordbok.")
                return

            if not all(key in point for key in ("id", "x", "y")):
                self.reject_route("Et rutepunkt mangler id, x eller y.")
                return

            if not isinstance(point["id"], str):
                self.reject_route("Node-ID må være tekst.")
                return

            # Koordinater må være tall, men ikke boolske verdier.
            # NaN og uendelige verdier kan ikke brukes til styring.
            for axis in ("x", "y"):
                value = point[axis]

                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                ):
                    self.reject_route(
                        f"Koordinaten {axis} må være et tall."
                    )
                    return

                try:
                    finite = math.isfinite(value)
                except OverflowError:
                    finite = False

                if not finite:
                    self.reject_route(
                        f"Koordinaten {axis} må være et endelig tall."
                    )
                    return

        # Stopp før den gamle ruten erstattes.
        self.stop_robot()
        self.route = route
        self.target_index = 0

        if not self.route:
            self.get_logger().info("Mottok tom rute. Roboten stoppes.")
            return

        self.get_logger().info("Kjører")

        self.get_logger().info(
            f"Mottok {len(self.route)} rutepunkter."
        )

        for point in self.route:
            self.get_logger().info(
                f"Node {point['id']}: x={point['x']}, y={point['y']}"
            )

    def update_pose(self):
        # Ingen rute å følge: send stopp og avslutt oppdateringen.
        if self.target_index >= len(self.route):
            self.stop_robot()
            return


        try:
            # Time() ber om den nyeste tilgjengelige transformasjonen.
            # Resultatet beskriver roboten i kartets koordinatsystem.
            transform = self.tf_buffer.lookup_transform(
                "map",
                "tb3_0/base_footprint",
                Time(),
            )
        except TransformException:
            # Ikke styr roboten hvis posisjonen ikke kan hentes.
            self.stop_robot()
            return

        # Kontroller at posisjonen er fersk.
        # Noden må bruke simuleringstid, som TF-dataene.
        pose_time = Time.from_msg(transform.header.stamp)
        pose_age = (
            self.get_clock().now() - pose_time
        ).nanoseconds / 1_000_000_000

        # Samme toleranse som i planleggeren.
        if pose_age > self.max_pose_age or pose_age < -0.1:
            self.stop_robot()
            return

        position = transform.transform.translation
        rotation = transform.transform.rotation

        self.robot_x = position.x
        self.robot_y = position.y

        # Konverter quaternion til yaw: retning i kartplanet.
        # Yaw lagres i radianer til styringsberegningene.
        self.robot_yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )

        # self.get_logger().info(
        #     f"Robot: x={self.robot_x:.2f}, "
        #     f"y={self.robot_y:.2f}, "
        #     f"retning={math.degrees(self.robot_yaw):.1f} grader"
        # )

        # Velg punktet vi foreløpig er på vei mot.
        target = self.route[self.target_index]

        dx = target["x"] - self.robot_x
        dy = target["y"] - self.robot_y
        distance = math.hypot(dx, dy)

        if distance <= self.position_tolerance:
            # Stopp ved punktet før vi velger det neste.
            self.stop_robot()
            self.get_logger().info(f"Nådde punkt: {target['id']}")

            self.target_index += 1

            if self.target_index >= len(self.route):
                self.get_logger().info("Hele ruten er fullført.")
                self.state = "scan_POI"

            # Neste timerkall behandler neste punkt.
            return

        # Beregn retningen fra roboten til målpunktet.
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.robot_yaw

        # Normaliser til [-pi, pi] for å velge korteste sving.
        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error),
        )

        # Lag en bevegelsesmelding, som i veggfølgeren.
        move_msg = Twist()

        if self.state is "follow_path":
            # P-regulator for retning.
            # Positiv vinkelfeil gir venstresving.
            angular_speed = self.Kp * angle_error

            move_msg.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, angular_speed),
            )

            if abs(angle_error) > self.turn_threshold:
                # Snu på stedet til roboten peker nær nok mot målet.
                move_msg.linear.x = 0.0
            else:
                # Begrens farten og reduser den nær målpunktet.
                move_msg.linear.x = min(
                    self.max_linear_speed,
                    0.5 * distance,
                )

            # self.get_logger().info(
            #     f"Neste punkt: {target['id']}, "
            #     f"avstand: {distance:.2f} m, "
            #     f"vinkelfeil: {math.degrees(angle_error):.1f} grader"
            # )

            # # Vis kommandoen før den publiseres til roboten.
            # self.get_logger().info(
            #     f"Beregnet kommando: fremover={move_msg.linear.x:.2f} m/s, "
            #     f"sving={move_msg.angular.z:.2f} rad/s"
            # )

            # Send beregnet fart og svinging til roboten.

        else:

            # dersom roboten allerede ser i riktig retning skal se rundt seg 
            if angle_error < self.turn_threshold:
                move_msg.angular.z = 2.0


            # Fase 1, snu med korteste vei til roboten peker 90 grader fra ny heading
            if self.fase1 is False:
                
                # finner ut om roboten skal snu til høyre eller venstre for å nå ny vinkel
                if self.checked_direction is False:

                    # turn away from heading if robot is closer than 90 degrees, and towards if not
                    if angle_error < math.pi / 2:
                        self.direction = -1
                    else: 
                        self.direction = 1
                    self.checked_direction = True
                    
                angle_error = angle_error - math.pi / 2 # ny vinkel 90 grader nærmere enn originalvinkel

                angular_speed = abs(angle_error)

                move_msg.angular.z = self.direction * angular_speed

                if angle_error < self.turn_threshold - math.radians(10):
                    self.get_logger().info("ser 90 grader fra mål")
                    self.fase1 = True
                    self.direction = None
                    self.checked_direction = False

            # Fase 2, snu motsatt vei til roboten peker mot ny heading
            else:

                # dersom ny retning er nærmest fra høyre, snu til venstre, og motsatt
                if self.checked_direction is False:
                    if angle_error > 0:
                        self.direction = -1
                    else:
                        self.direction = 1
                    self.checked_direction = True

                angular_speed = abs(angle_error)

                move_msg.angular.z = self.direction * angular_speed
                
                if abs(angle_error) < self.turn_threshold:
                    self.get_logger().info("ser mot mål")
                    self.fase1 = False
                    self.direction = None
                    self.checked_direction = False
                    self.state = "follow_path"

                


        self.cmd_publisher.publish(move_msg)


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
