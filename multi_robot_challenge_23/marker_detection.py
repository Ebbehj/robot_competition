import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point
from scoring_interfaces.srv import SetMarkerPosition

VALID_IDS = range(5)     # Markørene scoring-noden kjenner: 0 til 4
BIG_FIRE_ID = 4


class MarkerDetection(Node):
    """Rapporterer markørene denne roboten ser til scoring-noden.

    Hver markør holdes for seg, så to markører i samme bilde blir begge
    rapportert. En avvist markør prøves igjen neste gang den blir sett.
    """

    def __init__(self):
        super().__init__('RobotLeaderNode')

        self.latest_position = None   # Siste posisjon fra marker_map_pose
        self.sightings = {}           # marker_id -> siste posisjon i kartet
        self.fresh = set()            # Markører sett siden forrige rapport
        self.reported = set()         # Godkjent av scoring
        self.pending = set()          # Venter på svar fra scoring

        # Relative navn: blir f.eks. /tb3_2/marker_id med namespace tb3_2.
        self.create_subscription(Pose, 'marker_map_pose', self.clbk_marker_map_pose, 10)
        self.create_subscription(Int64, 'marker_id', self.clbk_marker_id, 10)

        self.client = self.create_client(SetMarkerPosition, '/set_marker_position')
        self.fire_pub = self.create_publisher(Point, '/big_fire_found', 10)

        self.create_timer(0.5, self.timer_callback)

    def clbk_marker_map_pose(self, msg):
        # marker_pose publiserer posisjonen rett før ID-en.
        self.latest_position = msg.position

    def clbk_marker_id(self, msg):
        marker_id = msg.data
        if marker_id not in VALID_IDS or self.latest_position is None:
            return

        # Koble ID-en til posisjonen som kom rett før, med én gang.
        # Da blandes ikke to markører som er i samme bilde.
        if marker_id not in self.sightings:
            self.get_logger().info(
                f'Ser marker {marker_id} på '
                f'({self.latest_position.x:.2f}, {self.latest_position.y:.2f})'
            )
        self.sightings[marker_id] = self.latest_position
        self.fresh.add(marker_id)

    def timer_callback(self):
        if not self.client.service_is_ready():
            return

        # Rapporter alle markører med en ny observasjon.
        for marker_id in list(self.fresh):
            if marker_id in self.reported:
                self.fresh.discard(marker_id)
            elif marker_id not in self.pending:
                self.fresh.discard(marker_id)
                self.report_marker(marker_id, self.sightings[marker_id])

    def report_marker(self, marker_id, position):
        req = SetMarkerPosition.Request()
        req.marker_id = int(marker_id)
        req.marker_position.x = position.x
        req.marker_position.y = position.y
        # Scoring sammenligner også z, og markørene står på z = 0.
        req.marker_position.z = 0.0

        self.pending.add(marker_id)
        future = self.client.call_async(req)
        future.add_done_callback(
            lambda f, i=marker_id, p=req.marker_position: self.clbk_report(f, i, p))

    def clbk_report(self, future, marker_id, position):
        self.pending.discard(marker_id)

        try:
            accepted = future.result().accepted
        except Exception as error:
            self.get_logger().warn(f'Marker {marker_id}: ingen svar fra scoring ({error})')
            return

        if accepted:
            self.reported.add(marker_id)
            self.get_logger().info(f'Marker {marker_id} accepted')
            if marker_id == BIG_FIRE_ID:
                # Koordinatoren sender to roboter til den store brannen.
                self.fire_pub.publish(position)
        else:
            # Enten feil posisjon, eller så har en annen robot rapportert den.
            self.get_logger().warn(f'Marker {marker_id} not accepted')


def main(args=None):
    rclpy.init(args=args)

    marker_detection = MarkerDetection()

    rclpy.spin(marker_detection)

    marker_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()