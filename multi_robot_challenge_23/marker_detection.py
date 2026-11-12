import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point
from scoring_interfaces.srv import SetMarkerPosition

class MarkerDetection(Node):
    def __init__(self):
        super().__init__('RobotLeaderNode')

        #-----------------------------------------------------------------------------------
        #TODO: Create a subscriber to the marker_map_pose and the marker_id topics of either
        #      tb3_0 or tb3_1. As callback functions use clbk_marker_map_pose and clbk_marker_id.
        self.sub = self.create_subscription(Int64, 'marker_id', self.clbk_marker_id, 10)

        self.sub = self.create_subscription(Pose, 'marker_map_pose', self.clbk_marker_map_pose, 10)

        
        #-----------------------------------------------------------------------------------

        # Default values for variables
        self.prev_marker_id = -1
        self.marker_id = -1
        self.marker_position = Point()

        timer_period = 1.0  # seconds
        # Create timer function that gets executed once per second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.client = self.create_client(SetMarkerPosition, '/set_marker_position')
        self.reported = set()
        self.pending = set()
    def clbk_marker_map_pose(self, msg):
        self.marker_position = msg.position

    def clbk_marker_id(self, msg):
        self.marker_id = msg.data


    def timer_callback(self):
        #-----------------------------------------------------------------------------------
        #TODO: Whenever the current marker_id is different than the previous marker id
        #      print out both the marker_id and the marker_position using the self.get_logger().info() function 
         
        
         if self.prev_marker_id != self.marker_id:
            self.get_logger().info('marker_id = "%f"' % self.marker_id)
            self.get_logger().info('x pos =  "%f"' % self.marker_position.x)
            self.get_logger().info('y pos =  "%f"' % self.marker_position.y)
            self.prev_marker_id = self.marker_id

         if (self.marker_id >= 0
                and self.marker_id not in self.reported
                and self.marker_id not in self.pending
                and self.client.service_is_ready()):
            self.report_marker(self.marker_id, self.marker_position)           

        #-----------------------------------------------------------------------------------
    def report_marker(self, marker_id, position):
        req = SetMarkerPosition.Request()
        req.marker_id = int(marker_id)
        req.marker_position.x = position.x
        req.marker_position.y = position.y
        req.marker_position.z = 0.0

        self.pending.add(marker_id)
        future = self.client.call_async(req)
        future.add_done_callback(lambda f, i=marker_id: self.clbk_report(f, i))

    def clbk_report(self, future, marker_id):
        self.pending.discard(marker_id)
        if future.result().accepted:
            self.reported.add(marker_id)
            self.get_logger().info(f'Marker {marker_id} accepted')
        else:
            self.get_logger().warn(f'Marker {marker_id} not accepted')

def main(args=None):
    rclpy.init(args=args)

    marker_detection = MarkerDetection()

    rclpy.spin(marker_detection)

    marker_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()