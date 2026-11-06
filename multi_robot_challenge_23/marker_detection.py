import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point


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
                    

        #-----------------------------------------------------------------------------------
        
def main(args=None):
    rclpy.init(args=args)

    marker_detection = MarkerDetection()

    rclpy.spin(marker_detection)

    marker_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()