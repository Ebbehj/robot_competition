import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
class ControllerClass(Node):
    def __init__(self):
        super().__init__('RobotLeaderNode')

        #-----------------------------------------------------------------------------------
        #TODO: Create a subscriber to each of the robot_handler publishers
        #      using the existing callback functions (clbk_tb3_0_test and clbk_tb3_1_test)    
        self.sub0 = self.create_subscription(Float64, '/tb3_0/namespace_test', self.clbk_tb3_0_test, 100)
        self.sub1 = self.create_subscription(Float64, '/tb3_1/namespace_test', self.clbk_tb3_1_test,  100)

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                        depth=5,
                        )

        self.create_subscription(OccupancyGrid, '/map', callback=self.clbk_map, qos_profile=qos_profile)

        #-----------------------------------------------------------------------------------

        self.tb3_0_lidar_value = 1000
        self.tb3_1_lidar_value = 1000

        timer_period = 1.0  # seconds
        # Create timer function that gets executed once per second
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def clbk_tb3_0_test(self, msg):
        self.tb3_0_lidar_value = msg.data

    def clbk_tb3_1_test(self, msg):
        self.tb3_1_lidar_value = msg.data

    def timer_callback(self):
        self.get_logger().info('TB3_0 Lidar Value: '+ str(self.tb3_0_lidar_value))
        self.get_logger().info('TB3_1 Lidar Value: '+ str(self.tb3_1_lidar_value))
    
    def get_map_pos(self, map_iter):
        x = int(map_iter/self.map_msg.info.width)
        y = int(map_iter - x*self.map_msg.info.width)

        return [x, y]

    def get_map_iter(self, x, y):
        map_iter = x*self.map_msg_og.info.width + y

        return map_iter

def main(args=None):
    rclpy.init(args=args)

    robot_leader = ControllerClassClass()

    rclpy.spin(robot_leader)

    robot_leader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()