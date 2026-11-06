import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#TODO: Import the message types that you need for your publishers and subscribers here:

class Wall_Follower(Node):
    def __init__(self):
        #TODO: Initialze a ros node
        super().__init__("controller")

        #TODO: Create a subscriber to the /scan topic using as a callback function the existing function self.clbk_laser and queue size of 10
        self.sub = self.create_subscription(LaserScan, 'scan', self.clbk_laser, 10)


        #TODO: Create a publisher to the /cmd_vel topic with a queue size of 10
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)


        #Default values for the lidar variables as a placeholder until the actual sensor values are recieved through from the ros topic
        self.lidar_left1 = 100
        self.lidar_left2 = 100

        self.lidar_right1 = 100
        self.lidar_right2 = 100
        self.found = False
        self.lidar_straight1 = 100
        self.lidar_straight2 = 100
        self.angle = 35
        self.timer1 = 0
        #Creates a timer definition -> during rclpy.spin() the function self.timer_callback() will be executed every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.mode = 0
    #Callback function for the Turtlebots Lidar topic /scan
    def clbk_laser(self, msg):
        
        

        self.lidar_left1 = msg.ranges[90]
        self.lidar_left2 = msg.ranges[90 - self.angle]

        self.lidar_right1 = msg.ranges[270]
        self.lidar_right2 = msg.ranges[270 + self.angle]

        self.lidar_straight1 = msg.ranges[15]
        self.lidar_straight2 = msg.ranges[345]
        

    def timer_callback(self):
        #Creates a message from type Twist
        sensor_sensitivity = 0.8
        turn_sensor_sensetivity = 0.77
        vel_msg = Twist()
        
        distance_margin = 0.01
        #Defines the speed at which the robot will move forward (value between 0 and 1)
        vel_msg.linear.x = 0.3
        #Defines the speed at which the robot will turn around its own axis (value between -1 and 1)
        vel_msg.angular.z = 0.0
        
        distance_right_x = sensor_sensitivity
        
        distance_right_y = self.lidar_right1/ math.cos(math.radians(self.angle))
      #  self.get_logger().info('angle cos "%f"' % math.cos(math.radians(self.angle)))
        self.get_logger().info('mode "%f"' % self.mode)
        self.get_logger().info('lidar_right2 "%f"' % self.lidar_right2 )
        self.get_logger().info('distance_right_y "%f"' % distance_right_y )
        self.get_logger().info('timer = "%f"' % self.timer1 )


      #  self.get_logger().info('range  "%f" - "%f"' % ((distance_right_y - distance_margin),(distance_right_y + distance_margin) ))
       # self.get_logger().info('lidar_right1 "%f"' % self.lidar_right1 )
        # self.get_logger().info('lidar_right2 "%f"' % self.lidar_right2 )

        if(self.lidar_straight1 <= sensor_sensitivity):
            self.found = True

        if(self.mode == 0):
            vel_msg.linear.x = 0.3
            

            if self.lidar_straight1 < sensor_sensitivity and self.found:
                self.mode = 1
            if self.lidar_right2 > distance_right_y + 0.1 and self.found:
                self.mode = 2

        if(self.mode == 1):        
            vel_msg.angular.z = 0.3
            vel_msg.linear.x = 0.0
            if self.lidar_straight1 > turn_sensor_sensetivity  and self.lidar_straight2 > turn_sensor_sensetivity:
                vel_msg.angular.z = 0.0
                self.mode = 0
            
        
        if(self.mode == 2):
           vel_msg.angular.z = -0.4
           vel_msg.linear.x = 0.1
           self.timer1 = self.timer1 +1 
           if self.timer1 == 10:
            self.mode = 0
            self.timer1 = 0

   

        #TODO: Set vel_msg.linear.x and vel_msg.angular.z depending on the values from self.lidar_left_front and self.lidar_right_front
     #   if(self.lidar_left1 < sensor_sensitivity and self.lidar_right_front > sensor_sensitivity):
      #      vel_msg.angular.z = -0.28
       
         
       # elif(self.lidar_right1< sensor_sensitivity and self.lidar_left_front > sensor_sensitivity):
         #   vel_msg.angular.z = 0.28
        #else:
          #  vel_msg.angular.z = 0.0
           # vel_msg.linear.x = 1.0

         

        #TODO: Publish vel_msg using the previously defined publisher
        self.pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)

    controller = Wall_Follower()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()