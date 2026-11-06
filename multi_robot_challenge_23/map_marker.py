import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class MapMarkerClass(Node):
    def __init__(self):
        super().__init__('MapMarkerNode')

        self.pub_marker = self.create_publisher(Marker, 'marker_visual', 2)

        self.marker_msg = Marker()
        #Defines the transformation frame with which the following data is associated
        self.marker_msg.header.frame_id = "/map"
        #Defines the current time in ros time
        self.marker_msg.header.stamp = self.get_clock().now().to_msg()
        #Assign a unique marker id
        self.marker_msg.id = 0
        #Define the type of oject that is displayed
        self.marker_msg.type = Marker.POINTS
        #Define the action that is taken
        self.marker_msg.action = Marker.ADD
        #Define part of the orientation of the object displayed in rviz
        self.marker_msg.pose.orientation.w =1.0
        # Defines the size of the marker (in meters) displayed in rviz
        self.marker_msg.scale.x=0.1
        self.marker_msg.scale.y=0.1
        # Define the color (red, green and blue from 0-1) and the opacity (alpha from 0-1)
        self.marker_msg.color.r = 0.0/255.0
        self.marker_msg.color.g = 255.0/255.0
        self.marker_msg.color.b = 0.0/255.0
        self.marker_msg.color.a = 1.0
        #Define how long the object should last before being automatically deleted, where 0 indicates forever
        self.marker_msg.lifetime = rclpy.duration.Duration().to_msg()

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        #Position of point 1
        point_1 = Point()
        point_1.x = 10.0
        point_1.y = 5.0

        #Position of point 2
        point_2 = Point()
        point_2.x = 10.0
        point_2.y = 10.0

        #Add Points to the Marker message
        self.marker_msg.points = [point_1, point_2]
        #Publish Marker message
        self.pub_marker.publish(self.marker_msg)

def main(args=None):
    rclpy.init(args=args)

    map_marker = MapMarkerClass()

    rclpy.spin(map_marker)

    map_marker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()