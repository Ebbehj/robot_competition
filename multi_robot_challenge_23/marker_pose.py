from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64

import tf2_geometry_msgs


class MarkerMapPose(Node):
    def __init__(self):
        super().__init__('MarkerMapPose')

        # Initiate a ROS parameter with a default value and then read the parameters value
        self.declare_parameter('namespace', 'tb3_5')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        self.camera_frame = self.namespace + '/camera_rgb_optical_frame'
        self.map_frame = 'map'

        # Detections that are waiting for their transform to arrive.
        self.pending = deque(maxlen=10)

        self.marker_recognition_sub = self.create_subscription(
            ArucoMarkers, '/' + self.namespace + '/aruco_markers', self.clbk_marker_recognition, 10)
        self.marker_map_pose_pub = self.create_publisher(
            Pose, '/' + self.namespace + '/marker_map_pose', 10)
        self.marker_id_pub = self.create_publisher(
            Int64, '/' + self.namespace + '/marker_id', 10)

        # Transform listener used for looking up transformations from one frame to another
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def clbk_marker_recognition(self, msg):
        # No timer: work is only done when a marker is actually detected.
        self.pending.append(msg)

        # Try every waiting detection. Keep the ones whose transform isn't there yet.
        still_waiting = deque(maxlen=self.pending.maxlen)
        for detection in self.pending:
            if not self.publish_markers(detection):
                still_waiting.append(detection)
        self.pending = still_waiting

    def publish_markers(self, msg):
        """Transform all markers in msg to the map frame and publish them.

        Uses the camera position from the moment the image was taken, so the
        result stays correct while the robot is driving or turning.
        Returns False if the transform for that moment isn't available yet.
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                Time.from_msg(msg.header.stamp),
            )
        except TransformException:
            return False

        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            # Publish the pose first, so it is in place when the ID arrives.
            map_pose = tf2_geometry_msgs.do_transform_pose(pose, trans)
            self.marker_map_pose_pub.publish(map_pose)

            id_msg = Int64()
            id_msg.data = int(marker_id)
            self.marker_id_pub.publish(id_msg)

        return True


def main(args=None):
    rclpy.init(args=args)

    controller = MarkerMapPose()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()