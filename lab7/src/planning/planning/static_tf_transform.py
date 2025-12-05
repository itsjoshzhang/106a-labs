#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np

class ConstantTransformPublisher(Node):
    def __init__(self):
        super().__init__('constant_tf_publisher')
        self.br = StaticTransformBroadcaster(self)

        # Homogeneous transform G_ar->base_link
        G = np.array([
            [-1, 0, 0, 0.0],
            [ 0, 0, 1, 0.16],
            [ 0, 1, 0, -0.13],
            [ 0, 0, 0, 1.0]
        ])

        # Create TransformStamped
        self.transform = TransformStamped()
        # ---------------------------
        # TODO: Fill out TransformStamped message
        # --------------------------
        
        self.transform.header.frame_id = 'ar_marker_10'
        self.transform.child_frame_id = 'base_link'

        t = self.transform.transform
        t.translation.x = G[0,3]
        t.translation.y = G[1,3]
        t.translation.z = G[2,3]

        r = R.from_matrix(G[:3, :3])
        t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = r.as_quat()

        self.timer = self.create_timer(0.05, self.broadcast_tf)

    def broadcast_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform)


def main():
    rclpy.init()
    node = ConstantTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
