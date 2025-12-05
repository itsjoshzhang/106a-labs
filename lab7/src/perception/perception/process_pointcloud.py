import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PointStamped
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d


class RealSensePCSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_pc_subscriber')

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/color/image_rect_raw',
            self.depth_callback,
            10
        )

        self.img_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_rect_raw',
            self.image_callback,
            10
        )

        # Timers
        self.create_timer(0.1, self.block_points_callback)

        # State Variables
        self.depth_image = None
        self.rgb_image = None
        self.blocks = []

    def depth_callback(self, msg):
        if not self.depth_image:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.get_logger().info(f"Got depth image of shape {self.depth_image.shape}")
    
    def image_callback(self, msg):
        if not self.rgb_img:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.get_logger().info(f"Got rgb image of shape {self.rgb_image.shape}")

    def block_points_callback(self):
        if self.depth_image and self.rgb_image and not self.blocks:
            # TODO: Call sam3 to get the masks
            masks = ...
            for m in masks:
                masked_depth = m * self.depth_image
                # TODO: Convert to open3d depth image
                o3d_masked_depth = ...
                pcd = o3d.geometry.create_point_cloud_from_depth_image()
                
                

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePCSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    