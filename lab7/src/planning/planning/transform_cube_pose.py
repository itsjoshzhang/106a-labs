import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped 
from tf2_geometry_msgs import do_transform_point

class TransformCubePose(Node):
    def __init__(self):
        super().__init__('transform_cube_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cube_pose_sub = self.create_subscription(
            PointStamped,
            '/cube_pose',
            self.cube_pose_callback,
            10
        )

        # Please ensure this is filled
        self.cube_pose_pub = self.create_publisher(PointStamped, "/cube_pose_base", 10)

        rclpy.spin_once(self, timeout_sec=2)
        self.cube_pose = None

    def cube_pose_callback(self, msg: PointStamped):
        try:
            transformed_pose = self.transform_cube_pose(msg)
            if transformed_pose is not None:
                self.cube_pose_pub.publish(transformed_pose)
                self.get_logger().info(
                    f"Published cube_pose_base: [{transformed_pose.point.x:.3f}, "
                    f"{transformed_pose.point.y:.3f}, {transformed_pose.point.z:.3f}]"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to transform cube pose: {str(e)}")

    def transform_cube_pose(self, msg: PointStamped):
        """ 
        Transform point into base_link frame
        Args: 
            - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
        Returns:
            Point: point in base_link_frame in form [x, y, z]
        """

        transform = self.tf_buffer.lookup_transform(
            'base_link',                    
            msg.header.frame_id,          
            rclpy.time.Time(),              
            timeout=rclpy.duration.Duration(seconds=1.0)
        )   
        point_in_base_link = do_transform_point(msg, transform)
        return point_in_base_link

def main(args=None):
    rclpy.init(args=args)
    node = TransformCubePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
