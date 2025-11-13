import tf2_ros
import sys
import rclpy
from rclpy.node import Node


class TF_Echo(Node):

    def __init__(self, tf, sf):
        super().__init__('tf_echo')
        self.tf = tf
        self.sf = sf

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.timer = self.create_timer(0.1, self.listener_callback)

    def listener_callback(self):
        try:
            trans = self.tfBuffer.lookup_transform(self.tf, self.sf, rclpy.time.Time())
            print(f'Translation: {trans.transform.translation}')
            print(f'Rotation: {trans.transform.rotation}')
        except Exception as e:
            print(e, "retry")


def main(args=None):
    rclpy.init(args=args)
    target_frame = sys.argv[1]
    source_frame = sys.argv[2]

    node = TF_Echo(target_frame, source_frame)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()