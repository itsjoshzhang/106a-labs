import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import forward_kinematics.forward_kinematics as fk


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        output = fk.ur7e_forward_kinematics_from_joint_state(msg)
        print(output)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()