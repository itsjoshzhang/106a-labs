#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import transforms3d.euler as euler
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, PointStamped
from tf2_geometry_msgs import do_transform_pose
from plannedcntrl.trajectory import plan_curved_trajectory  # Your existing Bezier planner
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # Publisher and TF setup
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Controller gains
        self.Kp = np.diag([0.2, 0.2])
        self.Kd = np.diag([-0.2, 0.2])
        self.Ki = np.diag([0.0, 0.0])

        # Subscriber
        self.create_subscription(PointStamped, '/goal_point', self.planning_callback, 10)

        self.get_logger().info('TurtleBot controller node initialized.')

    # ------------------------------------------------------------------
    # Main waypoint controller
    # ------------------------------------------------------------------
    def controller(self, waypoint):
        prev_err = np.zeros(2)
        integral = np.zeros(2)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            except Exception as e:
                continue
            
            p = PoseStamped()
            p.header.frame_id = 'odom'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = waypoint[0]
            p.pose.position.y = waypoint[1]
            p.pose.position.z = 1.0

            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0

            # see goal from robot pov
            pose_base = do_transform_pose(p, trans)

            x_rel = pose_base.pose.position.x
            y_rel = pose_base.pose.position.y

            dist = math.sqrt(x_rel**2 + y_rel**2)

            yaw_err = math.atan2(y_rel, x_rel)

            #STOP THE ROBOT
            if dist < 0.05 and abs(yaw_err) < 0.2:
                stop_cmd = Twist()
                self.pub.publish(stop_cmd)

            err = np.array([dist, yaw_err])

            integral += err * 0.1
            derivative = (err-prev_err) / 0.1
            prev_err = err

            control = (self.Kp @ err + self.Ki @ integral + self.Kd @ derivative)

            v = control[0]
            w = control[1]

            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.pub.publish(cmd)

            time.sleep(0.1)

    # ------------------------------------------------------------------
    # Callback when goal point is published
    # ------------------------------------------------------------------
    def planning_callback(self, msg: PointStamped):
        trajectory = plan_curved_trajectory((msg.point.x, msg.point.y))

        for waypoint in trajectory:
            self.controller(waypoint)

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_from_yaw(yaw):
        """Return quaternion (x, y, z, w) from yaw angle."""
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()