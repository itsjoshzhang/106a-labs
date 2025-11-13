#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointPosController(Node):
    def __init__(self, joint_angles):
        super().__init__('ur7e_joint_pos_controller')
        
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.joint_positions = list(joint_angles)
        self.pub = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10)
        
    
    def publish_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points.append(point)
        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    joint_angles = [float(i) for i in sys.argv[1:7]]
    node = JointPosController(joint_angles)

    rclpy.spin_once(node, timeout_sec=1)
    node.publish_trajectory()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()