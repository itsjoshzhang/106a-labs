#!usr/bin/env python

import numpy as np
import scipy as sp
import forward_kinematics.kin_func_skeleton as kfs 

def ur7e_foward_kinematics_from_angles(joint_angles):
    """
    Calculate the orientation of the ur7e's end-effector tool given
    the joint angles of each joint in radians

    Parameters:
    ------------
    joint_angles ((6x) np.ndarray): 6 joint angles (s0, s1, e0, w1, w2, w3)

    Returns: 
    ------------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    q0 = np.ndarray((3, 6)) # Points on each joint axis in the zero config
    w0 = np.ndarray((3, 6)) # Axis vector of each joint axis in the zero config


    q0[:, 0] = [0., 0., 0.1625] # shoulder pan joint - shoulder_link
    q0[:, 1] = [0., 0., 0.1625] # shoulder lift joint - upper_arm_link
    q0[:, 2] = [0.425, 0., 0.1625] # elbow_joint - forearm_link
    q0[:, 3] = [0.817, 0.1333, 0.1625] # wrist 1 - wrist_1_link
    q0[:, 4] = [0.817, 0.1333, 0.06285] # wrist 2 - wrist_2_link
    q0[:, 5] = [0.817, 0.233, 0.06285] # wrist 3 - wrist_3_link

    w0[:, 0] = [0., 0., 1] # shoulder pan joint
    w0[:, 1] = [0, 1., 0] # shoulder lift joint
    w0[:, 2] = [0., 1., 0] # elbow_joint
    w0[:, 3] = [0., 1., 0] # wrist 1
    w0[:, 4] = [0., 0., -1] # wrist 2 
    w0[:, 5] = [0., 1., 0] # wrist 3

    # Rotation matrix from base_link to wrist_3_link in zero config
    R = np.array([[-1., 0., 0.],
                  [0., 0., 1.], 
                  [0., 1., 0.]])

    # YOUR CODE HERE (Task 1)
    g0 = np.eye(4)
    g0[:3, :3] = R
    g0[:3, 3] = q0[:, 5]
    ei = np.zeros((6, 6))

    for i in range(6):
        q = q0[:, i]
        w = w0[:, i]
        v = np.cross(q, w)
        ei[:3, i] = v
        ei[3:, i] = w
    
    return kfs.prod_exp(ei, joint_angles) @ g0


def ur7e_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the ur7e's end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of ur7e robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    # YOUR CODE HERE (Task 2)
    joint_dict = dict(zip(list(joint_state.name), list(joint_state.position)))
    name_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                      'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    angles = np.array([joint_dict[name] for name in name_order])
    return ur7e_foward_kinematics_from_angles(angles)
    # END YOUR CODE HERE