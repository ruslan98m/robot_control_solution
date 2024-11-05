#!/usr/bin/python3

import rospy
import numpy as np
from robot_control import UR5Control
from robot_control_solution.robot_kinematics import UR5Kinematics

def run() -> None:
    # Initialize the ROS node
    rospy.init_node('robot_motion_control')

    robot = UR5Control()  # Initialize robot control

    # Define the desired end effector orientation and position
    roll = 0
    pitch = np.pi / 2
    yaw = 0
    z = 0.3
    y = [-0.3, 0.3, 0.3, -0.3]
    x = [0.3, 0.3, 0.2, 0.2]

    # Create rotation matrix from Euler angles
    euler_angles = np.array([roll, pitch, yaw])
    R = UR5Kinematics.eulerAnglesToRotationMatrix(euler_angles)  # Method needs to be defined in your class
    # Create target poses
    target_poses = []
    for i in range(len(x)):
        target_pose = np.eye(4)  # 4x4 Identity matrix
        target_pose[:3, :3] = R  # Set rotation part
        target_pose[0, 3] = x[i]  # Set x position
        target_pose[1, 3] = y[i]  # Set y position
        target_pose[2, 3] = z      # Set z position
        target_poses.append(target_pose)

    robot.movePtP(target_poses[0])
    while not rospy.is_shutdown():
        for pose in target_poses:
            if not robot.moveLine(pose):
                rospy.logerr("Trajectory error")
                return
            rospy.loginfo("Trajectory finished")

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass