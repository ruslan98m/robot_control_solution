import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from robot_control_solution.robot_kinematics import UR5Kinematics
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from dataclasses import dataclass

@dataclass
class MotionProfileSettings:
    max_velocity: float  # Maximum velocity
    max_acceleration: float  # Maximum acceleration
    jerk: float  # Maximum jerk

@dataclass
class CartMotionProfileSettings:
    angular: MotionProfileSettings  # Angular motion profile settings
    linear: MotionProfileSettings  # Linear motion profile settings


class UR5Control(UR5Kinematics): 
    def __init__(self, freq: int = 100, joints_profile: MotionProfileSettings = None, 
                 cart_profile: CartMotionProfileSettings = None):
        """
        Initialize the UR5Control class.

        :param freq: Frequency of control loop in Hz.
        :param joints_profile: Motion profile settings for joint movement.
        :param cart_profile: Motion profile settings for Cartesian movement.
        """
        super().__init__()

        # Set default motion profile settings if not provided
        if joints_profile is None:
            joints_profile = MotionProfileSettings(0.3, 1.0, 1.0)  # max_velocity, max_acceleration, jerk
        
        if cart_profile is None:
            cart_profile = CartMotionProfileSettings(
                MotionProfileSettings(0.1, 0.2, 0.3),  # Angular motion settings
                MotionProfileSettings(0.1, 0.2, 0.3)   # Linear motion settings
            )
        
        # Class attributes initialization
        self.__freq: int = freq  # Control loop frequency
        self.__current_joints_vel: np.ndarray = np.zeros(6)  # Current joint velocities
        self.__current_joints_pos: np.ndarray = np.zeros(6)  # Current joint positions
        self.__joints_last_pos: np.ndarray = np.zeros(6)
        self.__joints_profile: MotionProfileSettings = joints_profile  # Joint motion profile settings
        self.__cart_profile: CartMotionProfileSettings = cart_profile  # Cartesian motion profile settings

        # Subscribe to joint state updates and set up publishers
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.jointStateCallback)
        self.joint_position_pub = rospy.Publisher("/joint_group_position_controller/command", Float64MultiArray, queue_size=10)

    def jointStateCallback(self, msg: JointState) -> None:
        """
        Callback for updating joint states from incoming messages.

        :param msg: The joint state message received.
        """
        if len(msg.position) == 6:  
            # Update the current joint velocities and positions
            self.__current_joints_vel = np.array(msg.velocity)
            self.__current_joints_pos = np.array(msg.position)  
        else:
            rospy.logwarn("Received joint state with insufficient position data.")

    def readJointsStates(self):
        """
        Read the current joint positions and velocities.

        :return: A tuple containing current joint positions and velocities.
        """
        return self.__current_joints_pos, self.__current_joints_vel
    
    def setJointPos(self, joints_pos: np.ndarray) -> None:
        """
        Publish the desired joint positions to the joint position topic.

        :param joints_pos: Array of target joint positions.
        """
        joint_positions = Float64MultiArray()
        joint_positions.data = list(joints_pos)
        self.__joints_last_pos = joints_pos
        self.joint_position_pub.publish(joint_positions)

    def movePtP(self, target_transform: np.ndarray, profile_settings: MotionProfileSettings = None) -> bool:
        """
        Move the robot in a point-to-point motion to the specified target transformation.

        :param target_transform: The target transformation matrix.
        :param profile_settings: Optional motion profile settings for the movement.
        :return: True if the movement was successful, False otherwise.
        """
        try:
            if profile_settings is None:
                profile_settings = self.__joints_profile

            # Obtain possible joint solutions for the target transformation
            solutions = np.empty((8, 6), dtype=float)
        
            num_solutions = self.inverseKinematics(target_transform, solutions)
            if num_solutions == 0:
                rospy.logerr("Target position is not reachable")
                return False

            # Find the nearest joint solution for the desired target
            target_joints_angles = self.calculateNearestIKSolution(target_transform, self.__joints_last_pos)
            
            # Initialize Ruckig for 6 joints
            otg = Ruckig(6, 1.0/self.__freq)
            input = InputParameter(6)  # Input parameter for 6 joints
            output = OutputParameter(6)  # Output parameter for 6 joints

            # Set initial conditions
            input.current_velocity = np.zeros(6)
            input.current_acceleration = np.zeros(6)
            input.target_velocity = np.zeros(6)
            input.target_acceleration = np.zeros(6)
            input.current_position = self.__joints_last_pos  # Current joint angles
            input.target_position = target_joints_angles  # Target joint angles

            # Set maximum motion parameters
            input.max_velocity = np.full(6, self.__joints_profile.max_velocity)
            input.max_acceleration = np.full(6, self.__joints_profile.max_acceleration)
            input.max_jerk = np.full(6, self.__joints_profile.jerk)

            # Update position loop
            rate = rospy.Rate(self.__freq)
            res = Result.Working
            while (res == Result.Working) and (not rospy.is_shutdown()):
                res = otg.update(input, output)
                # Set new joint angles
                self.setJointPos(np.array(output.new_position))
                output.pass_to_input(input)  # Update input parameters
                rate.sleep()  # Sleep to maintain the control frequency
            return True
        except Exception as e:
            rospy.logerr(e)
            return False
    
    def moveLine(self, target_transform: np.ndarray, profile_settings: CartMotionProfileSettings = None) -> bool:
        """
        Move the robot in a linear trajectory to the specified target transformation.

        :param target_transform: The target transformation matrix.
        :param profile_settings: Optional motion profile settings for the movement.
        :return: True if the movement was successful, False otherwise.
        """
        try:
            if profile_settings is None:
                profile_settings = self.__cart_profile
                
            # Get the current joint angles and convert them to a transformation matrix
            current_transform = self.forwardKinematics(self.__joints_last_pos)

            # Extract quaternions and positions using methods from UR5Kinematics
            current_quat, current_pos = self.extractRotationAndTranslation(current_transform)
            target_quat, target_pos = self.extractRotationAndTranslation(target_transform)

            # Obtain solutions for the target pose
            target_solutions = np.empty((8, 6), dtype=float)
            num_solutions = self.inverseKinematics(target_transform, target_solutions)
            if num_solutions == 0:
                rospy.logerr("Target position is not reachable")
                return False

            # Initialize Ruckig for movement
            input = InputParameter(7)  # Input parameter for 7 elements (4 for quaternion, 3 for position)
            output = OutputParameter(7)  # Output parameter for 7 elements
            otg = Ruckig(7, 1.0/self.__freq)  # Create Ruckig instance with the specified frequency

            # Set initial conditions
            input.current_velocity = np.zeros(7)
            input.current_acceleration = np.zeros(7)
            input.target_velocity = np.zeros(7)
            input.target_acceleration = np.zeros(7)

            # Set current and target positions
            input.current_position = np.concatenate((current_quat, current_pos))
            input.target_position = np.concatenate((target_quat, target_pos))

            input.max_velocity = np.empty(7, dtype=float)
            input.max_acceleration = np.empty(7, dtype=float)
            input.max_jerk = np.empty(7, dtype=float)

            input.max_velocity = np.concatenate((np.full(4,self.__cart_profile.angular.max_velocity), np.full(3,self.__cart_profile.linear.max_velocity)))
            input.max_acceleration = np.concatenate((np.full(4,self.__cart_profile.angular.max_acceleration), np.full(3,self.__cart_profile.linear.max_acceleration)))
            input.max_jerk = np.concatenate((np.full(4,self.__cart_profile.angular.jerk), np.full(3,self.__cart_profile.linear.jerk)))

            rate = rospy.Rate(self.__freq)
            res = Result.Working
            while (res == Result.Working) and (not rospy.is_shutdown()):
                res = otg.update(input, output)
                # Extract new positions and convert them for inverse kinematics
                new_quat = output.new_position[:4]
                new_pos = output.new_position[4:]
                
                new_quat /= np.linalg.norm(new_quat)
                new_transform = self.createTransformationMatrix(new_quat, new_pos)
                target_solutions = np.empty((8, 6), dtype=float)
                num_solution = self.inverseKinematics(new_transform, target_solutions)
                if num_solution == 0:
                    rospy.logerr("Target position is not reachable")
                    return False
                
                # Find the nearest joint solution
                target_joints_angles = self.calculateNearestIKSolution(new_transform, self.__joints_last_pos)
                self.setJointPos(target_joints_angles)
                
                output.pass_to_input(input)  # Update input parameters
                rate.sleep()
            return True
        except Exception as e:
            rospy.logerr(e)
            return False
    
    def extractRotationAndTranslation(self, transformation: np.ndarray) -> tuple:
        """
        Extracts the rotation (as a quaternion) and translation from a transformation matrix.

        :param transformation: A 4x4 transformation matrix.
        :return: A tuple containing the quaternion and translation vector.
        """
        # Extract rotation part
        rotation_matrix = transformation[:3, :3]
        translation_vector = transformation[:3, 3]

        # Convert rotation matrix to quaternion
        quaternion = self.rotationMatrixToQuaternion(rotation_matrix)

        return quaternion, translation_vector
    
    def rotationMatrixToQuaternion(self, rotation_matrix: np.ndarray) -> np.ndarray:
        """
        Converts a rotation matrix into a quaternion representation.

        :param rotation_matrix: A 3x3 numpy array representing the rotation matrix.
        :return: A numpy array representing the quaternion in the form [x, y, z, w].
        """
        # Check if the input is a valid rotation matrix
        if not np.allclose(np.dot(rotation_matrix, rotation_matrix.T), np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation_matrix), 1.0, atol=1e-6):
            raise ValueError("Input matrix is not a valid rotation matrix.")
        
        # Calculate the trace of the rotation matrix
        trace = np.trace(rotation_matrix)

        # If the trace is greater than zero, use the first method to calculate the quaternion
        if trace > 0:
            w = np.sqrt(1.0 + trace) / 2.0  # Calculate the scalar part (w) of the quaternion
            x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4.0 * w)  # Calculate x component
            y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4.0 * w)  # Calculate y component
            z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4.0 * w)  # Calculate z component
        else:
            # If the trace is not positive, use the largest diagonal element to determine the dominant axis
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                # Calculate x component based on the first row/column
                x = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) / 2.0
                w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4.0 * x)  # Calculate w component
                y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / (4.0 * x)  # Calculate y component
                z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / (4.0 * x)  # Calculate z component
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                # Calculate y component based on the second row/column
                y = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) / 2.0
                w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4.0 * y)  # Calculate w component
                x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / (4.0 * y)  # Calculate x component
                z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / (4.0 * y)  # Calculate z component
            else:
                # Calculate z component based on the third row/column
                z = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) / 2.0
                w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4.0 * z)  # Calculate w component
                x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / (4.0 * z)  # Calculate x component
                y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / (4.0 * z)  # Calculate y component

        # Return the quaternion as a numpy array in the form [x, y, z, w]
        return np.array([x, y, z, w])

    
    def createTransformationMatrix(self, quaternion: np.ndarray, translation: np.ndarray) -> np.ndarray:
        """
        Creates a 4x4 transformation matrix from a quaternion and a translation vector.

        :param quaternion: A numpy array representing the quaternion [x, y, z, w].
        :param translation: A numpy array representing the translation vector [tx, ty, tz].
        :return: A 4x4 transformation matrix.
        """
        x, y, z, w = quaternion

        # Create rotation matrix from quaternion
        rotation_matrix = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])

        # Create the transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation

        return transformation_matrix