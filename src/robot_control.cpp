#include "robot_control.hpp"

// Constructor: Initializes publishers and subscribers, and sets control parameters
UR5Control::UR5Control(const ros::NodeHandle& nh, double freq,
                       const motion_profile_settings_t &joints_profile, 
                       const cart_motion_profile_settings_t &cart_profile) :
    nh_(nh), freq_(freq),
    current_joints_vel_(Vector6d::Zero()),
    joints_profile_(joints_profile), 
    cart_profile_(cart_profile) 
{
    joint_state_subscriber_ = nh_.subscribe("/joint_states", 1, &UR5Control::jointStateCallback, this);
    joint_position_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);
}

// Callback: Handles incoming joint state messages and updates the joint angles and velocities
void UR5Control::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    Vector6d joints_pos;
    if (msg->position.size() == joints_pos.size()) { 
        for (size_t i = 0; i < joints_pos.size(); ++i) {
            joints_pos[i] = msg->position[i];
            current_joints_vel_[i] = msg->velocity[i];
        }
        //joint_angles_ = wrapToPi(joints_pos);
    } else {
        ROS_WARN("Received joint state with insufficient position data.");
    }
}

// Sets the desired joint positions by publishing to the joint position topic
void UR5Control::setJointPos(const Vector6d& joints_pos) {
    std_msgs::Float64MultiArray joint_positions; // Create a message to hold joint positions
    setJointAngles(joints_pos); // Update the internal joint angles with the new positions
    joint_positions.data.resize(6); // Resize the message array to hold 6 joint positions
    joint_positions.data.assign(joints_pos.data(), joints_pos.data() + joints_pos.size()); // Copy joint positions into the message
    joint_position_pub_.publish(joint_positions); // Publish the joint positions to the ROS topic
}

// Moves the robot to a target position using a point-to-point motion profile
bool UR5Control::movePtP(const Matrix4d& target_transform, const motion_profile_settings_t& profile_settings) {
    Matrix86d joint_solutions; // Storage for the inverse kinematics solutions
    int num_solutions = inverseKinematics(target_transform, joint_solutions); // Calculate inverse kinematics for target position
    if (num_solutions <= 0) { // Check if any solutions were found
        ROS_ERROR("Target position is not reachable"); // Log an error if not reachable
        return false; // Return false to indicate failure
    }

    // Find the nearest joint angles solution from the computed IK solutions
    Vector6d target_joints_angles = findNearestIKSolution(joint_angles_, joint_solutions);
    Ruckig<6> otg(1./freq_); // Initialize the trajectory generator with frequency
    InputParameter<6> input; // Create input parameters for motion
    OutputParameter<6> output; // Create output parameters for motion

    // Initialize input parameters for motion profile
    input.current_velocity.fill(0);
    input.current_acceleration.fill(0);
    input.target_velocity.fill(0);
    input.target_acceleration.fill(0);
    std::copy(joint_angles_.data(), joint_angles_.data() + 6, input.current_position.begin()); // Set current joint positions
    std::copy(target_joints_angles.data(), target_joints_angles.data() + 6, input.target_position.begin()); // Set target joint positions

    // Set motion profile limits
    input.max_velocity.fill(profile_settings.max_velocity);
    input.max_acceleration.fill(profile_settings.max_acceleration);
    input.max_jerk.fill(profile_settings.jerk);

    ros::Rate rate(freq_); // Set loop rate based on frequency
    while (otg.update(input, output) == Result::Working && ros::ok()) { // Update motion until completed or interrupted
        setJointPos(Map<Vector6d>(output.new_position.data(), 6)); // Set the new joint positions
        output.pass_to_input(input); // Pass output parameters to input for the next iteration
        rate.sleep(); // Sleep for the defined rate
        ros::spinOnce(); // Allow ROS to process incoming messages
    }
    return true; // Return true to indicate successful movement
}

// Moves the robot in a linear trajectory from the current to the target pose
bool UR5Control::moveLine(const Matrix4d& target_transform, const cart_motion_profile_settings_t& profile_settings) {
    Matrix86d target_solutions, current_solutions; // Storage for target and current IK solutions
    // Calculate inverse kinematics for target position
    if (inverseKinematics(target_transform, target_solutions) == 0) {
        ROS_ERROR("Target position is not reachable"); // Log an error if not reachable
        return false; // Return false to indicate failure
    }

    Vector6d initial_joint_angles = getJointAngles(); // Get the current joint angles
    Matrix4d current_transform = forwardKinematics(initial_joint_angles); // Calculate the current transformation from joint angles
    // Calculate inverse kinematics for the current position
    if (inverseKinematics(current_transform, current_solutions) == 0) {
        ROS_ERROR("Current position is invalid"); // Log an error if the current position is invalid
        return false; // Return false to indicate failure
    }

    int n_sol = 0; // Variable to hold the index of the valid solution
    for (int i = 0; i < current_solutions.rows(); ++i) { // Find the closest solution to the initial joint angles
        if (current_solutions.row(i).isApprox(initial_joint_angles.transpose(), 1e-5)) {
            n_sol = i; // Store the index of the matching solution
            break; // Exit loop upon finding the match
        }
    }

    // Check if the solution is valid
    if (target_solutions.row(n_sol).isZero() || target_solutions.row(n_sol).hasNaN()) {
        ROS_ERROR("Cannot generate linear motion"); // Log an error if linear motion cannot be generated
        return false; // Return false to indicate failure
    }

    InputParameter<7> input; // Create input parameters for motion with 7 dimensions
    OutputParameter<7> output; // Create output parameters for motion
    Ruckig<7> otg(1./freq_); // Initialize trajectory generator for 7 dimensions
    Quaterniond target_quat, current_quat; // Quaternions for target and current orientation
    Vector3d target_pos, current_pos; // Vectors for target and current position

    // Extract rotation and translation from the target and current transformation matrices
    extractRotationAndTranslation(target_transform, target_quat, target_pos);
    extractRotationAndTranslation(current_transform, current_quat, current_pos);
    // Initialize input parameters for motion profile
    input.current_velocity.fill(0);
    input.current_acceleration.fill(0);
    input.target_velocity.fill(0);
    input.target_acceleration.fill(0);
    // Set current and target positions and orientations in the input
    std::copy(current_quat.coeffs().data(), current_quat.coeffs().data() + 4, input.current_position.begin());
    std::copy(target_quat.coeffs().data(), target_quat.coeffs().data() + 4, input.target_position.begin());
    std::copy(current_pos.data(), current_pos.data() + 3, input.current_position.begin() + 4);
    std::copy(target_pos.data(), target_pos.data() + 3, input.target_position.begin() + 4);

    // Set maximum velocity, acceleration, and jerk for angular motion
    for (size_t i = 0; i < 4; ++i) {
        input.max_velocity[i] = profile_settings.angular.max_velocity;
        input.max_acceleration[i] = profile_settings.angular.max_acceleration;
        input.max_jerk[i] = profile_settings.angular.jerk;
    }
    // Set maximum velocity, acceleration, and jerk for linear motion
    for (size_t i = 4; i < 7; ++i) {
        input.max_velocity[i] = profile_settings.linear.max_velocity;
        input.max_acceleration[i] = profile_settings.linear.max_acceleration;
        input.max_jerk[i] = profile_settings.linear.jerk;
    }

    ros::Rate rate(freq_); // Set loop rate based on frequency
    while (otg.update(input, output) == Result::Working && ros::ok()) { // Update motion until completed or interrupted
        Quaterniond new_quat(Map<Vector4d>(output.new_position.data())); // Extract new quaternion from output
        new_quat.normalize(); // Normalize the quaternion
        Vector3d new_pos = Map<Vector3d>(output.new_position.data() + 4); // Extract new position from output

        // Create a new transformation matrix from the new quaternion and position
        Matrix4d new_transform = createTransformationMatrix(new_quat, new_pos);
        Matrix86d joint_solutions; // Storage for joint solutions
        int num_solutions = inverseKinematics(new_transform, joint_solutions); // Calculate IK for the new transform
        if (num_solutions <= 0) {
            ROS_ERROR("Target position is not reachable"); // Log an error if not reachable
            return false; // Return false to indicate failure
        }
        // Find the nearest joint angles solution from the computed IK solutions
        Vector6d target_joints_angles = findNearestIKSolution(getJointAngles(), joint_solutions);
        setJointPos(target_joints_angles); // Set the joint positions
        output.pass_to_input(input); // Pass output parameters to input for the next iteration
        rate.sleep(); // Sleep for the defined rate
        ros::spinOnce(); // Allow ROS to process incoming messages
    }
    return true; // Return true to indicate successful linear movement
}

bool UR5Control::movePtP(const Matrix4d& target_transform)
{
    return movePtP(target_transform, joints_profile_);
}

bool UR5Control::moveLine(const Matrix4d& target_transform)
{
    return moveLine(target_transform, cart_profile_);
}