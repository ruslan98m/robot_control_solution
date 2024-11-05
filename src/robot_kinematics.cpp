#include "robot_kinematics.hpp"

Matrix3d UR5Kinematics::eulerAnglesToRotationMatrix(const Vector3d& angles) 
{
    Matrix3d R_x = Eigen::AngleAxisd(angles(0), Eigen::Vector3d::UnitX()).toRotationMatrix();
    Matrix3d R_y = Eigen::AngleAxisd(angles(1), Eigen::Vector3d::UnitY()).toRotationMatrix();
    Matrix3d R_z = Eigen::AngleAxisd(angles(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return R_z * R_y * R_x;
}

Vector3d UR5Kinematics::rotationMatrixToEulerAngles(const Matrix3d& R) 
{
    double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)); 
    bool singular = sy < 1e-6; 
    double x, y, z;
    if (!singular) {
        x = std::atan2(R(2, 1), R(2, 2)); // roll
        y = std::atan2(-R(2, 0), sy);      // pitch
        z = std::atan2(R(1, 0), R(0, 0));  // yaw
    } else {
        x = std::atan2(-R(1, 2), R(1, 1)); // roll
        y = std::atan2(-R(2, 0), sy);      // pitch
        z = 0;                              // yaw
    }
    return Eigen::Vector3d(x, y, z); 
}

// Function to extract rotation and translation from a transformation matrix
void UR5Kinematics::extractRotationAndTranslation(const Matrix4d& transform, Quaterniond& quaternion, Vector3d& translation) {
    // Extract translation vector from the transformation matrix
    translation = transform.block<3, 1>(0, 3); // Gets the last column (translation part)
    // Extract the rotation matrix from the transformation matrix
    Matrix3d rotation_matrix = transform.block<3, 3>(0, 0); // Gets the upper-left 3x3 submatrix (rotation part)
    // Set the quaternion from the rotation matrix
    quaternion = Quaterniond(rotation_matrix); // Converts the rotation matrix to a quaternion
}

Matrix4d UR5Kinematics::createTransformationMatrix(const Quaterniond& quaternion, const Vector3d& translation) {
    Matrix4d transform = Matrix4d::Identity(); // Initialize a 4x4 identity matrix
    // Set the rotation part of the matrix from the quaternion
    transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    // Set the translation part of the matrix
    transform.block<3, 1>(0, 3) = translation;
    return transform;
}    

// Method for forward kinematics
Matrix4d UR5Kinematics::forwardKinematics(const Vector6d &joint_angles) const {
    Matrix4d transform; 
    // Calls the external library function to compute forward kinematics
    ur_kinematics::forward(joint_angles.data(), transform.data());  
    return transform.transpose(); // Returns the transformation matrix representing the end effector pose
}

/* Function solving the inverse kinematics problem */
int UR5Kinematics::inverseKinematics(const Matrix4d &end_effector_pose, Matrix86d &solutions) const {
    Matrix68d sols_inv; // Temporary storage for inverse solutions
    Matrix4d end_effector_pose_inv = end_effector_pose.transpose();
    // Calls the external library function to compute inverse kinematics
    int num_solutions = ur_kinematics::inverse(end_effector_pose_inv.data(), sols_inv.data()); 
    solutions = sols_inv.transpose(); // Transposes solutions for output
    return num_solutions; // Returns the number of valid solutions found
}

// Function to find the nearest vector from a list of solutions
Vector6d UR5Kinematics::findNearestIKSolution(const Vector6d& target, Matrix86d& solutions) const {
    double min_distance = std::numeric_limits<double>::max(); // Initialize minimum distance to a large value
    int nearest_solution_idx = 0; 
    // Iterate through all solutions to find the nearest one
    for (int i = 0; i < solutions.rows(); i++) {
        if (solutions.row(i).isZero() || solutions.row(i).hasNaN()) continue; // Skip invalid solutions
        double distance = (wrapToPi(solutions.row(i)) - wrapToPi(target.transpose())).norm(); // Calculate the Euclidean distance
        // Update nearest vector if a closer one is found
        if (distance < min_distance) {
            min_distance = distance; // Update minimum distance
            nearest_solution_idx = i; // Update nearest solution index
        }
    }
    return wrapToPi(solutions.row(nearest_solution_idx)); // Return the nearest solution found
}

// Combined function to compute the nearest IK solution based on the initial joint angles
Vector6d UR5Kinematics::calculateNearestIKSolution(const Matrix4d &end_effector_pose, const Vector6d &initial_joint_angles) {
    // Calculate all possible IK solutions
    Matrix86d solutions;
    if (inverseKinematics(end_effector_pose, solutions) > 0) {
        return findNearestIKSolution(initial_joint_angles, solutions); // Find the nearest valid solution
    }
    return getJointAngles(); 
}

// Overloaded function using the member variable as default
Vector6d UR5Kinematics::calculateNearestIKSolution(const Matrix4d &end_effector_pose) {
    Vector6d sol = calculateNearestIKSolution(end_effector_pose, getJointAngles());
    return wrapToPi(sol);// Use current joint angles as initial guess
};

Vector6d UR5Kinematics::wrapToPi(const Vector6d& input) {
    Vector6d output;
    for (int i = 0; i < input.size(); ++i) {
        output[i] = std::fmod(input[i] + M_PI, 2 * M_PI); 
        if (output[i] < 0) {
            output[i] += 2 * M_PI;  
        }
        output[i] -= M_PI; 
    }
    return output;
}

Vector6d UR5Kinematics::getJointAngles() const{
    return joint_angles_;
}

void UR5Kinematics::setJointAngles(const Vector6d& joint_angles) {
    joint_angles_ = joint_angles;
}