#ifndef __ROBOT_KINEMATICS
#define __ROBOT_KINEMATICS

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "ur_kin.h"
#include <ruckig/ruckig.hpp>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Matrix86d = Eigen::Matrix<double, 8, 6>;
using Matrix68d = Eigen::Matrix<double, 6, 8>;
using MatrixXd = Eigen::MatrixXd;

using namespace ruckig;
using namespace Eigen;

class UR5Kinematics 
{
    public:
        Vector6d joint_angles_;
        UR5Kinematics(const Vector6d &joint_angles = Vector6d::Zero()) : 
            joint_angles_(joint_angles) {};
       
        Vector6d getJointAngles() const;
        void setJointAngles(const Vector6d& joint_angles);

        Matrix4d forwardKinematics(const Vector6d &joint_angles) const;
        int inverseKinematics(const Matrix4d &end_effector_pose, Matrix86d &solutions) const;
        Vector6d findNearestIKSolution(const Vector6d& target, Matrix86d& solutions) const;
        Vector6d calculateNearestIKSolution(const Matrix4d &end_effector_pose, const Vector6d &initial_joint_angles);
        Vector6d calculateNearestIKSolution(const Matrix4d &end_effector_pose);

        static Matrix3d eulerAnglesToRotationMatrix(const Vector3d& angles);
        static Vector3d rotationMatrixToEulerAngles(const Matrix3d& R);
        static Matrix4d createTransformationMatrix(const Quaterniond& quaternion, const Vector3d& translation);
        static void extractRotationAndTranslation(const Matrix4d& transform, Quaterniond& quaternion, Vector3d& translation);
        static Vector6d wrapToPi(const Vector6d& input);

};

#endif //__ROBOT_KINEMATICS