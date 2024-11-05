#ifndef UR5CONTROL_HPP
#define UR5CONTROL_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <ruckig/ruckig.hpp>
#include "robot_kinematics.hpp"  

// Structure to hold motion profile settings
struct motion_profile_settings_t {
    double max_velocity;     // Maximum velocity allowed for the motion profile
    double max_acceleration; // Maximum acceleration allowed for the motion profile
    double jerk;             // Maximum jerk (rate of change of acceleration) allowed for the motion profile
};

// Structure to hold both angular and linear motion profile settings
struct cart_motion_profile_settings_t {
    motion_profile_settings_t angular; // Motion profile settings for angular movements
    motion_profile_settings_t linear;   // Motion profile settings for linear movements
};

// Default motion profile settings for Cartesian motion with specified values for angular and linear profiles
const cart_motion_profile_settings_t cart_profile_def = {
    .angular = {0.1, 0.2, 0.3}, // Default angular settings: max_velocity = 0.1, max_acceleration = 0.2, jerk = 0.3
    .linear = {0.1, 0.2, 0.3},   // Default linear settings: max_velocity = 0.1, max_acceleration = 0.2, jerk = 0.3
};

// Default motion profile settings for joint movements with specified values
const motion_profile_settings_t joints_profile_def = {
    0.3, // Maximum velocity for joint movements
    1.0, // Maximum acceleration for joint movements
    1.0  // Maximum jerk for joint movements
};

using namespace Eigen;

class UR5Control : public UR5Kinematics {
private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_subscriber_;
    ros::Publisher joint_position_pub_;
    Vector6d current_joints_vel_;                // Current joint velocities
    cart_motion_profile_settings_t cart_profile_; // Cartesian motion profile settings
    motion_profile_settings_t joints_profile_;   // Joint motion profile settings
    double freq_;                                // Control frequency

public:
    // Callback for joint state updates from the robot
    virtual void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    // Constructor to initialize the UR5 control interface
    UR5Control(const ros::NodeHandle& nh, double freq = 100.0,
               const motion_profile_settings_t &joints_profile = joints_profile_def, 
               const cart_motion_profile_settings_t &cart_profile = cart_profile_def);

    // Method to set joint positions
    void setJointPos(const Vector6d& joints_pos);
    // Method to move the robot to a target position using point-to-point motion
    bool movePtP(const Matrix4d& target_transform, const motion_profile_settings_t &profile_settings);
    bool movePtP(const Matrix4d& target_transform);
    // Method to move the robot in a linear trajectory to a target position
    bool moveLine(const Matrix4d& target_transform, const cart_motion_profile_settings_t &profile_settings);
    bool moveLine(const Matrix4d& target_transform);
};

#endif // UR5CONTROL_HPP
