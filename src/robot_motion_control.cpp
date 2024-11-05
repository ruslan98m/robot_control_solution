#include <ros/ros.h>
#include <Eigen/Dense>
#include "robot_control.hpp"


using namespace std;


void run() {
    ros::NodeHandle nh; 
    double freq = 100;
    UR5Control robot(nh, freq); // Инициализация робота

    double roll = 0;   
    double pitch = M_PI/2;  
    double yaw = 0;    
    double z = 0.3;
    double y[] = {-0.3, 0.3, 0.3, -0.3};
    double x[] = {0.3, 0.3, 0.2, 0.2};
    Vector3d eulerAngles(roll, pitch, yaw);
    Matrix3d R = UR5Kinematics::eulerAnglesToRotationMatrix(eulerAngles);
    std::array<Matrix4d, 4> target_poses;

    for (int i = 0; i < target_poses.size(); ++i) {
        target_poses[i] = Matrix4d::Identity();
        target_poses[i].block<3, 3>(0, 0) = R;
        target_poses[i](0, 3) = x[i];
        target_poses[i](1, 3) = y[i];
        target_poses[i](2, 3) = z;
    }

    ros::Rate rate(10);
    robot.moveLine(target_poses[0]);
    while (ros::ok()) {
        for (const auto& pose : target_poses) {
            if(!robot.moveLine(pose))
            {
                ROS_ERROR("Trajectory error");
                return;
            }
            ROS_INFO("Trajectory finished");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_motion_control"); 
    run(); 
    return 0;  
}