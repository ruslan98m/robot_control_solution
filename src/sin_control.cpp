#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

class SineWaveJointController {
public:
    SineWaveJointController() {
        // Initialize the node and create the publisher
        nh_ = ros::NodeHandle();
        joint_position_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 10);

        // Load parameters from the parameter server
        nh_.param("frequency", frequency_, 0.5);  // Frequency of the sine wave
        nh_.param("joint_limits/shoulder_pan_joint/min", min_limits_[0], -1.57); // Minimum limit
        nh_.param("joint_limits/shoulder_pan_joint/max", max_limits_[0], 1.57); // Maximum limit
        nh_.param("joint_limits/shoulder_lift_joint/min", min_limits_[1], -1.57);
        nh_.param("joint_limits/shoulder_lift_joint/max", max_limits_[1], 1.57);
        nh_.param("joint_limits/elbow_joint/min", min_limits_[2], -2.9);
        nh_.param("joint_limits/elbow_joint/max", max_limits_[2], 2.9);
        nh_.param("joint_limits/wrist_1_joint/min", min_limits_[3], -1.57);
        nh_.param("joint_limits/wrist_1_joint/max", max_limits_[3], 1.57);
        nh_.param("joint_limits/wrist_2_joint/min", min_limits_[4], -1.57);
        nh_.param("joint_limits/wrist_2_joint/max", max_limits_[4], 1.57);
        nh_.param("joint_limits/wrist_3_joint/min", min_limits_[5], -1.57);
        nh_.param("joint_limits/wrist_3_joint/max", max_limits_[5], 1.57);

        time_ = ros::Time::now();  // Initialize time
        joint_names_ = {
            "shoulder_pan_joint", 
            "shoulder_lift_joint", 
            "elbow_joint", 
            "wrist_1_joint", 
            "wrist_2_joint", 
            "wrist_3_joint"
        };
    }

    void run() {
        ros::Rate loop_rate(50);  // 50 Hz
        while (ros::ok()) {

            ros::Duration duration = ros::Time::now() - time_;
            std_msgs::Float64MultiArray joint_positions;
            joint_positions.data.resize(6);

            // Fill sinusoidal values
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                double mid_range = (max_limits_[i] + min_limits_[i]) / 2.0;
                double half_range = (max_limits_[i] - min_limits_[i]) / 2.0;

                // Generate the sinusoidal position within specified joint limits
                joint_positions.data[i] = mid_range + half_range * std::sin(frequency_ * duration.toSec() + i);
            }
            joint_position_pub_.publish(joint_positions);  // Publish the position
            ros::spinOnce();  
            loop_rate.sleep();  
        }
    }

private:
    ros::NodeHandle nh_;  // Node handle
    ros::Publisher joint_position_pub_;  // Publisher for joint trajectory
    std::vector<std::string> joint_names_;  // Joint names
    double amplitude_;  // Amplitude of the sine wave
    double frequency_;  // Frequency of the sine wave
    ros::Time time_;  // Time variable
    double min_limits_[6];  // Minimum limits for joints
    double max_limits_[6];  // Maximum limits for joints
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sine_wave_joint_controller"); 
    SineWaveJointController controller;  
    controller.run(); 
    return 0;  
}