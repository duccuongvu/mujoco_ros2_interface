#pragma once
#include "mujoco_ros2_interface/mujoco_simulator.hpp"
#include "mujoco_ros2_interface/mujoco_visualizer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <memory>

#define VISUALIZE 1

class MujocoNode : public rclcpp::Node {
public:
    MujocoNode();

private:
    void stepSimulation();
    void torqueCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    sensor_msgs::msg::JointState joint_msg_;
    
    std::unique_ptr<MujocoSimulator> sim_;
#if VISUALIZE == 1
    std::unique_ptr<MujocoVisualizer> vis_;
#endif

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr torque_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
