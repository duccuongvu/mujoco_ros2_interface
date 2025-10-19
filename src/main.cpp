#include "mujoco_ros2_interface/mujoco_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MujocoNode>());
    rclcpp::shutdown();
    return 0;
}
