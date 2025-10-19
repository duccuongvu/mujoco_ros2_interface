#include "mujoco_ros2_interface/mujoco_node.hpp"

MujocoNode::MujocoNode() : Node("mujoco_node") {
    RCLCPP_INFO(this->get_logger(), "Starting MuJoCo node...");

    std::string model_path = this->declare_parameter<std::string>("model_path", "");
    if (model_path.empty())
        throw std::runtime_error("Parameter 'model_path' not set!");

    sim_ = std::make_unique<MujocoSimulator>(model_path);

#if VISUALIZE == 1
    vis_ = std::make_unique<MujocoVisualizer>(sim_->model(), sim_->data());
#endif

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    torque_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_torque_cmd", 10,
        std::bind(&MujocoNode::torqueCallback, this, std::placeholders::_1));
    
        // Initialize joint names ONCE

    joint_msg_.name.reserve(sim_->model()->njnt);
    for (int i = 0; i < sim_->model()->njnt; ++i) {
        if (sim_->model()->names) {
            joint_msg_.name.push_back(
                std::string(sim_->model()->names + sim_->model()->name_jntadr[i]));
        } else {
            joint_msg_.name.push_back("joint_" + std::to_string(i));
        }
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(2),
                               std::bind(&MujocoNode::stepSimulation, this));
    // timer_ = create_wall_timer(
    //     std::chrono::duration<double>(sim_->model()->opt.timestep),
    //     std::bind(&MujocoNode::stepSimulation, this));

}

void MujocoNode::stepSimulation() {
    // sim_->step(sim_->model()->opt.timestep);

    static auto sim_start = std::chrono::steady_clock::now();
    static double sim_time = 0.0;

    // Compute elapsed real time
    auto now = std::chrono::steady_clock::now();
    double real_time = std::chrono::duration<double>(now - sim_start).count();
    double dt = sim_->model()->opt.timestep;
    // Step simulation until sim_time catches up to real_time
    while (sim_time < real_time) 
    {
        sim_->step();
        sim_time += dt;
    }



    auto pos = sim_->getPositions();
    auto vel = sim_->getVelocities();

    // Reuse the joint_msg_ with predefined names
    joint_msg_.header.stamp = this->get_clock()->now();
    joint_msg_.position = pos;
    joint_msg_.velocity = vel;
    joint_pub_->publish(joint_msg_);

#if VISUALIZE == 1
    vis_->render();
    if (vis_->shouldClose()) rclcpp::shutdown();
#endif
}

void MujocoNode::torqueCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    sim_->setTorques(msg->effort);
}
