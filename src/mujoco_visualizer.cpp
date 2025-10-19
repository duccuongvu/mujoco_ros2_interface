#include "mujoco_ros2_interface/mujoco_visualizer.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <filesystem>

MujocoVisualizer::MujocoVisualizer(mjModel* model, mjData* data)
    : model_(model), data_(data)
{
    if (!glfwInit()) throw std::runtime_error("Failed to initialize GLFW");

    window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulation", nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(window_, this);

    glfwSetKeyCallback(window_, keyCallback);
    glfwSetMouseButtonCallback(window_, mouseButtonCallback);
    glfwSetCursorPosCallback(window_, cursorPosCallback);
    glfwSetScrollCallback(window_, scrollCallback);

    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    mjv_makeScene(model_, &scn_, 2000);
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);

    paused_ = false;
    show_contact_ = false;
    show_wireframe_ = false;
    follow_target_ = false;
    std::cout << "[MuJoCo] Visualizer initialized. Use keyboard shortcuts for control.\n";

}

MujocoVisualizer::~MujocoVisualizer() {
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    if (window_) {
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
}

void MujocoVisualizer::render() {
    mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    mjr_render(viewport, &scn_, &con_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

bool MujocoVisualizer::shouldClose() const {
    return glfwWindowShouldClose(window_);
}

void MujocoVisualizer::keyCallback(GLFWwindow* window, int key, int, int action, int) {
    auto vis = static_cast<MujocoVisualizer*>(glfwGetWindowUserPointer(window));
    // if (action == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    //     mj_resetData(vis->model_, vis->data_);
    //     std::cout << "[MuJoCo] Simulation reset.\n";
    // }
    if (action != GLFW_PRESS) return;
    switch (key) {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;


        case GLFW_KEY_BACKSPACE:
            mj_resetData(vis->model_, vis->data_);
            RCLCPP_INFO(rclcpp::get_logger("mujoco_visualizer"), "[MuJoCo] Simulation reset.");
            break;

        case GLFW_KEY_J:
            vis->opt_.flags[mjVIS_JOINT] = !vis->opt_.flags[mjVIS_JOINT];
            RCLCPP_INFO(rclcpp::get_logger("mujoco_visualizer"), "[MuJoCo] Toggle joint visualization.");
            break;

        case GLFW_KEY_T:
            vis->opt_.flags[mjVIS_TRANSPARENT] = !vis->opt_.flags[mjVIS_TRANSPARENT];
            RCLCPP_INFO(rclcpp::get_logger("mujoco_visualizer"), "[MuJoCo] Toggle transparency.");
            break;

        default:
            break;
    }
}

void MujocoVisualizer::mouseButtonCallback(GLFWwindow* window, int button, int action, int) {
    auto vis = static_cast<MujocoVisualizer*>(glfwGetWindowUserPointer(window));
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    vis->last_x_ = xpos;
    vis->last_y_ = ypos;

    if (button == GLFW_MOUSE_BUTTON_LEFT)
        vis->left_pressed_ = (action == GLFW_PRESS);
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        vis->right_pressed_ = (action == GLFW_PRESS);
}

void MujocoVisualizer::cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    auto vis = static_cast<MujocoVisualizer*>(glfwGetWindowUserPointer(window));
    double dx = xpos - vis->last_x_;
    double dy = ypos - vis->last_y_;
    vis->last_x_ = xpos;
    vis->last_y_ = ypos;

    if (!vis->left_pressed_ && !vis->right_pressed_) return;
    const double sensitivity = 0.005;

    if (vis->left_pressed_)
        mjv_moveCamera(vis->model_, mjMOUSE_ROTATE_V, dx * sensitivity, dy * sensitivity, &vis->scn_, &vis->cam_);
    else if (vis->right_pressed_)
        mjv_moveCamera(vis->model_, mjMOUSE_MOVE_V, dx * sensitivity, dy * sensitivity, &vis->scn_, &vis->cam_);
}

void MujocoVisualizer::scrollCallback(GLFWwindow* window, double, double yoffset) {
    auto vis = static_cast<MujocoVisualizer*>(glfwGetWindowUserPointer(window));
    mjv_moveCamera(vis->model_, mjMOUSE_ZOOM, 0, yoffset * 0.1, &vis->scn_, &vis->cam_);
}
