#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

class MujocoVisualizer {
public:
    MujocoVisualizer(mjModel* model, mjData* data);
    ~MujocoVisualizer();

    void render();
    bool shouldClose() const;

private:
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    GLFWwindow* window_ = nullptr;
    mjModel* model_;
    mjData* data_;

    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;

    bool left_pressed_ = false;
    bool right_pressed_ = false;
    double last_x_ = 0.0;
    double last_y_ = 0.0;

    // State flags
    bool paused_ = false;
    bool show_contact_ = false;
    bool show_wireframe_ = false;
    bool follow_target_ = false;

    bool ctrl_pressed_ = false;
    int selected_body_ = -1;

};
