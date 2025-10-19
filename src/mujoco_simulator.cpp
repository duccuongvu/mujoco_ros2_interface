#include "mujoco_ros2_interface/mujoco_simulator.hpp"
#include <stdexcept>

MujocoSimulator::MujocoSimulator(const std::string& model_path) {
    char error[1000] = "Could not load model";
    model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
    if (!model_) throw std::runtime_error(error);
    data_ = mj_makeData(model_);
}

MujocoSimulator::~MujocoSimulator() {
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
}

void MujocoSimulator::step() {
    if (!model_ || !data_) return;
    mj_step(model_, data_);
}

void MujocoSimulator::reset() {
    if (model_ && data_) mj_resetData(model_, data_);
}

std::vector<double> MujocoSimulator::getPositions() const {
    return std::vector<double>(data_->qpos, data_->qpos + model_->nq);
}

std::vector<double> MujocoSimulator::getVelocities() const {
    return std::vector<double>(data_->qvel, data_->qvel + model_->nv);
}

void MujocoSimulator::setTorques(const std::vector<double>& torques) {
    int n = std::min<int>(torques.size(), model_->nu);
    for (int i = 0; i < n; ++i) data_->ctrl[i] = torques[i];
}
