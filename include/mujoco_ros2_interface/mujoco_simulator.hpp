#pragma once
#include <mujoco/mujoco.h>
#include <string>
#include <vector>

class MujocoSimulator {
public:
    MujocoSimulator(const std::string& model_path);
    ~MujocoSimulator();

    void step();
    void init();
    void reset();
    std::vector<double> getPositions() const;
    std::vector<double> getVelocities() const;
    void setTorques(const std::vector<double>& torques);

    mjModel* model() const { return model_; }
    mjData* data() const { return data_; }

private:
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
};
