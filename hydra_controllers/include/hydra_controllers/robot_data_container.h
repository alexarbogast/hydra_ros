#pragma once

#include <za_hw/za_state_interface.h>
#include <za_hw/za_model_interface.h>
#include <hydra_hw/positioner_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <Eigen/Dense>
#include <mutex>

namespace hydra_controllers {
enum class ControlMode {
    TaskPriorityControl,
    CoordinatedTaskPriorityControl
};

struct ZaDataContainer {
    std::unique_ptr<za_hw::ZaStateHandle> state_handle_;
    std::unique_ptr<za_hw::ZaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    Eigen::Vector3d position_d_;
    Eigen::Matrix<double, 6, 1> twist_setpoint_;
    std::unique_ptr<std::mutex> pose_twist_setpoint_mutex_;

    ControlMode mode_ = ControlMode::CoordinatedTaskPriorityControl;
};

struct PositionerDataContainer {
    std::unique_ptr<hydra_hw::PositionerStateHandle> state_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
};

} // namespace hydra_controllers