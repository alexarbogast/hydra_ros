#pragma once

#include <za_hw/za_state_interface.h>
#include <za_hw/za_model_interface.h>
#include <hydra_hw/positioner_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <Eigen/Dense>
#include <mutex>

#include <cartesian_trajectory/cartesian_state.h>

namespace hydra_controllers {
enum class ControlMode {
    TaskPriorityControl,
    CoordinatedTaskPriorityControl
};

struct ZaDataContainer {
    std::unique_ptr<za_hw::ZaStateHandle> state_handle_;
    std::unique_ptr<za_hw::ZaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    cartesian_controllers::CartesianState setpoint_;

    ControlMode mode_ = ControlMode::TaskPriorityControl;
};

struct PositionerDataContainer {
    std::unique_ptr<hydra_hw::PositionerStateHandle> state_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
};

} // namespace hydra_controllers