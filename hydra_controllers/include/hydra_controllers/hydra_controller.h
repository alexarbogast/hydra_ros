#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <dynamic_reconfigure/server.h>
#include <za_hw/za_state_interface.h>
#include <za_hw/za_model_interface.h>
#include <ros/node_handle.h>
#include <Eigen/Dense>

#include <za_controllers/taskpriority_paramConfig.h>

namespace hydra_controllers {

using JointHandles = std::vector<std::vector<hardware_interface::JointHandle>>;
using StateHandles = std::vector<std::unique_ptr<za_hw::ZaStateHandle>>;
using ModelHandles = std::vector<std::unique_ptr<za_hw::ZaModelHandle>>;

class HydraController : public controller_interface::MultiInterfaceController<
                                    hardware_interface::VelocityJointInterface,
                                    za_hw::ZaModelInterface,
                                    za_hw::ZaStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

private:

    // setpoints
    Eigen::Vector3d z_align_;

    //hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    JointHandles joint_handles_;
    StateHandles state_handles_;
    ModelHandles model_handles_;

    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<za_controllers::taskpriority_paramConfig>>
        dynamic_server_posvel_param_;
    ros::NodeHandle dynamic_reconfigure_posvel_param_node_;
    double Kp_, Ko_, Kr_;
    void taskpriorityParamCallback(za_controllers::taskpriority_paramConfig& config,
                             uint32_t level);
};

} // namespace hydra_controllers