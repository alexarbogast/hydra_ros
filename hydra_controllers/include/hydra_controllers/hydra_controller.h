#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <dynamic_reconfigure/server.h>
#include <za_hw/za_state_interface.h>
#include <za_hw/za_model_interface.h>
#include <za_msgs/PosVelSetpoint.h>
#include <ros/node_handle.h>
#include <Eigen/Dense>

#include <za_controllers/taskpriority_paramConfig.h>

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

    ControlMode mode_ = ControlMode::TaskPriorityControl;
};

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
    std::map<std::string, ZaDataContainer> arms_data_;

    Eigen::Vector3d z_align_;
    std::vector<ros::Subscriber> setpoints_subs_;

    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<za_controllers::taskpriority_paramConfig>>
        dynamic_server_posvel_param_;
    ros::NodeHandle dynamic_reconfigure_posvel_param_node_;
    double Kp_, Ko_, Kr_;
    void taskpriorityParamCallback(za_controllers::taskpriority_paramConfig& config,
                             uint32_t level);

    std::vector<ros::Subscriber> sub_commands;
    void commandCallback(const za_msgs::PosVelSetpointConstPtr& msg, 
                         const std::string& arm_id);

    bool initArm(hardware_interface::RobotHW* robot_hw, 
                 const std::string& arm_id,
                 const std::vector<std::string>& joint_names);
    void startingArm(ZaDataContainer& arm_data);
    void updateArm(ZaDataContainer& arm_data);
};

} // namespace hydra_controllers