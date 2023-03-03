#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <za_msgs/PosVelSetpoint.h>
#include <ros/node_handle.h>

#include <za_controllers/taskpriority_paramConfig.h>
#include <hydra_controllers/robot_data_container.h>
#include <hydra_controllers/cached_controller_data.h>

namespace hydra_controllers {

class HydraController : public controller_interface::MultiInterfaceController<
                                    hardware_interface::VelocityJointInterface,
                                    za_hw::ZaModelInterface,
                                    za_hw::ZaStateInterface,
                                    hydra_hw::PositionerStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

private:
    std::map<std::string, ZaDataContainer> arms_data_;
    PositionerDataContainer positioner_data_;

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

    bool initPositioner(hardware_interface::RobotHW* robot_hw,
                        const std::string& positioner_id,
                        const std::vector<std::string>& joint_names);
    void updatePositioner(PositionerDataContainer& positioner_data,
                          CachedControllerData& controller_data);

    bool initArm(hardware_interface::RobotHW* robot_hw, 
                 const std::string& arm_id,
                 const std::vector<std::string>& joint_names);
    void startingArm(ZaDataContainer& arm_data);
    void updateArm(ZaDataContainer& arm_data,
                   CachedModelData& model_cache);
};

} // namespace hydra_controllers