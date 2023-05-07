#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <za_msgs/PosVelSetpoint.h>
#include <ros/node_handle.h>

#include <hydra_controllers/hydra_paramConfig.h>
#include <hydra_controllers/robot_data_container.h>
#include <hydra_controllers/cached_controller_data.h>
#include <hydra_controllers/control_methods.h>
#include <hydra_controllers/SwitchCoordination.h>
#include <hydra_hw/hydra_model_interface.h>

#include <cartesian_trajectory_adapter/multi_trajectory_adapter.h>

namespace hydra_controllers {

class HydraController : public controller_interface::MultiInterfaceController<
                                    hardware_interface::VelocityJointInterface,
                                    za_hw::ZaModelInterface,
                                    za_hw::ZaStateInterface,
                                    hydra_hw::PositionerStateInterface,
                                    hydra_hw::HydraModelInterface>,
                        public cartesian_trajectory_controllers::MultiTrajectoryAdapter{
public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

private:
    ArmDataMap arms_data_;
    PositionerDataContainer positioner_data_;
    std::unique_ptr<hydra_hw::HydraModelHandle> model_handle_;

    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<hydra_controllers::hydra_paramConfig>>
        dynamic_server_posvel_param_;
    ros::NodeHandle dynamic_reconfigure_posvel_param_node_;
    ControllerParameters controller_params_;
    void taskpriorityParamCallback(hydra_controllers::hydra_paramConfig& config,
                             uint32_t level);

    ros::ServiceServer coordination_server_;
    bool serviceCallback(SwitchCoordination::Request& req,
                         SwitchCoordination::Response& resp);

    bool initPositioner(hardware_interface::RobotHW* robot_hw,
                        const std::string& positioner_id,
                        const std::vector<std::string>& joint_names);
    void updatePositioner(PositionerDataContainer& positioner_data,
                          CachedControllerData& controller_data);

    bool initArm(hardware_interface::RobotHW* robot_hw, 
                 const std::string& arm_id,
                 const std::vector<std::string>& joint_names);
    void startingArm(const std::string& arm_id,
                     ZaDataContainer& arm_data);
    void updateArm(ZaDataContainer& arm_data,
                   CachedModelData& model_cache,
                   double positioner_cmd);
};

} // namespace hydra_controllers