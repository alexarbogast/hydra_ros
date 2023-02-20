#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <hydra_hw/positioner_state_interface.h>
#include <za_hw/trigger_rate.h>
#include <sensor_msgs/JointState.h>


namespace hydra_control 
{

class PositionerStateController
    : public controller_interface::MultiInterfaceController<hydra_hw::PositionerStateInterface> {
public:
    PositionerStateController() = default;

    /**
    * Initializes the controller with interfaces and publishers.
    *
    * @param[in] robot_hardware RobotHW instance to get a franka_hw::FrankaStateInterface from.
    * @param[in] root_node_handle Node handle in the controller_manager namespace.
    * @param[in] controller_node_handle Node handle in the controller namespace.
    */
    bool init(hardware_interface::RobotHW* robot_hardware,
              ros::NodeHandle& root_node_handle,
              ros::NodeHandle& controller_node_handle) override;

    /**
   * Reads the current robot state from the franka_hw::FrankaStateInterface and publishes it.
   *
   * @param[in] time Current ROS time.
   * @param[in] period Time since the last update.
   */
   void update(const ros::Time& time, const ros::Duration& period) override;

private:
    void publishJointStates(const ros::Time& time);

    std::string arm_id_;

    hydra_hw::PositionerStateInterface* positioner_state_interface_{};
    std::unique_ptr<hydra_hw::PositionerStateHandle> positioner_state_handle_;
    
    realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher_joint_states_;

    robot_hw::TriggerRate trigger_publish_;
    hydra::PositionerState positioner_state_;
    uint64_t sequence_number_ = 0;
    std::vector<std::string> joint_names_;
};
 
} // namespace hydra_control