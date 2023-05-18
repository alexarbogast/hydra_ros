#include <hydra_control/positioner_state_controller.h>
#include <pluginlib/class_list_macros.h>


namespace hydra_control
{

bool PositionerStateController::init(hardware_interface::RobotHW* robot_hardware,
                                     ros::NodeHandle& root_node_handle,
                                     ros::NodeHandle& controller_node_handle) {
    positioner_state_interface_ = robot_hardware->get<hydra_hw::PositionerStateInterface>();
    if (positioner_state_interface_ == nullptr) {
        ROS_ERROR("PositionerStateController: Could not get positioner state interface from hardware");
        return false;
    }
    if (!controller_node_handle.getParam("arm_id", arm_id_)) {
        ROS_ERROR("ZaStateController: Could not get parameter arm_id");
        return false;
    }
    double publish_rate(30.0);
    if (!controller_node_handle.getParam("publish_rate", publish_rate)) {
        ROS_INFO_STREAM("ZaStateController: Did not find publish_rate. Using default "
                         << publish_rate << " [Hz].");
    }
    trigger_publish_ = robot_hw::TriggerRate(publish_rate);

    if (!controller_node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != 1) {
    ROS_ERROR(
        "PositionerStateController: Invalid or no joint_names parameters provided, "
        "currently only 1 DOF positioners are supported"
        "aborting controller init!");
    return false;
    }

    try {
        positioner_state_handle_ = std::make_unique<hydra_hw::PositionerStateHandle>(
            positioner_state_interface_->getHandle(arm_id_ + "_robot"));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("PositionerStateController: Exception getting positioner state handle: " << ex.what());
        return false;
    }

    publisher_joint_states_.init(controller_node_handle, "joint_states", 1);
    {
        std::lock_guard<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> lock(
            publisher_joint_states_);
        publisher_joint_states_.msg_.name.resize(joint_names_.size());
        publisher_joint_states_.msg_.position.resize(1);
        publisher_joint_states_.msg_.velocity.resize(1);
    }
    return true;
}

void PositionerStateController::update(const ros::Time& time, const ros::Duration& period)
{
    if (trigger_publish_())
    {
        positioner_state_ = positioner_state_handle_->getPositionerState();
        publishJointStates(time);
        sequence_number_++;
    }
}

void PositionerStateController::publishJointStates(const ros::Time& time)
{
    if (publisher_joint_states_.trylock()) 
    {

        publisher_joint_states_.msg_.name = joint_names_;
        publisher_joint_states_.msg_.position = {positioner_state_.q};
        publisher_joint_states_.msg_.velocity = {positioner_state_.dq};
    
        publisher_joint_states_.msg_.header.stamp = time;
        publisher_joint_states_.msg_.header.seq = sequence_number_;
        publisher_joint_states_.unlockAndPublish();
    }
}

} // namespace hydra_control

PLUGINLIB_EXPORT_CLASS(hydra_control::PositionerStateController, controller_interface::ControllerBase)