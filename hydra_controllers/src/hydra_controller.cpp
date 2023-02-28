#include <hydra_controllers/hydra_controller.h>
#include <hydra_controllers/control_methods.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace hydra_controllers {

const static std::string param_name = "joints"; 

bool HydraController::initArm(hardware_interface::RobotHW* robot_hw,
                        const std::string& arm_id,
                        const std::vector<std::string>& joint_names
                        ) {
    ZaDataContainer arm_data;
    auto* model_interface = robot_hw->get<za_hw::ZaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM(
            "HydraController: Error getting model interface from hardware");
        return false;
    }
    try {
        arm_data.model_handle_ = std::make_unique<za_hw::ZaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "HydraController: Exception getting model handle from "
            "interface: "
            << ex.what());
        return false;
    }

    auto* state_interface = robot_hw->get<za_hw::ZaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM(
            "HydraController: Error getting state interface from hardware");
        return false;
    }
    try {
        arm_data.state_handle_ = std::make_unique<za_hw::ZaStateHandle>(
                        state_interface->getHandle(arm_id + "_robot"));

        // we must start in a known (non-singular) position
        // possibly pass this as a parameter
        std::array<double, 6> q_start = {0, 0.53, 0.47, 0, -1, 0};
        for (size_t j = 0; j < q_start.size(); j++) {
            if (std::abs(arm_data.state_handle_->getRobotState().q_d[j] - q_start[j]) > 0.1) {
                ROS_ERROR_STREAM(
                    "HydraController: Robot is not in the expected starting position ");
                return false;
            }
        }
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "HydraController: Exception getting state handle from "
            "interface: "
            << ex.what());
        return false;
    }

    auto* velocity_joint_interface = robot_hw->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface == nullptr) {
        ROS_ERROR_STREAM(
            "HydraController: Error getting velocity joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        try {
        arm_data.joint_handles_.push_back(velocity_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "HydraController: Exception getting joint handles: "
            << ex.what());
        return false;
        }
    }

    arm_data.position_d_.setZero();
    arm_data.twist_setpoint_.setZero();
    arm_data.pose_twist_setpoint_mutex_ = std::make_unique<std::mutex>();

    arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));
    return true;
}   

bool HydraController::init(hardware_interface::RobotHW* robot_hw, 
                           ros::NodeHandle& node_handle) {
    std::vector<std::string> arm_ids;
    if (!node_handle.getParam("arm_ids", arm_ids)) {
        ROS_ERROR("HydraController: Could not get parameter 'arm_ids'");
        return false;
    }

    if (not node_handle.getParam("Kp", Kp_) or 
        not node_handle.getParam("Ko", Ko_) or
        not node_handle.getParam("Kr", Kr_)) {
        ROS_ERROR("Missing controller gains 'Kp' or 'Ko' or 'Kr'");
        return false;
    }
    std::vector<double> z_align;
    if (not node_handle.getParam("z_align", z_align)) {
        ROS_ERROR("Missing z alignment axis 'z_align'");
        return false;
    }
    z_align_ = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>
        (z_align.data(), z_align.size());

    for (const auto& arm_id : arm_ids) {
        boost::function<void(const za_msgs::PosVelSetpointConstPtr&)> callback = 
            boost::bind(&HydraController::commandCallback, this, _1, arm_id);
        
        ros::SubscribeOptions subscriber_options;
        subscriber_options.init(arm_id + "/command", 1, callback);
        subscriber_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
        setpoints_subs_.emplace_back(node_handle.subscribe(subscriber_options));
         
        std::vector<std::string> joint_names;
        if (!node_handle.getParam(arm_id + "/" + param_name, joint_names)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << 
                "'(namespace: " << node_handle.getNamespace() << ")");
            return false; 
        }

        if (!initArm(robot_hw, arm_id, joint_names)) {
            return false;
        }
    }

    dynamic_reconfigure_posvel_param_node_ =
        ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_taskpriority_param_node");
    
    dynamic_server_posvel_param_ = std::make_unique<
        dynamic_reconfigure::Server<za_controllers::taskpriority_paramConfig>>(
            dynamic_reconfigure_posvel_param_node_);
    
    dynamic_server_posvel_param_->setCallback(
        boost::bind(&HydraController::taskpriorityParamCallback, this, _1, _2));

    return true;
}

void HydraController::starting(const ros::Time&) {
    for (auto& arm_data : arms_data_) {
        startingArm(arm_data.second);
    }
}

void HydraController::startingArm(ZaDataContainer& arm_data) {
    za::RobotState initial_state = arm_data.state_handle_->getRobotState();

    Eigen::Affine3d initial_transformation(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    arm_data.position_d_ = initial_transformation.translation();
}

void HydraController::update(const ros::Time&, const ros::Duration& period) {
    for (auto& arm_data : arms_data_) {
        updateArm(arm_data.second); 
    }
}

void HydraController::updateArm(ZaDataContainer& arm_data) {
    switch (arm_data.mode_) {
        case ControlMode::TaskPriorityControl: {
            TPCControllerInfo info(Kp_, Ko_, Kr_, z_align_);
            taskPriorityControl(arm_data, info);
            break; 
        }
        case ControlMode::CoordinatedTaskPriorityControl: {
            CTPCControllerInfo info;
            coordinatedTaskPriorityControl(arm_data, info);
            break;
        }
        default:
            ROS_ERROR_STREAM("Unknown control mode: " << (int)arm_data.mode_);
            break;
    }
}

void HydraController::stopping(const ros::Time&) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void HydraController::taskpriorityParamCallback(za_controllers::taskpriority_paramConfig& config,
                                                uint32_t /*level*/) {
    Kp_ = config.translation_gain;
    Ko_ = config.rotation_gain;
    Kr_ = config.redundancy_gain;
}

void HydraController::commandCallback(const za_msgs::PosVelSetpointConstPtr& msg, 
                                      const std::string& arm_id) {
    auto& arm_data = arms_data_[arm_id];
    std::lock_guard<std::mutex> twist_setpoint_mutex_lock(*arm_data.pose_twist_setpoint_mutex_);

    arm_data.position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    arm_data.twist_setpoint_ << 
        msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
        msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}

} // namespace hydra_controllers

PLUGINLIB_EXPORT_CLASS(hydra_controllers::HydraController, controller_interface::ControllerBase)