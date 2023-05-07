#include <hydra_controllers/hydra_controller.h>
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

    arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));
    return true;
}

bool HydraController::initPositioner(hardware_interface::RobotHW* robot_hw,
                                     const std::string& positioner_id,
                                     const std::vector<std::string>& joint_names
                                     ) {
    auto* state_interface = robot_hw->get<hydra_hw::PositionerStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM(
            "HydraController: Error getting positioner state interface from hardware");
        return false;
    }
    try {
        positioner_data_.state_handle_ = 
            std::make_unique<hydra_hw::PositionerStateHandle>(
                        state_interface->getHandle(positioner_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "HydraController: Exception getting positioner state handle from "
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
    for (size_t i = 0; i < joint_names.size(); ++i) {
        try {
            positioner_data_.joint_handles_.push_back(
                velocity_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "HydraController: Exception getting positioner joint handles: "
            << ex.what());
        return false;
        }
    }
    return true;
}

bool HydraController::init(hardware_interface::RobotHW* robot_hw, 
                           ros::NodeHandle& node_handle) {
    // read global parameters
    std::vector<std::string> arm_ids;
    if (!node_handle.getParam("arm_ids", arm_ids)) {
        ROS_ERROR("HydraController: Could not get parameter 'arm_ids'");
        return false;
    }
    std::string positioner_id;
    if (!node_handle.getParam("positioner_id", positioner_id)) {
        ROS_ERROR("HydraController: Could not get parameter 'positioner_id'");
        return false;
    }

    if (not node_handle.getParam("Kp", controller_params_.Kp) or 
        not node_handle.getParam("Ko", controller_params_.Ko) or
        not node_handle.getParam("Kr", controller_params_.Kr) or
        not node_handle.getParam("Kpos", controller_params_.Kpos)) {
        ROS_ERROR("Missing controller gains 'Kp' or 'Ko' or 'Kr' or 'Kpos'");
        return false;
    }
    std::vector<double> z_align;
    if (not node_handle.getParam("z_align", z_align)) {
        ROS_ERROR("Missing z alignment axis 'z_align'");
        return false;
    }
    controller_params_.z_align = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>
        (z_align.data(), z_align.size());

    // initialize coordination server
    coordination_server_ = node_handle.advertiseService("switch_coordination", &HydraController::serviceCallback, this);

    // initialize manipulators
    for (const auto& arm_id : arm_ids) {         
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

    // initialize positioner
    std::vector<std::string> joint_names;
    if (!node_handle.getParam(positioner_id + "/" + param_name, joint_names)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << 
            "'(namespace: " << node_handle.getNamespace() << ")");
        return false; 
    }
    if (!initPositioner(robot_hw, positioner_id, joint_names)){
        return false;
    }

    auto* hydra_model_interface_ = robot_hw->get<hydra_hw::HydraModelInterface>();
    if (hydra_model_interface_ == nullptr) {
        ROS_ERROR(
            "HydraController: Could not get Hydra model interface from "
            "hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<hydra_hw::HydraModelHandle>(
            hydra_model_interface_->getHandle("hydra_model"));
    } catch (hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
            "HydraController: Exception getting model handle from interface: " << e.what());
        return false;
    }

    dynamic_reconfigure_posvel_param_node_ =
        ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_taskpriority_param_node");
    
    dynamic_server_posvel_param_ = std::make_unique<
        dynamic_reconfigure::Server<hydra_controllers::hydra_paramConfig>>(
            dynamic_reconfigure_posvel_param_node_);
    
    dynamic_server_posvel_param_->setCallback(
        boost::bind(&HydraController::taskpriorityParamCallback, this, _1, _2));

    /* Initialize trajectory adapters */
    std::vector<cartesian_trajectory_controllers::StateHandle> setpoints;
    setpoints.reserve(arms_data_.size());
    std::transform(arms_data_.begin(), arms_data_.end(), std::back_inserter(setpoints),
                   [](const auto& pair) { return &pair.second.setpoint_; });
    MultiTrajectoryAdapter::init(node_handle, arm_ids, setpoints);
    return true;
}

void HydraController::starting(const ros::Time&) {
    for (auto& arm_data : arms_data_) {
        startingArm(arm_data.first, arm_data.second);
    }
}

void HydraController::startingArm(const std::string& arm_id,
                                  ZaDataContainer& arm_data) {
    switch (arm_data.mode_) {
        case ControlMode::TaskPriorityControl: {
            za::RobotState initial_state = arm_data.state_handle_->getRobotState();
            
            Eigen::Affine3d initial_transformation(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            arm_data.setpoint_.p = initial_transformation.translation();
            break;
        }
        case ControlMode::CoordinatedTaskPriorityControl: {
            Eigen::Affine3d initial_transformation(
                Eigen::Matrix4d::Map(model_handle_->
                    getPose(arm_id, hydra::Frame::kEndEffector).data()));
            arm_data.setpoint_.p = initial_transformation.translation();
            break;
        }
        default:
            break;
    }
}

void HydraController::update(const ros::Time&, const ros::Duration& period) {
    CachedControllerData cache;
    cacheControllerData(arms_data_, model_handle_, cache);
    
    updatePositioner(positioner_data_, cache);
    for (auto& arm_data : arms_data_) {
        // ================== trajectory generation ===================
        auto& traj_adap = *(adapters_.at(arm_data.first));
        if (traj_adap.isActive() && !traj_adap.isDone()) {
            traj_adap.sample(period.toSec(), arm_data.second.setpoint_);
        }
        
        // ================== control ================================
        updateArm(arm_data.second, cache.model_cache[arm_data.first],
                  cache.positioner_command_); 
    }
}

void HydraController::updateArm(ZaDataContainer& arm_data,
                                CachedModelData& model_cache,
                                double positioner_cmd) {
    switch (arm_data.mode_) {
        case ControlMode::TaskPriorityControl: {
            taskPriorityControl(arm_data, model_cache, controller_params_);
            break;
        }
        case ControlMode::CoordinatedTaskPriorityControl: {
            coordinatedTaskPriorityControl(arm_data, model_cache, 
                                        controller_params_, positioner_cmd);
            break;
        }
        default:
            ROS_ERROR_STREAM("Unknown control mode: " << (int)arm_data.mode_);
            break;
    }
}

void HydraController::updatePositioner(PositionerDataContainer& positioner_data,
                                       CachedControllerData& controller_data) {
    positionerControl(positioner_data, arms_data_,
                      controller_data, controller_params_);
}

void HydraController::stopping(const ros::Time&) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

bool HydraController::serviceCallback(SwitchCoordination::Request& req,
                                      SwitchCoordination::Response& resp) {
    auto& arm_data = arms_data_[req.arm_id];
    if (req.coordinated) {
        arm_data.mode_ = ControlMode::CoordinatedTaskPriorityControl;
        Eigen::Affine3d initial_transformation(
                Eigen::Matrix4d::Map(model_handle_->
                    getPose(req.arm_id, hydra::Frame::kEndEffector).data()));
        arm_data.setpoint_ = cartesian_controllers::CartesianState();
        arm_data.setpoint_.p = initial_transformation.translation();
    } 
    else {
        arm_data.mode_ = ControlMode::TaskPriorityControl;
        za::RobotState robot_state = arm_data.state_handle_->getRobotState();

        Eigen::Affine3d initial_transformation(
                Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        arm_data.setpoint_ = cartesian_controllers::CartesianState();
        arm_data.setpoint_.p = initial_transformation.translation();
    }
    resp.success = true;
    return true;
}

void HydraController::taskpriorityParamCallback(hydra_controllers::hydra_paramConfig& config,
                                                uint32_t /*level*/) {
    controller_params_.Kp = config.translation_gain;
    controller_params_.Ko = config.rotation_gain;
    controller_params_.Kr = config.redundancy_gain;
    controller_params_.Kpos = config.positioner_gain;
}

} // namespace hydra_controllers

PLUGINLIB_EXPORT_CLASS(hydra_controllers::HydraController, controller_interface::ControllerBase)