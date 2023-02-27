#include <hydra_controllers/hydra_controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace hydra_controllers {

const static std::string param_name = "joints"; 

bool HydraController::init(hardware_interface::RobotHW* robot_hw, 
                           ros::NodeHandle& node_handle) {
    // read ros parameters
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

    auto* velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr) {
        ROS_ERROR(
            "HydraController: Could not get Cartesian velocity interface from "
            "hardware");
        return false;
    }

    auto* model_interface = robot_hw->get<za_hw::ZaModelInterface>();
            if (model_interface == nullptr) {
            ROS_ERROR_STREAM("HydraController: Error getting model interface from hardware");
            return false;
    }

    auto* state_interface = robot_hw->get<za_hw::ZaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("HydraController: Could not get za state interface from hardware");
        return false;
    }

    joint_handles_.resize(arm_ids.size());
    model_handles_.reserve(arm_ids.size());
    state_handles_.reserve(arm_ids.size());
    for (unsigned int i = 0; i < arm_ids.size(); i++) {
        std::vector<std::string> joint_names;
        if (!node_handle.getParam(arm_ids[i] + "/" + param_name, joint_names)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << 
                "'(namespace: " << node_handle.getNamespace() << ")");
            return false; 
        }

        auto& joint_handle = joint_handles_[i];
        joint_handle.resize(joint_names.size());
        for (size_t j = 0; j < joint_names.size(); ++j) {
            try {
                joint_handle[j] = velocity_joint_interface_->getHandle(joint_names[j]);
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                    "HydraController: Exception getting joint handles: " << e.what());
                return false;
            }
        }

        try {
            model_handles_.emplace_back(std::make_unique<za_hw::ZaModelHandle>(
                                model_interface->getHandle(arm_ids[i] + "_model")));
        } catch (hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                "HydraController: Exception getting model handle from interface: " << e.what());
            return false;
        }

        try {
            state_handles_.emplace_back(std::make_unique<za_hw::ZaStateHandle>(
                                state_interface->getHandle(arm_ids[i] + "_robot")));

            // we must start in a known (non-singular) position
            // possibly pass this as a parameter
            std::array<double, 6> q_start = {0, 0.53, 0.47, 0, -1, 0};
            for (size_t j = 0; j < q_start.size(); j++) {
                if (std::abs(state_handles_[i]->getRobotState().q_d[j] - q_start[j]) > 0.1) {
                    ROS_ERROR_STREAM(
                        "HydraController: Robot is not in the expected starting position ");
                    return false;
                }
            }
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                "HydraController: Exception getting state handle: " << e.what());
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

}

void HydraController::update(const ros::Time&, const ros::Duration& period) {

}

void HydraController::stopping(const ros::Time&) {

}

void HydraController::taskpriorityParamCallback(za_controllers::taskpriority_paramConfig& config,
                                                uint32_t /*level*/) {
    Kp_ = config.translation_gain;
    Ko_ = config.rotation_gain;
    Kr_ = config.redundancy_gain;
}

} // namespace hydra_controllers

PLUGINLIB_EXPORT_CLASS(hydra_controllers::HydraController, controller_interface::ControllerBase)