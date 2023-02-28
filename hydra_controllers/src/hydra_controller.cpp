#include <hydra_controllers/hydra_controller.h>
#include <za_controllers/pseudo_inversion.h>
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
    za::RobotState robot_state = arm_data.state_handle_->getRobotState();
    
    Eigen::Affine3d pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

    std::array<double, 36> jacobian_array = 
        arm_data.model_handle_->getZeroJacobian(za::Frame::kEndEffector);    
    Eigen::Map<Eigen::Matrix<double, 6, 6>> jacobian(jacobian_array.data());
    Eigen::MatrixXd jacobian_pinv;
    za_controllers::pseudoInverse(jacobian, jacobian_pinv, false);

    /* ========= task tracking ========= */ 
    Eigen::Vector3d pose_error = Kp_ * (arm_data.position_d_ - pose.translation());

    const auto& z_eef = pose.matrix().block<3, 1>(0, 2);
    Eigen::Vector3d orient_error = Ko_ * z_eef.cross(z_align_);

    Eigen::Matrix<double, 6, 1> error(6);
    error << pose_error, orient_error;

    Eigen::Matrix<double, 6, 1> dp_d(arm_data.twist_setpoint_);
    dp_d += error;

    /* ====== redundancy resolution ====== */
    Eigen::MatrixXd null_project = Eigen::Matrix<double, 6, 6>::Identity() 
        - jacobian_pinv * jacobian;
    Eigen::Matrix<double, 6, 1> dp_redundancy = Eigen::Matrix<double, 6, 1>::Zero();

    std::array<double, 216> hessian_array =
        arm_data.model_handle_->getZeroHessian(za::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 36, 6>> hessian(hessian_array.data());

    Eigen::Matrix<double, 6, 6> J_Jt = jacobian * jacobian.transpose();
    double det_J_Jt = J_Jt.determinant();
    auto inv_J_Jt = J_Jt.inverse();

    Eigen::Matrix<double, 36, 1> vec_inv_J_Jt;
    Eigen::MatrixXd::Map(&vec_inv_J_Jt[0], 6, 6) = inv_J_Jt;
    
    // find manipulability Jacobian
    Eigen::Matrix<double, 6, 1> Jm;
    Jm.setZero();
    for (int i = 0; i < 6; i++) {
        const auto& Hi = hessian.block<6, 6>(i * 6, 0);
        Eigen::Matrix<double, 36, 1> vec_J_HiT;
        Eigen::MatrixXd::Map(&vec_J_HiT[0], 6, 6) = jacobian * Hi.transpose();

        Jm(i, 0) = sqrt(abs(det_J_Jt)) * vec_J_HiT.dot(vec_inv_J_Jt);
    }

    // use redundant axis (z-rotation) to drive the posture to maximum manipulability
    dp_redundancy = -Kr_ * jacobian * Jm;
    dp_redundancy.block<5, 1>(0, 0).setZero();

    Eigen::Matrix<double, 6, 1> dq_cmd = (jacobian_pinv * (dp_d + dp_redundancy));

    for (size_t i = 0; i < 6; ++i) {
        arm_data.joint_handles_[i].setCommand(dq_cmd(i));
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