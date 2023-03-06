#include <hydra_controllers/control_methods.h>
#include <za_controllers/pseudo_inversion.h>

namespace hydra_controllers {

void taskPriorityControl(ZaDataContainer& arm_data,
                         CachedModelData& input,
                         const TPCControllerParameters& context) {
    /* ========= task tracking ========= */ 
    Eigen::Vector3d pose_error = context.Kp * 
        (arm_data.position_d_ - input.pose.translation());

    const auto& z_eef = input.pose.matrix().block<3, 1>(0, 2);
    Eigen::Vector3d orient_error = context.Ko * z_eef.cross(context.z_align);
    Eigen::Matrix<double, 6, 1> error(6);
    error << pose_error, orient_error;

    Eigen::Matrix<double, 6, 1> dp_d(arm_data.twist_setpoint_);
    dp_d += error;

    /* ====== redundancy resolution ====== */
    Eigen::Matrix<double, 6, 1> dp_redundancy = Eigen::Matrix<double, 6, 1>::Zero();
    dp_redundancy = -context.Kr * input.Jo * input.Jm;
    dp_redundancy.block<5, 1>(0, 0).setZero();

    Eigen::Matrix<double, 6, 1> dq_cmd = (input.Jo_pinv * (dp_d + dp_redundancy));

    for (size_t i = 0; i < 6; ++i) {
        arm_data.joint_handles_[i].setCommand(dq_cmd(i));
    }
}

void coordinatedTaskPriorityControl(ZaDataContainer& arm_data,
                                    CachedModelData& input,
                                    const CTPCControllerParameters& context) {
    /* ========= task tracking ========= */ 
    Eigen::Vector3d pose_error = context.Kp *
        (arm_data.position_d_ - input.pose_p.translation());

    const auto& z_eef = input.pose.matrix().block<3, 1>(0, 2);
    Eigen::Vector3d orient_error = context.Ko * z_eef.cross(context.z_align);
    Eigen::Matrix<double, 6, 1> error(6);
    error << pose_error, orient_error;

    Eigen::Matrix<double, 6, 1> dp_d(arm_data.twist_setpoint_);
    dp_d += error;

    /* ====== redundancy resolution ====== */
    Eigen::Matrix<double, 6, 1> dp_redundancy = Eigen::Matrix<double, 6, 1>::Zero();
    dp_redundancy = -context.Kr * input.Jo * input.Jm;
    dp_redundancy.block<5, 1>(0, 0).setZero();

    /* ===== coordinated motion control =====*/
    Eigen::Matrix<double, 6, 6> J_rob = input.Jp.block<6, 6>(0, 1);
    J_rob.block<3, 6>(3, 0) = input.Jo.block<3, 6>(3, 0);
    Eigen::Matrix<double, 6, 1> J_pos = input.Jp.block<6, 1>(0, 0);
    J_pos.block<3, 1>(3, 0).setZero();

    Eigen::MatrixXd J_rob_pinv;
    za_controllers::pseudoInverse(J_rob, J_rob_pinv); 

    double pos_vel = 0.0;
    
    Eigen::Matrix<double, 6, 1> dq_cmd = J_rob_pinv * ((dp_d + dp_redundancy) - (J_pos * pos_vel));
    for (size_t i = 0; i < 6; ++i) {
        arm_data.joint_handles_[i].setCommand(dq_cmd(i));
    }
}

void positionerControl(PositionerDataContainer& positioner_data,
                       CachedControllerData& controller_data) {
    // find dm/dqp = (dm1/dq1)*(dq1/dqp) + ... + (dmN/dqN)*(dqN/dqp)
    double dqr_cmd = 0.0;
    positioner_data.joint_handles_[0].setCommand(dqr_cmd); 
    controller_data.positioner_command_ = dqr_cmd;
}

/*
 * This function stores data that is costly to compute to avoid duplicate
 * calculations in the robot and positioner controllers
 */
void cacheControllerData(std::map<std::string, ZaDataContainer>& arms_data,
                         const std::unique_ptr<hydra_hw::HydraModelHandle>& model_handle,
                         CachedControllerData& data) {
    using namespace Eigen;
    for (auto& arm : arms_data) {
        CachedModelData model_data;
        
        // find pose in base frame
        za::RobotState robot_state = arm.second.state_handle_->getRobotState();
        model_data.pose = Matrix4d::Map(robot_state.O_T_EE.data());

        // find the base frame jacobian
        model_data.Jo.setZero();
        std::array<double, 36> jacobian_array = 
            arm.second.model_handle_->getZeroJacobian(za::Frame::kEndEffector);  
        Map<Eigen::Matrix<double, 6, 6>> jacobian(jacobian_array.data());
        model_data.Jo = Map<Eigen::Matrix<double, 6, 6>>(jacobian_array.data());
        za_controllers::pseudoInverse(jacobian, model_data.Jo_pinv, false);

        // find the gradient of the manipulability
        model_data.Jm.setZero();
        std::array<double, 216> hessian_array =
        arm.second.model_handle_->getZeroHessian(za::Frame::kEndEffector);
        Map<Matrix<double, 36, 6>> hessian(hessian_array.data());

        Matrix<double, 6, 6> J_Jt = jacobian * jacobian.transpose();
        double det_J_Jt = J_Jt.determinant();
        auto inv_J_Jt = J_Jt.inverse();
        
        Matrix<double, 36, 1> vec_inv_J_Jt;
        MatrixXd::Map(&vec_inv_J_Jt[0], 6, 6) = inv_J_Jt;

        // find manipulability Jacobian
        for (int i = 0; i < 6; i++) {
            const auto& Hi = hessian.block<6, 6>(i * 6, 0);
            Matrix<double, 36, 1> vec_J_HiT;
            MatrixXd::Map(&vec_J_HiT[0], 6, 6) = jacobian * Hi.transpose();

            model_data.Jm(i, 0) = sqrt(abs(det_J_Jt)) * vec_J_HiT.dot(vec_inv_J_Jt);
        }

        if (arm.second.mode_ == ControlMode::CoordinatedTaskPriorityControl)
        {
            std::array<double, 42> pos_jacobian_array = 
                model_handle->positionerJacobian(arm.first, hydra::Frame::kEndEffector);
            model_data.Jp = Map<Eigen::Matrix<double, 6, 7>>(pos_jacobian_array.data());
        
            std::array<double, 16> pose_p_array = 
                model_handle->getPose(arm.first, hydra::Frame::kEndEffector);
            model_data.pose_p = Map<Eigen::Matrix<double, 4, 4>>(pose_p_array.data());
        }
        
        data.model_cache.emplace(std::make_pair(arm.first, std::move(model_data)));
    }
}

} // namespace hydra_controllers