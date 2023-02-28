#include <hydra_controllers/control_methods.h>
#include <za_controllers/pseudo_inversion.h>

namespace hydra_controllers {

void taskPriorityControl(ZaDataContainer& arm_data,
                         const TPCControllerInfo& context) {
    za::RobotState robot_state = arm_data.state_handle_->getRobotState();
    
    Eigen::Affine3d pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

    std::array<double, 36> jacobian_array = 
        arm_data.model_handle_->getZeroJacobian(za::Frame::kEndEffector);    
    Eigen::Map<Eigen::Matrix<double, 6, 6>> jacobian(jacobian_array.data());
    Eigen::MatrixXd jacobian_pinv;
    za_controllers::pseudoInverse(jacobian, jacobian_pinv, false);

    /* ========= task tracking ========= */ 
    Eigen::Vector3d pose_error = context.Kp * 
        (arm_data.position_d_ - pose.translation());

    const auto& z_eef = pose.matrix().block<3, 1>(0, 2);
    Eigen::Vector3d orient_error = context.Ko * z_eef.cross(context.z_align);

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
    dp_redundancy = -context.Kr * jacobian * Jm;
    dp_redundancy.block<5, 1>(0, 0).setZero();

    Eigen::Matrix<double, 6, 1> dq_cmd = (jacobian_pinv * (dp_d + dp_redundancy));

    for (size_t i = 0; i < 6; ++i) {
        arm_data.joint_handles_[i].setCommand(dq_cmd(i));
    }
}

void coordinatedTaskPriorityControl(ZaDataContainer& arm_data,
                                    const CTPCControllerInfo& context) {

}

} // namespace hydra_controllers