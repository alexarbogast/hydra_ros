#pragma once

#include <Eigen/Dense>
#include <map>

namespace hydra_controllers {
/*
 * Task Priority Control Parameters
 *
 * used to relay current gains to control loop
 */
struct TPCControllerParameters {
    TPCControllerParameters() = default;
    TPCControllerParameters(double Kp, double Ko, double Kr, 
                      const Eigen::Vector3d& z_align) 
        : Kp(Kp), Ko(Ko), Kr(Kr), z_align(z_align)
    {} 

    double Kp, Ko, Kr;
    Eigen::Vector3d z_align;
};

/*
 * Coordinated Task Priority Control Parameters
 *
 * used to relay current gains to control loop
 */
struct CTPCControllerParameters {
    CTPCControllerParameters() = default;
    CTPCControllerParameters(double Kp, double Ko, double Kr, 
                      const Eigen::Vector3d& z_align) 
        : Kp(Kp), Ko(Ko), Kr(Kr), z_align(z_align)
    {} 

    double Kp, Ko, Kr;
    Eigen::Vector3d z_align;
    double positioner_cmd;
};

/*
 * Cached Model Data
 * 
 * This function stores data that is costly to compute to avoid duplicate
 * calculations in the robot and positioner controllers
 */
struct CachedModelData {
    /* Pose in baseframe */
    Eigen::Affine3d pose;

    /* Base frame jacobian */
    Eigen::Matrix<double, 6, 6> Jo;

    /* Psuedo inverse of base frame jacobian */
    Eigen::MatrixXd Jo_pinv;

    /* Manipulability jacobian */
    Eigen::Matrix<double, 6, 1> Jm;

    /* ========== coordinated tpc data =============*/
    /* Positioner jacobian */
    Eigen::Matrix<double, 6, 7> Jp;
    
    /* Pose in positioner frame */
    Eigen::Affine3d pose_p;
};

/*
 * Cached Controller Data
 * 
 * A map to the cached model data and other info necessary for control loop 
 */
struct CachedControllerData {
    std::map<std::string, CachedModelData> model_cache;

    double positioner_command_;
};

} // namespace hydra_controllers