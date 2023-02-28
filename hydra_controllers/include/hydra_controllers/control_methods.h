# pragma once

#include <hydra_controllers/hydra_controller.h>

namespace hydra_controllers {

struct TPCControllerInfo {
    TPCControllerInfo(double Kp, double Ko, double Kr, 
                      const Eigen::Vector3d& z_align) 
        : Kp(Kp), Ko(Ko), Kr(Kr), z_align(z_align)
    {} 

    double Kp, Ko, Kr;
    Eigen::Vector3d z_align;
};

struct CTPCControllerInfo {
    double Kp, Ko, Kr;
    Eigen::Vector3d z_align;
};

void taskPriorityControl(ZaDataContainer& arm_data,
                         const TPCControllerInfo& context);
void coordinatedTaskPriorityControl(ZaDataContainer& arm_data,
                                    const CTPCControllerInfo& context);

} // namespace hydra_controllers