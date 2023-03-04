#pragma once

#include <urdf/model.h>
#include <hydra_hw/model_base.h>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace hydra_gazebo {

class ModelKDL : public hydra_hw::ModelBase {
public:
    ModelKDL(const urdf::Model& model,
             const std::vector<std::string>& arm_ids,
             const std::vector<std::string>& tips,
             const std::string positioner_frame);
    
    /**
    * Gets the 6x7 Jacobian for the given frame, relative to that frame.
    *
    * The Jacobian is represented as a 6x7 matrix in column-major format.
    *
    * @param[in] frame The desired frame.
    * @param[in] robot_state State from which the pose should be calculated.
    *
    * @return Vectorized 6x7 Jacobian, column-major.
    */
    std::array<double, 42> positionerJacobian(const std::string& arm_id_,
                                              hydra::Frame frame,
                                              const std::array<double, 7>& q) const override;
private:
    static int segment(hydra::Frame frame);

    std::map<std::string, std::shared_ptr<KDL::Chain>> chains_; 
};

} // namespace hydra_gazebo