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
             const std::vector<std::string>& tips,
             const std::string positioner_frame);
private:
    std::vector<std::unique_ptr<KDL::Chain>> chains_; 
};

} // namespace hydra_gazebo