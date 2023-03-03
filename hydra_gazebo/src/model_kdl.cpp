#include <hydra_gazebo/model_kdl.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace hydra_gazebo {

ModelKDL::ModelKDL(const urdf::Model& model,
                   const std::vector<std::string>& tips,
                   const std::string positioner_frame) {
    KDL::Tree tree;
    if (not kdl_parser::treeFromUrdfModel(model, tree)) {
        throw std::invalid_argument("Cannot construct KDL tree from URDF");
    }

    for (const auto& tip : tips) {
        this->chains_.emplace_back(std::make_unique<KDL::Chain>());
        if (not tree.getChain(positioner_frame, tip, *(this->chains_.back()))) {
            throw std::invalid_argument("Cannot find chain within URDF tree from root '" + 
            positioner_frame + "' to tip '" + tip + "'. Do these links exist?");
        }
    }
}

} // namespace hydra_gazebo