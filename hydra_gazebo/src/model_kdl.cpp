#include <hydra_gazebo/model_kdl.h>
#include <za_gazebo/model_kdl.h>

#include <za_hw/model_base.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace hydra_gazebo {

int ModelKDL::segment(hydra::Frame frame) {
  // clang-format off
  switch (frame) {
    case hydra::Frame::kPositioner:  return 0;
    case hydra::Frame::kJoint1:      return 8;
    case hydra::Frame::kJoint2:      return 9;
    case hydra::Frame::kJoint3:      return 10;
    case hydra::Frame::kJoint4:      return 11;
    case hydra::Frame::kJoint5:      return 12;
    case hydra::Frame::kJoint6:      return 13;
    case hydra::Frame::kFlange:      return 14;
    default: return -1;
  }
  // clang-format on
}

ModelKDL::ModelKDL(const urdf::Model& model,
                   const std::vector<std::string>& arm_ids,
                   const std::vector<std::string>& tips,
                   const std::string positioner_frame) {
    KDL::Tree tree;
    if (not kdl_parser::treeFromUrdfModel(model, tree)) {
        throw std::invalid_argument("Cannot construct KDL tree from URDF");
    }

    if (arm_ids.size() != tips.size()) {
        ROS_ERROR_STREAM("Mismatch in size of arm_ids and tip frames"
        << arm_ids.size() << " vs " << tips.size());
    }

    for (size_t i = 0; i < tips.size(); i++) {
        auto chain = std::make_shared<KDL::Chain>();
        if (not tree.getChain(positioner_frame, tips[i], *chain)) {
            throw std::invalid_argument("Cannot find chain within URDF tree from root '" + 
            positioner_frame + "' to tip '" + tips[i] + "'. Do these links exist?");
        }
        this->chains_.emplace(std::make_pair(arm_ids[i], chain));
    }

    //for (const auto& chain : this->chains_) {
    //    std::cout << "Expected positioner, got : " <<  chain.second->getSegment(segment(hydra::Frame::kPositioner)).getName() << std::endl;
    //    std::cout << "Expected joint1, got : " <<  chain.second->getSegment(segment(hydra::Frame::kJoint1)).getName() << std::endl;
    //    std::cout << "Expected joint2, got : " <<  chain.second->getSegment(segment(hydra::Frame::kJoint2)).getName() << std::endl;
    //    std::cout << "Expected joint3, got : " <<  chain.second->getSegment(segment(hydra::Frame::kJoint3)).getName() << std::endl;
    //    std::cout << "Expected joint4, got : " <<  chain.second->getSegment(segment(hydra::Frame::kJoint4)).getName() << std::endl;
    //    std::cout << "Expected joint5, got : " <<  chain.second->getSegment(segment(hydra::Frame::kJoint5)).getName() << std::endl;
//
    //    for (const auto& seg : chain.second->segments) {
    //        std::cout << seg.getName() << std::endl;
    //    }
    //    std::cout << std::endl;
    //}
}

std::array<double, 42> ModelKDL::positionerJacobian(const std::string& arm_id_,
                                                    hydra::Frame frame,
                                                    const std::array<double, 7>& q) 
                                                    const {
    KDL::JntArray kq;
    KDL::Jacobian J(7);  // NOLINT(readability-identifier-naming)
    kq.data = Eigen::Matrix<double, 7, 1>(q.data());

    std::cout << arm_id_ << std::endl;
    
    KDL::ChainJntToJacSolver solver(*(this->chains_.at(arm_id_)));
    int segmentNr = frame == hydra::Frame::kEndEffector ?
                             this->chains_.at(arm_id_)->getNrOfSegments() : segment(frame); 

    int error = solver.JntToJac(kq, J, segmentNr);
    if (error != KDL::SolverI::E_NOERROR) {
        throw std::logic_error("KDL zero jacobian calculation failed with error: ");
    }

    std::array<double, 42> result; 
    Eigen::MatrixXd::Map(&result[0], 6, 7) = J.data;
    return result;
}

} // namespace hydra_gazebo