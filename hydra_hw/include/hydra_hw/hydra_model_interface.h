#pragma once

#include <memory>
#include <hydra_hw/model_base.h>
#include <hydra_hw/positioner_state_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <za_hw/za_state_interface.h>

namespace hydra_hw {

class HydraModelHandle {
public:
    HydraModelHandle() = delete;

    HydraModelHandle(const std::string& name,
                    hydra_hw::ModelBase& model,
                    std::map<std::string, std::shared_ptr<za_hw::ZaStateHandle>>& robot_state_map,
                    std::shared_ptr<hydra_hw::PositionerStateHandle> pos_state_handle)
        : name_(name), model_(&model), robot_state_map_(robot_state_map), 
          pos_state_handle_(pos_state_handle) {}


    std::string getName() const noexcept { return name_; }

    /**
     * Gets the 4x4 pose matrix for the given frame in positioner, calculated from the current
     * robot state.
     *
     * The pose is represented as a 4x4 matrix in column-major format.
     *
     * @param[in] frame The desired frame.
     *
     * @return Vectorized 4x4 pose matrix, column-major.
     *
     * @see franka::Model::pose
     */
    std::array<double, 16> getPose(const std::string& arm_id, 
                                   const hydra::Frame& frame) const {
        std::array<double, 7> q = combinedState(arm_id);
        return model_->pose(arm_id, frame, q);
    }

    /**
     * Gets the 4x4 pose matrix for the given frame in base frame, calculated from the given
     * robot state.
     *
     * The pose is represented as a 4x4 matrix in column-major format.
     *
     * @param[in] frame The desired frame.
     * @param[in] q Joint position. Unit: \f$[rad]\f$.
     *
     * @return Vectorized 4x4 pose matrix, column-major.
     *
     * @see franka::Model::pose
     */
    std::array<double, 16> getPose(
        const std::string& arm_id,
        const hydra::Frame& frame,
        const std::array<double, 7>& q)  // NOLINT (readability-identifier-naming)
        const {
        return model_->pose(arm_id, frame, q);
    }

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
    std::array<double, 42> positionerJacobian(const std::string& arm_id,
                                              hydra::Frame frame,
                                              const std::array<double, 7>& q) const {
        return model_->positionerJacobian(arm_id, frame, q);
    }

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
    std::array<double, 42> positionerJacobian(const std::string& arm_id,
                                              hydra::Frame frame) const {
        std::array<double, 7> q = combinedState(arm_id);
        return model_->positionerJacobian(arm_id, frame, q);
    }
private:
    inline std::array<double, 7> combinedState(const std::string& arm_id) const {
        auto& robot_state = robot_state_map_.at(arm_id)->getRobotState();
        std::array<double, 7> q;
        std::copy(std::begin(robot_state.q), std::end(robot_state.q), std::next(std::begin(q)));
        q[0] = pos_state_handle_->getPositionerState().q;
        return q;
    }

    std::string name_;
    const hydra_hw::ModelBase* model_;
    std::map<std::string, std::shared_ptr<za_hw::ZaStateHandle>> robot_state_map_;
    std::shared_ptr<const hydra_hw::PositionerStateHandle> pos_state_handle_;
};

/**
 * Hardware interface to perform calculations using the dynamic model of a robot.
 */
class HydraModelInterface : public hardware_interface::HardwareResourceManager<HydraModelHandle> {
};

} // namespace hydra_hw