#pragma once

#include <memory>
#include <hydra_hw/model_base.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <za_hw/za_state_interface.h>

namespace hydra_hw {

class HydraModelHandle {
public:
    HydraModelHandle() = delete;

    HydraModelHandle(const std::string& name,
                    hydra_hw::ModelBase& model,
                    std::map<std::string, std::shared_ptr<za_hw::ZaStateHandle>>& robot_state_map)
        : name_(name), model_(&model), robot_state_map_(robot_state_map) {}

    HydraModelHandle(const std::string& name,
                    hydra_hw::ModelBase& model,
                    std::map<std::string, std::shared_ptr<za_hw::ZaStateHandle>>&& robot_state_map)
        : name_(name), model_(&model), robot_state_map_(std::move(robot_state_map)) {}

    std::string getName() const noexcept { return name_; }

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
    std::array<double, 42> positionerJacobian() const {
        return model_->positionerJacobian();
    }

private:
    std::string name_;
    const hydra_hw::ModelBase* model_;
    std::map<std::string, std::shared_ptr<za_hw::ZaStateHandle>> robot_state_map_;
};

/**
 * Hardware interface to perform calculations using the dynamic model of a robot.
 */
class HydraModelInterface : public hardware_interface::HardwareResourceManager<HydraModelHandle> {
};

} // namespace hydra_hw