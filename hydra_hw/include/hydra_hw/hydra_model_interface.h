#pragma once

#include <hydra_hw/model_base.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hydra_hw {

class HydraModelHandle {
public:
    HydraModelHandle() = delete;

    HydraModelHandle(const std::string& name,
                    hydra_hw::ModelBase& model)
        : name_(name), model_(&model) {}

    std::string getName() const noexcept {return name_; }

private:
    std::string name_;
    const hydra_hw::ModelBase* model_;
};

/**
 * Hardware interface to perform calculations using the dynamic model of a robot.
 */
class HydraModelInterface : public hardware_interface::HardwareResourceManager<HydraModelHandle> {
};

} // namespace hydra_hw