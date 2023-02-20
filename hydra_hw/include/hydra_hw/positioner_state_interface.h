#pragma once

#include <hydra_hw/positioner_state.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hydra_hw 
{
class PositionerStateHandle {
public:
    PositionerStateHandle() = delete;

    PositionerStateHandle(const std::string& name, 
                          hydra::PositionerState& positioner_state)
        : name_(name), positioner_state_(&positioner_state) {}

    /**
    * Gets the name of the state handle.
    *
    * @return Name of the state handle.
    */
    const std::string& getName() const noexcept { return name_; }

    /**
    * Gets the current robot state.
    *
    * @return Current robot state.
    */
    const hydra::PositionerState& getPositionerState() const noexcept { return *positioner_state_; }

private:
    std::string name_;
    const hydra::PositionerState* positioner_state_;
};

/**
* Hardware interface to read the complete positioner state.
*
* @see hydra::PositionerState for a description of the values included in the positioner state.
*/
class PositionerStateInterface : public hardware_interface::HardwareResourceManager<PositionerStateHandle> {
};

} // namespace hydra_hw