#pragma once

#include <za_hw/za_state.h>
#include <hydra_hw/positioner_state.h>

namespace hydra_hw {
class ModelBase {
public:
    virtual ~ModelBase() noexcept = default;

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
    virtual std::array<double, 42> positionerJacobian() const = 0;
};

} // namespace hydra_hw