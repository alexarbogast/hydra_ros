#pragma once

#include <hydra_hw/positioner_state.h>

namespace hydra
{
/**
 * Enumerates the six joints, the flange, and the end effector of a robot.
 */
enum class Frame {
  kPositioner,
  kJoint1,
  kJoint2,
  kJoint3,
  kJoint4,
  kJoint5,
  kJoint6,
  kFlange,
  kEndEffector
};

} // namespace za

namespace hydra_hw {
class ModelBase {
public:
    virtual ~ModelBase() noexcept = default;

    /**
     * Gets the 4x4 pose matrix for the given frame in positioner frame.
     *
     * The pose is represented as a 4x4 matrix in column-major format.
     *
     * @param[in] frame The desired frame.
     * @param[in] q Joint position.
     *
     * @return Vectorized 4x4 pose matrix, column-major.
     */
    virtual std::array<double, 16> pose(
        const std::string& arm_id,
        hydra::Frame frame,
        const std::array<double, 7>& q) 
        const = 0;

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
    virtual std::array<double, 42> positionerJacobian(const std::string& arm_id,
                                                      hydra::Frame frame,
                                                      const std::array<double, 7>& q) 
                                                      const = 0;
};

} // namespace hydra_hw