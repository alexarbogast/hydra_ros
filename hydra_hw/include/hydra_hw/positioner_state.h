#pragma once

namespace hydra 
{
struct PositionerState {

    /**
     * \f$q\f$
     * Measured joint position. Unit: \f$[rad]\f$
     */
    double q;

    /**
     * \f$q_d\f$
     * Desired joint position. Unit: \f$[rad]\f$
     */
    double q_d;

    /**
     * \f$\dot{q}\f$
    * Measured joint velocity. Unit: \f$[\frac{rad}{s}]\f$
    */
    double dq;
    
    /**
     * \f$\dot{q}_d\f$
    * Desired joint velocity. Unit: \f$[\frac{rad}{s}]\f$
    */
    double dq_d;

    /**
    * \f$\ddot{q}_d\f$
    * Desired joint acceleration. Unit: \f$[\frac{rad}{s^2}]\f$
    */
    double ddq_d;
};

} // namespace hydra