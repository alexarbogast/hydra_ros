#pragma once

#include <gazebo_ros_control/robot_hw_sim.h>
#include <za_hw/za_state.h>
#include <za_hw/za_state_interface.h>
#include <za_hw/za_model_interface.h>
#include <za_hw/model_base.h>
#include <za_gazebo/joint.h>
#include <za_gazebo/controller_verifier.h>
#include <za_gazebo/statemachine.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hydra_hw/positioner_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <boost_sml/sml.hpp>

namespace hydra_gazebo
{
const double kDefaultTauExtLowpassFilter = 1.0;  // no filtering per default of tau_ext_hat_filtered

class PositionerHWSim : public gazebo_ros_control::RobotHWSim
{
public:
    //PositionerHWSim();

    /**
    * Initialize the simulated robot hardware and parse all supported transmissions.
    *
    * @param[in] robot_namespace the name of the robot passed inside the `<robotNamespace>` tag from
    * the URDF
    * @param[in] model_nh root node handle of the node into which this plugin is loaded (usually
    * Gazebo)
    * @param[in] parent the underlying gazebo model type of the robot which was added
    * @param[in] urdf the parsed URDF which should be added
    * @param[in] transmissions a list of transmissions of the model which should be simulated
    * @return `true` if initialization succeeds, `false` otherwise
    */
    bool initSim(const std::string& robot_namespace,
                 ros::NodeHandle model_nh,
                 gazebo::physics::ModelPtr parent,
                 const urdf::Model* const urdf,
                 std::vector<transmission_interface::TransmissionInfo> transmissions) override;

    /**
    * @param[in] time   the current (simulated) ROS time
    * @param[in] period the time step at which the simulation is running
    */
    void readSim(ros::Time time, ros::Duration period) override;

    /**
    * Pass the data send from controllers via the hardware interfaces onto the simulation.
    *
    * This will e.g. write the joint commands (torques or forces) to the corresponding joint in
    * Gazebo in each timestep. These commands are usually send via an
    * [EffortJointInterface](http://docs.ros.org/en/jade/api/hardware_interface/html/c++/classhardware__interface_1_1EffortJointInterface.html)
    *
    * @param[in] time   the current (simulated) ROS time
    * @param[in] period the time step at which the simulation is running
    */
    void writeSim(ros::Time time, ros::Duration period) override;

    /**
    * Switches the control mode of the robot arm
    * @param start_list list of controllers to start
    * @param stop_list list of controllers to stop
    */
    void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                  const std::list<hardware_interface::ControllerInfo>& stop_list) override;

    /**
    * Check (in non-realtime) if given controllers could be started and stopped from the current
    * state of the RobotHW with regard to necessary hardware interface switches and prepare the
    * switching. Start and stop list are disjoint. This handles the check and preparation, the actual
    * switch is commited in doSwitch().
    */
    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                       const std::list<hardware_interface::ControllerInfo>& /*stop_list*/) override;

    /**
     * Return the arm_id namespace parameter associated with this hardware interface
     * 
     * @return const std::string& 
     */
    inline const std::string& getArmID() const { return this->arm_id_; }

protected:
    void initJointStateHandle(const std::shared_ptr<za_gazebo::Joint>& joint);
    void initEffortCommandHandle(const std::shared_ptr<za_gazebo::Joint>& joint);
    void initVelocityCommandHandle(const std::shared_ptr<za_gazebo::Joint>& joint);
    void initPositionCommandHandle(const std::shared_ptr<za_gazebo::Joint>& joint);
    void initPositionerStateHandle(const std::string& robot,
                                   const urdf::Model& urdf,
                                   const transmission_interface::TransmissionInfo& transmission);

    void initServices(ros::NodeHandle& nh);

    void restartControllers(); 

    double positionControl(za_gazebo::Joint& joint, double setpoint, const ros::Duration& period);
    double velocityControl(za_gazebo::Joint& joint, double setpoint, const ros::Duration& period);

    template <int N>
    std::array<double, N> readArray(std::string param, std::string name = "") {
        std::array<double, N> x;

        std::istringstream iss(param);
        std::vector<std::string> values{std::istream_iterator<std::string>{iss},
                                        std::istream_iterator<std::string>{}};
        if (values.size() != N) {
        throw std::invalid_argument("Expected parameter '" + name + "' to have exactely " +
                                    std::to_string(N) + " numbers separated by spaces, but found " +
                                    std::to_string(values.size()));
        }
        std::transform(values.begin(), values.end(), x.begin(),
                    [](std::string v) -> double { return std::stod(v); });
        return x;
    }

    void forControlledJoint(
        const std::list<hardware_interface::ControllerInfo>& controllers,
        const std::function<void(za_gazebo::Joint& joint, const za_gazebo::ControlMethod&)>& f);


    bool robot_initialized_;
    std::unique_ptr<za_gazebo::ControllerVerifier> verifier_;

    std::string arm_id_;
    std::map<std::string, std::shared_ptr<za_gazebo::Joint>> joints_;

    gazebo::physics::ModelPtr robot_;

    hardware_interface::JointStateInterface jsi_;
    hardware_interface::EffortJointInterface eji_;
    hardware_interface::VelocityJointInterface vji_;
    hardware_interface::PositionJointInterface pji_;
    hydra_hw::PositionerStateInterface psi_;

    //boost::sml::sm<za_gazebo::StateMachine, boost::sml::thread_safe<std::mutex>> sm_;
    hydra::PositionerState robot_state_;
    
    double tau_ext_lowpass_filter_;

    ros::Publisher robot_initialized_pub_;
    ros::ServiceServer service_user_stop_;
    ros::ServiceClient service_controller_list_;
    ros::ServiceClient service_controller_switch_;
};

} // namespace hydra_gazebo